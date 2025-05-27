/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * Introduction
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/

/**********************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************//**
 * @file      tstdriver.c
 * 
 * @version   1.0
 *
 * @date      02-05-2025
 *
 * @brief     Emulator of a possible driver that uses the serial port
 *    
 * @author    Fábio D. Pacheco, 
 * @email     fabio.d.pacheco@inesctec.pt or pacheco.castro.fabio@gmail.com
 *
 * @copyright Copyright (c) [2025] [Fábio D. Pacheco]
 * 
 * @note      
 * 
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * Imported libraries
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <argp.h>
#include <unistd.h>
#include <semaphore.h>
#include <serialposix.h>
#include <signal.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <shmbuf.h>
#include <time.h>
#include <dlfcn.h>
#include <mixip.h>

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * Data structures
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/

struct arguments{
  char * name;                         // Instance name
  char * serial;                       // Serial port file path name, e.g., ttyUSB0
  char * driver;                       // Path to the driver, e.g., driver.so
};

struct memshared{
  buffer_t * buf;
  char       path[ 128 ];
};

struct driver{
  int (* dsetup)( serial_manager_t * ); // int dsetup( serial_managet_t * serial );
  int (* dloop)( flow_t * );            // int dloop( flow_t * flow );
  int (* dread)( buffer_t * );          // int dread( buffer_t * buf );
  int (* dwrite)( buffer_t * );         // int dwrite( buffer_t * buf );
};

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * Prototypes
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/

static error_t parseArgs( int key, char * arg, struct argp_state *state );
void cleanup( struct memshared * shr1, struct memshared * shr2, serial_manager_t * serial, flow_t * stop, void * handle );
void signalhandler( int signum );

void * load_driver( const char * path, struct driver * dev );

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * Global variables
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/

const char  * argp_program_version     = "serial broker 1.0";
const char  * argp_program_bug_address = "fabio.d.pacheco@inesctec.pt";
static char   doc[ ]                   = "Serial port broker specified by argument.";
static char   args_doc[ ]              = "Instance-Name Serial-Port";

static struct argp_option options[ ] = {
  {"name",      'n', "<instance-name>" , 0, "The instance name"                   , 0 },
  {"serial",    's', "<tty-file>"      , 0, "The serial port file for the device" , 0 },
  {"driver",    'd', "<driver-file>"   , 0, "The driver file for the device *.so" , 0 },
  { 0 }
};
   
static struct argp argp = { options, parseArgs, args_doc, doc, NULL, NULL, NULL };

volatile sig_atomic_t signal_flag = 0;

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * Functions
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
static error_t
parseArgs( int key, char * arg, struct argp_state * state ){
  struct arguments * arguments = state->input;
  switch( key ){
    case 'n':
      arguments->name = arg;
      break;

    case 's':
      arguments->serial = arg;
      break;
    
    case 'd':
      arguments->driver = arg;

    case ARGP_KEY_ARG:
      // Index of each extra argument not specified by an option key
      break;
 
    case ARGP_KEY_END:
      // Number of extra arguments not specified by an option key
      if( 0 != state->arg_num )
        argp_usage( state );
      break;

    default:
      return ARGP_ERR_UNKNOWN;
  }
  return 0;
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
void 
cleanup( struct memshared * shr1, struct memshared * shr2, serial_manager_t * serial, flow_t * stop, void * handle ){
  if( NULL != shr1 ) shm_close2( shr1->path, shr1->buf, sizeof(buffer_t) );
  if( NULL != shr2 ) shm_close2( shr2->path, shr2->buf, sizeof(buffer_t) );
  if( NULL != serial ) munmap( serial, sizeof(serial_manager_t) );
  if( NULL != stop ) munmap( stop, sizeof(flow_t) );
  if( NULL != handle ) dlclose( handle );
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
void
signalhandler( int signum __attribute__((unused)) ){
  signal_flag = 1;
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
void * 
load_driver( const char * path, struct driver * dev ){
  if( !path || !dev ){
    errno = EINVAL;
    return NULL;
  }

  char * err = NULL;
  void * handle = dlopen( path, RTLD_NOW | RTLD_GLOBAL );
  //   Check for all dependencies_/           \
  //                                           \_Made available for symbol resolution of subsequently loaded shared objects...
  if( !handle ){
    fprintf( stderr, "[%d] error dlopen %s...\n", getpid( ), dlerror( ) );
    return NULL;
  }

  // Clear any existing error
  dlerror( );

  char * func[ ] = { "dsetup", "dloop", "dread", "dwrite" };
  void (** devs[ ])( void ) = { ( void (**)(void) ) &dev->dsetup,
                                ( void (**)(void) ) &dev->dloop,
                                ( void (**)(void) ) &dev->dread,
                                ( void (**)(void) ) &dev->dwrite };

  for( int i = 0 ; i < 4 ; ++i ){
    // According to the ISO C standard, POSIX.1-2008
    * ( void ** ) devs[ i ] = dlsym( handle, func[ i ] );
    err = dlerror( );
    if( NULL != err ){
      fprintf( stderr, "[%d] in template %s, %s function is missing (%s)...\n", getpid( ), path, func[ i ], dlerror( ) );
      return NULL;
    }
  }  

  return handle;
}

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * Main function
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/

int
main( int argc, char **argv ){
  struct arguments arguments = { .name = "", .serial = "", .driver = "" };
  argp_parse( &argp, argc, argv, 0, 0, &arguments );

  // Check the parameters
  if( !strcmp( arguments.name, "" ) || !strcmp( arguments.serial, "" ) || !strcmp( arguments.driver, "" ) ){
    errno = EINVAL;
    perror("missing arguments");
    return EXIT_FAILURE;
  }

  // Establish the signal handler
  struct sigaction act;
  act.sa_handler = signalhandler;
  sigemptyset( &act.sa_mask );
  act.sa_flags = 0;

  if( -1 == sigaction( SIGTERM, &act, NULL ) || 
      -1 == sigaction( SIGINT, &act, NULL )  ||
      -1 == sigaction( SIGQUIT, &act, NULL ) ){
    perror( "sigaction" );
    return EXIT_FAILURE;
  }
  
  struct memshared tx;
  snprintf( tx.path, sizeof(tx.path), "/%s_tx", arguments.name );  
  tx.buf = (buffer_t *) shm_open2( tx.path, sizeof(buffer_t), O_CREAT | O_RDWR, 0666 );
  if( !tx.buf ){
    perror("shm_open2");
    shm_unlink( tx.path );
    return EXIT_FAILURE;
  }
  sem_init( &tx.buf->empty, 1, 0 );
  sem_init( &tx.buf->available, 1, 0 );

  struct memshared rx;
  snprintf( rx.path, sizeof(rx.path), "/%s_rx", arguments.name );  
  rx.buf = (buffer_t *) shm_open2( rx.path, sizeof(buffer_t), O_CREAT | O_RDWR, 0666 );
  if( !rx.buf ){
    perror("shm_open2");
    shm_close2( tx.path, tx.buf, sizeof(buffer_t) );
    shm_unlink( rx.path );
    return EXIT_FAILURE;
  }
  sem_init( &rx.buf->empty, 1, 0 );
  sem_init( &rx.buf->available, 1, 0 );

  // Capability for the driver 
  flow_t * flow = (flow_t * ) mmap( NULL, sizeof( flow_t ), PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS, -1, 0 );
  if( MAP_FAILED == flow ){
    perror("mmap flow_t");
    cleanup( &tx, &rx, NULL, NULL, NULL );
    return EXIT_FAILURE;
  }
  flow->network = 0;
  sem_init( &flow->rx_done, 1, 0 );
  sem_init( &flow->rx_wait, 1, 0 );
  sem_init( &flow->tx_done, 1, 0 );
  sem_init( &flow->tx_wait, 1, 0 );

  // Configure the serial port
  serial_manager_t * serial = (serial_manager_t *) mmap( NULL, 
                                                         sizeof( serial_manager_t ), 
                                                         PROT_READ | PROT_WRITE, 
                                                         MAP_SHARED | MAP_ANONYMOUS,
                                                         -1,
                                                         0 );

  if( -1 == serial_open( &serial->sr, arguments.serial, 0, NULL ) ){
    perror("serial_open");
    cleanup( &tx, &rx, serial, flow, NULL );
    return EXIT_FAILURE;
  }
  serial_manage( serial, EPOLLIN | EPOLLOUT );

  const uint32_t pollrate_us = 10e3;
  const uint16_t pollrate_ms = 10;
  int8_t result;
  
  printf("[%d] Serial broker connected at %s sharing the objects in /dev/shm%s and /dev/shm%s ...\n", getpid( ), arguments.serial, tx.path, rx.path );

  // Load the driver
  struct driver driver;
  void * driverlib;
  if( NULL == ( driverlib = load_driver( arguments.driver, &driver ) ) ){
    serial_close( &serial->sr );
    cleanup( &tx, &rx, serial, flow, NULL );
    return EXIT_FAILURE;
  }

  // Perform the driver setup configuration, give the serial object 
  if( -1 == driver.dsetup( serial ) ){                                          
    printf("[%d] Driver reported an error during setup, closing...\n", getpid( ) );
    serial_close( &serial->sr );
    cleanup( &tx, &rx, serial, flow, driverlib );
    return EXIT_FAILURE;
  }

  pid_t proc = fork( );

  if( !proc ){
    proc = fork( );
    for( ; ; ){
      if( !proc ){
        while( 0 != (result = serial_wait( serial, pollrate_ms ) ) ){
          if( -1 == usleep( pollrate_us ) ){
            perror("usleep");
            serial_close( &serial->sr );
            cleanup( &tx, &rx, serial, flow, driverlib );
            return EXIT_FAILURE;
          }
  
          if( flow->network ){
            sem_post( &flow->rx_done );
            sem_wait( &flow->rx_wait );
          }
        }
    
        if( flow->network ){
          sem_post( &flow->rx_done );
          sem_wait( &flow->rx_wait );
        }

        result = 0;

        while( 0 < ( rx.buf->len = serial_available( &serial->sr ) ) ){
          if( sizeof( rx.buf->data ) < rx.buf->len )
            rx.buf->len = serial_read( (char *) rx.buf->data, sizeof( rx.buf->data ), 0, sizeof( rx.buf->data ) - 1, &serial->sr );
          else
            rx.buf->len = serial_read( (char *) rx.buf->data, sizeof( rx.buf->data ), 0, rx.buf->len, &serial->sr );

          if( !rx.buf->len ){
            errno = EBUSY;
            perror("serial_read");
            result = 1;
          }

          // Give to the driver the information that arrived, let him decide what to do with it
          if( -1 == driver.dread( rx.buf ) ){                                           
            printf("[%d] Driver reported an error during read operation, closing...\n", getpid( ) );
            result = 1;
          }

          if( result ){
            serial_close( &serial->sr );
            cleanup( NULL, &rx, serial, flow, driverlib );
            return EXIT_FAILURE;
          }

          sem_post( &rx.buf->available );
          sem_wait( &rx.buf->empty );
        }
      }
      else{
        while( 1 != (result = serial_wait( serial, pollrate_ms ) ) ){
          if( -1 == usleep( pollrate_us ) ){
            perror("usleep");
            serial_close( &serial->sr );
            cleanup( &tx, NULL, serial, flow, driverlib );
            return EXIT_SUCCESS;
          }
  
          if( flow->network ){
            sem_post( &flow->tx_done );
            sem_wait( &flow->tx_wait );
          } 
        }
   
        if( flow->network ){
          sem_post( &flow->rx_done );
          sem_wait( &flow->rx_wait );
        }
        
        sem_post( &tx.buf->empty);
        sem_wait( &tx.buf->available );

        // Give to the driver the information that arrived from the nic, let him decide what to do with it
        if( -1 == driver.dwrite( tx.buf ) ){                                           
          printf("[%d] Driver reported an error during write operation, closing...\n", getpid( ) );
          serial_close( &serial->sr );
          cleanup( &tx, NULL, serial, flow, driverlib );
          return EXIT_FAILURE;
        }

        size_t len = serial_write( &serial->sr, tx.buf->data , tx.buf->len );
        fflush( serial->sr.fp );
      }
        
      if( signal_flag ){
        printf("[%d] Serial broker interrupted with signal...\n", getpid( ) );
        serial_close( &serial->sr );
        cleanup( &tx, &rx, serial, flow, driverlib );
  
        if( !proc )
          kill( getppid( ), SIGTERM );
        else
          kill( proc, SIGTERM );
  
        return EXIT_SUCCESS;
      }  
    }
  }

  printf("[%d] Driver launched %s...\n", getpid( ), arguments.driver);
  result = 0;
  for( ; ; ){
    // Driver development loop
    if( -1 == driver.dloop( flow ) ){                                           
      printf("[%d] Driver reported an error during loop, closing...\n", getpid( ) );
      result = 1;
    }

    if( signal_flag ){
      printf("[%d] Serial broker driver loop interrupted with signal...\n", getpid( ) );
      result = 1;
    }  

    if( result ){
      serial_close( &serial->sr );
      cleanup( &tx, &rx, serial, flow, driverlib );
      kill( proc, SIGTERM );
      return EXIT_SUCCESS;
    }
    
  }
}

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * End file
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
