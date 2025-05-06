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
#include <libserialposix.h>
#include <signal.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <shmbuf.h>
#include <time.h>

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * Data structures
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/

struct arguments{
  char *name;                          // Instance name
  char *serial;                        // Serial port file path name, e.g., ttyUSB0
};

typedef struct{
  uint8_t data[ BUFSIZ ];              // Buffer that will contain the serial port received information
  size_t  len;                         // Current length of data
  sem_t   available;                   // Semaphore used to advertise that information is available
  sem_t   empty;                       // Semaphore used to stop a process while the other is processing the information
} buffer_t ;

struct memshared{
  buffer_t * buf;
  char       path[ 128 ];
};

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * Prototypes
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/

static error_t parseArgs( int key, char * arg, struct argp_state *state );
void cleanup( struct memshared * shr1, struct memshared * shr2, serial_manager_t * serial );
void signalhandler( int signum );

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * Global variables
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/

const char  * argp_program_version     = "tstdriver 1.0";
const char  * argp_program_bug_address = "fabio.d.pacheco@inesctec.pt";
static char   doc[ ]                   = "An example driver that uses the serial port specified by argument.";
static char   args_doc[ ]              = "Instance-Name Serial-Port";

static struct argp_option options[ ] = {
  {"name",      'n', "<instance-name>" , 0, "The instance name"                   , 0 },
  {"serial",    's', "<tty-file>"      , 0, "The serial port file for the device" , 0 },
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
cleanup( struct memshared * shr1, struct memshared * shr2, serial_manager_t * serial ){
  if( NULL != shr1 ) shm_close2( shr1->path, shr1->buf, sizeof(buffer_t) );
  if( NULL != shr2 ) shm_close2( shr2->path, shr2->buf, sizeof(buffer_t) );
  if( NULL != serial ) munmap( serial, sizeof(serial_manager_t) );
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
void
signalhandler( int signum __attribute__((unused)) ){
  signal_flag = 1;
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
void
printPacket( const uint8_t * pk, const size_t len, const char * cover, int columnsize ){
  if( !pk ) return;

  time_t t;
  struct tm * info;
  time( &t );
  info = localtime( &t );
  char tm[12]; strftime( tm, sizeof(tm), "%H:%M:%S", info );

  int cont = columnsize;
  printf("<-----%s Packet [%s]----->\n", cover, tm );
  for( size_t i = 0 ; i < len ; ++i ){
    if( !cont || (i + 1 == len) ){
      printf( "%04ld:%02X\n", i, pk[i] );
      cont = columnsize;
    }
    else {
      printf( "%04ld:%02X ", i, pk[i] );
      cont--;
    }
  }
  printf("<-----%s Packet----->\n\n", cover);
}

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * Main function
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/

int
main( int argc, char **argv ){
  struct arguments arguments = { .name = "", .serial = "" };
  argp_parse( &argp, argc, argv, 0, 0, &arguments );

  // Check the parameters
  if( !strcmp( arguments.name, "" ) || !strcmp( arguments.serial, "" ) ){
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

  // Configure the serial port
  serial_manager_t * serial = (serial_manager_t *) mmap( NULL, 
                                                         sizeof( serial_manager_t ), 
                                                         PROT_READ | PROT_WRITE, 
                                                         MAP_SHARED | MAP_ANONYMOUS,
                                                         -1,
                                                         0 );

  if( -1 == serial_open( &serial->sr, arguments.serial, 0, NULL ) ){
    perror("serial_open");
    cleanup( &tx, &rx, serial );
    return EXIT_FAILURE;
  }
  serial_set_baudrate( B19200, &serial->sr ); 
  serial_manage( serial, EPOLLIN | EPOLLOUT );
  const uint32_t pollrate = 10e3;
  int8_t result;
  
  pid_t proc = fork( );

  printf("[%d] Driver connected at %s sharing the objects in /dev/shm%s and /dev/shm%s ...\n", getpid( ), arguments.serial, tx.path, rx.path );
  for( ; ; ){
    if( !proc ){
      while( 0 != (result = serial_wait( serial, -1 ) ) ){
        uint8_t error = 0;

        if( -1 == result ){
          perror("serial_wait");
          error = 1;
        }
  
        if( -1 == usleep( pollrate ) ){
          perror("usleep");
          error = 1;
        }
  
        if( error ){
          serial_close( &serial->sr );
          cleanup( NULL, &rx, serial );
          return EXIT_FAILURE;
        }
      }
  
      while( 0 < ( rx.buf->len = serial_available( &serial->sr ) ) ){
        if( sizeof( rx.buf->data ) < rx.buf->len )
          rx.buf->len = serial_read( (char *) rx.buf->data, sizeof( rx.buf->data ), 0, sizeof( rx.buf->data ) - 1, &serial->sr );
        else
          rx.buf->len = serial_read( (char *) rx.buf->data, sizeof( rx.buf->data ), 0, rx.buf->len, &serial->sr );
  
        if( !rx.buf->len ){
          errno = EBUSY;
          perror("serial_read");
          serial_close( &serial->sr );
          cleanup( NULL, &rx, serial );
          return EXIT_FAILURE;
        }

        sem_post( &rx.buf->available );
        sem_wait( &rx.buf->empty );
      }
    }
    else{
      while( 1 != (result = serial_wait( serial, pollrate ) ) ){
        uint8_t error = 0;
  
        if( -1 == result ){
          perror("serial_wait");
          error = 1;
        }
  
        if( -1 == usleep( pollrate ) ){
          perror("usleep");
          error = 1;
        }
  
        if( error ){
          serial_close( &serial->sr );
          cleanup( &tx, NULL, serial );
          return EXIT_SUCCESS;
        }
      }
      
      sem_post( &tx.buf->empty);
      sem_wait( &tx.buf->available );
          
      size_t len = serial_write( &serial->sr, tx.buf->data , tx.buf->len );
      fflush( serial->sr.fp );
    }
      
    if( signal_flag ){
      printf("[%d] Driver interrupted with signal...\n", getpid( ) );
      serial_close( &serial->sr );
      cleanup( &tx, &rx, serial );

      if( !proc )
        kill( getppid( ), SIGTERM );
      else
        kill( proc, SIGTERM );

      return EXIT_SUCCESS;
    }

  }
}

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * End file
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
