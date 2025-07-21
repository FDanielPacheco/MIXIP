/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * Introduction
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/

/**********************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************//**
 * @file      eth2ser.c
 * 
 * @version   1.0
 *
 * @date      02-05-2025
 *
 * @brief     Captures information from the network interface specified by the -i argument.
 *            Encodes the captured information and sends it to the serial port specified by the -s argument.
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
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <argp.h>
#include <unistd.h>
#include <errno.h>
#include <semaphore.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <cobs.h>
#include <rawsoc.h>
#include <math.h>
#include <shmbuf.h>
#include <mixip.h>
#include <signal.h>

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * Definitions
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/

#define SEGM_MSZ           256         // Maximum size of each serial port segment  
#define NMAX_SEGME         256         // Maximum number of segments
#define NMAX_FRAME         256         // Maximum number of frames

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * Data structures
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/

struct arguments{
  char * name;                         // Instance name
  char * interface;                    // Network interaface file descriptor
};
  
typedef struct{
  uint8_t      data[ SEGM_MSZ ];
  size_t       length;
} ser_segm_t;

typedef struct{
  ser_segm_t   frame[ NMAX_SEGME ];
  uint8_t      nsegments;
} eth_frame_t ;
  
typedef struct{
  eth_frame_t  frames[ NMAX_FRAME ];
  uint8_t      nframes;
  uint8_t      head;
  uint8_t      tail;
  sem_t        sem;
} mem_queue_t ;

struct memshared{
  translator_parameters_t * tra;
  char                      path[ NAME_MAX ];
};

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * Prototypes
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/

static error_t parseArgs( int key, char * arg, struct argp_state *state );
void cleanup( buffer_t * buf, mem_queue_t * queue, struct memshared * param );
void signalhandler( int signum );

int8_t et_encode( buffer_t * src );

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * Global variables
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/

const char  * argp_program_version     = "eth2ser 1.0";
const char  * argp_program_bug_address = "fabio.d.pacheco@inesctec.pt";
static char   doc[ ]                   = "Ethernet to Serial, is a program that captures and encodes information from the network interface and sends to the serial port.";
static char   args_doc[ ]              = "Instance-Name Network-Interface-FileDescriptor";

static struct argp_option options[ ] =  {
  {"name",      'n', "<instance-name>"    , 0, "The instance name"                           , 0 },
  {"interface", 'i', "<interface>"        , 0, "A file descriptor for the network interface" , 0 },
  { 0 }
};
  
static struct argp argp = { options, parseArgs, args_doc, doc, NULL, NULL, NULL };

volatile sig_atomic_t signal_flag = 0;

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * Functions
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
void
signalhandler( int signum __attribute__((unused)) ){
  signal_flag = 1;
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
static error_t
parseArgs( int key, char * arg, struct argp_state * state ){
  struct arguments * arguments = state->input;
  switch( key ){
    case 'n':
      arguments->name = arg;
      break;

    case 'i':
      arguments->interface = arg;
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
int8_t 
et_encode( buffer_t * src ){
  if( !src ){
    errno = EINVAL;
    return -1;
  }
  
  buffer_t tmp;
  cobs_ret_t result = cobs_encode( src->data, 
                                   src->len, 
                                   tmp.data, 
                                   sizeof(tmp.data),
                                   &tmp.len
                                  );
  
  if( COBS_RET_SUCCESS == result ){
    memcpy( &src->data[ 1 ], tmp.data, tmp.len );
    src->len = tmp.len + 1;
    return 1;
  }
  
  return 0;
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
void 
cleanup( buffer_t * buf, mem_queue_t * queue, struct memshared * param ){
  if( NULL != buf ) shm_close2( NULL, buf, sizeof(buffer_t) );
  if( NULL != param ) shm_close2( param->path, param->tra, sizeof(translator_parameters_t) );
  if( NULL != queue ) munmap( queue, sizeof(mem_queue_t) );
}

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * Main function
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/

int
main( int argc, char **argv ){
  struct arguments arguments = { .name = "", .interface = "" };
  argp_parse( &argp, argc, argv, 0, 0, &arguments );

  // Check the parameters
  if( !strcmp( arguments.name, "" ) || !strcmp( arguments.interface, "" ) ){
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

  // Shared memory definitions
  char path[ 128 ];
  snprintf( path, sizeof(path), "/%s_tx", arguments.name );
  //                                                               __ Inherit the creator permissions
  //                                                              /
  buffer_t * serial = shm_open2( path, sizeof(buffer_t), O_RDWR, 0 );
  if( !serial ){
    perror("shm_open2 (serial)");
    return EXIT_FAILURE;
  }
  
  // Open the shared memory for the driver to update the serial link segment size and other parameters
  snprintf( path, sizeof(path), "/%s_tx_param", arguments.name );
  printf("[%d] Translator sharing the objects in /dev/shm%s ...\n", getpid( ), path );
  struct memshared parameters;
  strncpy( parameters.path, path, sizeof(parameters.path) );
  parameters.tra = shm_open2( path, sizeof(translator_parameters_t), O_CREAT | O_RDWR, 0666 );
  if( !parameters.tra ){
    perror("shm_open2 (param)");
    cleanup( serial, NULL, NULL );
    return EXIT_FAILURE;
  }
  parameters.tra->boot = 0;
  
  // Connect to the interface
  int nic = atoi( arguments.interface );
  if( !nic ){
    perror("atoi");
    cleanup( serial, NULL, &parameters );
    return EXIT_FAILURE;
  }
  
  // Create the circle buffer oriented to frame
  mem_queue_t * queue = (mem_queue_t *) mmap( NULL, sizeof( mem_queue_t ), PROT_WRITE | PROT_READ , MAP_ANONYMOUS | MAP_SHARED , -1 , 0 );  
  if( MAP_FAILED == queue ){
    perror("mmap");
    cleanup( serial, NULL, &parameters );
    return EXIT_FAILURE;
  }
  memset( queue, 0, sizeof( mem_queue_t ) );
  sem_init( &queue->sem, MAP_SHARED, 1 );
  
  const long activation_poll_delay = 2;
  while( !parameters.tra->boot ){
    sleep( activation_poll_delay );
    printf("[%d] Waiting for eth2ser (%s) activation ...\n", getpid( ), path );
  }

  int8_t result;

  // Create the multiprocess
  pid_t proc = fork( );
  if( !proc ){    
    const uint8_t limiter = 0x00;
    buffer_t frame;
    
    for( ; ; ){
      for( ; ; ){
        printf("[%d] Capture from NIC process with a %d ring buffer and compact to %d bytes segments ...\n", getpid( ), parameters.tra->size_rb, parameters.tra->size_sls );
        fflush( stdout );

        if( signal_flag ){
          printf("[%d] eth2ser process 2 closed...\n", getpid( ) );
          return EXIT_SUCCESS;
        }

        frame.len = (size_t) read( nic, frame.data, sizeof(frame.data) ); 
        if( 0 < frame.len ){
          if( 1 > ( result = et_encode( &frame ) ) ){
            if( -1 == result ){
              perror("et_encode");
              cleanup( serial, queue, &parameters );
              return EXIT_FAILURE;
            }
            printf("[%d] Failed to encode ethernet frame...\n", getpid( ) );
            break;   
          }
          
          uint8_t ring_buffer_size = parameters.tra->size_rb;
          uint8_t segment_size = parameters.tra->size_sls;
          uint8_t payload_sz = segment_size - 1 - 1 - 1 - 1;
          //                      SOF Limiter_/  /    \   \_ EOF Limiter
          //                           Window  _/      \_ COBS overhead

          frame.data[ 0 ] = limiter;
          int ser_nsegm = (int) ceil( (float) frame.len / (float) payload_sz );
          if( 0 < ser_nsegm ){
            sem_wait( &queue->sem );  
            
            int i;
            for( i = 0 ; i < ser_nsegm ; ++i ){
              if( !i )
                queue->frames[ queue->head ].nsegments = 0;
              
              size_t index = (size_t) i * payload_sz;

              uint8_t * data = queue->frames[ queue->head ]
                                .frame[ queue->frames[ queue->head ].nsegments ]
                                .data;
              uint8_t * payload = &data[ 3 ];

              if( i + 1 == ser_nsegm ){
                size_t left = frame.len - index;
                memcpy( payload, &frame.data[ index ], left );
                //     Payload_/
                memset( &payload[ left ], IGNCR, payload_sz - left );
              }
              else
                memcpy( payload, &frame.data[ index ], payload_sz );

              data[ 0 ] = limiter;
              data[ 1 ] = COBS_TINYFRAME_SENTINEL_VALUE;
              data[ 2 ] = IGNCR;
              data[ segment_size - 1 ] = COBS_TINYFRAME_SENTINEL_VALUE;
              
              if( COBS_RET_SUCCESS != cobs_encode_tinyframe( &data[ 1 ], segment_size - 1 ) ){
                printf("[%d] Failed to encode serial segment...\n", getpid( ) );
                break;   
              }
              
              queue->frames[ queue->head ]
                .frame[ queue->frames[ queue->head ].nsegments ].length = segment_size;
              
              queue->frames[ queue->head ].nsegments ++;
              if( queue->frames[ queue->head ].nsegments + 1 >= NMAX_SEGME ){
                printf("[%d] Warning segments will get overwritten...\n", getpid( ) );
                queue->frames[ queue->head ].nsegments = 0;
              }              
            }
            if( i == ser_nsegm ){
              if( ring_buffer_size <= queue->head + 1 )
                queue->head = 0;
              else
                queue->head ++;

              if( ring_buffer_size >= queue->nframes + 1 )
                queue->nframes ++;
              
              fflush( stdout );
            }

            sem_post( &queue->sem );
          }

          
        }

      }
      if( signal_flag ){
        printf("[%d] eth2ser process 2 closed...\n", getpid( ) );
        return EXIT_SUCCESS;
      }
    }

  }

  printf("[%d] Write to serial port process ...\n", getpid( ) );

  const uint32_t pollrate = 10e3;
  uint8_t nframes = 0;

  for( ; ; ){
    nframes = 0;
    uint8_t ring_buffer_size = parameters.tra->size_rb;
    for( ; ; ){
      if( signal_flag ){
        kill( proc, SIGTERM );
        cleanup( serial, queue, &parameters );
        printf("[%d] eth2ser process 1 closed...\n", getpid( ) );
        return EXIT_SUCCESS;
      }

      sem_wait( &queue->sem );  
      
      if( 0 < queue->nframes ){
        nframes = ( queue->head - queue->tail + ring_buffer_size ) % ring_buffer_size ;
        if( !nframes )
          nframes = ring_buffer_size;
        
        sem_post( &queue->sem );
        break;
      }

      sem_post( &queue->sem );
      usleep( pollrate );
    }
    
    for( size_t i = 0 ; i < nframes ; ++i ){
      sem_wait( &queue->sem );
      eth_frame_t frame;
      memcpy( &frame, &(queue->frames[ queue->tail ]), sizeof(frame) );
      
      if( 0 < queue->nframes )
        queue->nframes --;
      
      if( ring_buffer_size <= queue->tail + 1 )
        queue->tail = 0;
      else 
        queue->tail ++;        

      sem_post( &queue->sem );
      
      for( size_t j = 0 ; j < frame.nsegments ; ++j ){
        sem_wait( &serial->empty );
        memcpy( serial->data, frame.frame[ j ].data, frame.frame[ j ].length );
        serial->len = frame.frame[ j ].length;
        sem_post( &serial->available );
      }
    }

  }
   
}

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * End file
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
