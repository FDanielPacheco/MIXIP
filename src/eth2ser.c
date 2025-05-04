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
#include <cobs.h>
#include <rawsoc.h>
#include <math.h>
#include <shmbuf.h>
#include <libserialposix.h>
#include <time.h>

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * Definitions
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/

#define SEGM_MSZ           256         // Maximum size of each serial port segment  
#define NMAX_SEGME         256         // Maximum number of segments
#define NMAX_FRAME         16          // Maximum number of frames

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * Data structures
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/

struct arguments{
  char *name;                          // Instance name
  char *interface;                     // Network interaface, e.g, eth0
  char *serial;                        // Serial port file path name, e.g., ttyUSB0
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

typedef struct{
  uint8_t data[ BUFSIZ ];              // Buffer that will contain the serial port received information
  size_t  len;                         // Current length of data
  sem_t   available;                   // Semaphore used to advertise that information is available
  sem_t   empty;                       // Semaphore used to stop a process while the other is processing the information
} buffer_t ;

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * Prototypes
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/

static error_t parseArgs( int key, char * arg, struct argp_state *state );

int8_t et_encode( buffer_t * src );

void cleanup( buffer_t * buf, mem_queue_t * queue );

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * Global variables
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/

const char  * argp_program_version     = "eth2ser 1.0";
const char  * argp_program_bug_address = "fabio.d.pacheco@inesctec.pt";
static char   doc[ ]                   = "Ethernet to Serial, is a program that captures and encodes information from the network interface and sends to the serial port.";
static char   args_doc[ ]              = "Instance-Name Network-Interface Serial-Port";

static struct argp_option options[ ] = {
  {"name",      'n', "<instance-name>" , 0, "The instance name"                                                      , 0 },
  {"interface", 'i', "<interface>"     , 0, "The ethernet interface for the ser2eth to send information"             , 0 },
  {"serial",    's', "<tty-file>"      , 0, "The serial port file for the ser2eth to capture and decode information" , 0 },
  { 0 }
};
  
static struct argp argp = { options, parseArgs, args_doc, doc, NULL, NULL, NULL };

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

    case 'i':
      arguments->interface = arg;
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
cleanup( buffer_t * buf, mem_queue_t * queue ){
  if( NULL != buf ) shm_close2( NULL, buf, sizeof(buffer_t) );
  if( NULL != queue ) munmap( queue, sizeof(mem_queue_t) );
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
  struct arguments arguments = { .name = "", .interface = "", .serial = "" };
  argp_parse( &argp, argc, argv, 0, 0, &arguments );

  // Shared memory definitions
  char path[ 128 ];
  snprintf( path, sizeof(path), "/%s_tx", arguments.name );
  //                                                          __ Inherit the creator permissions
  //                                                         /
  buffer_t * serial = shm_open2( path, sizeof(buffer_t), O_RDWR, 0 );
  if( !serial ){
    perror("shm_open2");
    return EXIT_FAILURE;
  }
  
  // Connect to the interface
  int nic = rawsocket( arguments.interface );
  if( -1 == nic ){
    perror("rawsocket");
    cleanup( serial, NULL );
    return EXIT_FAILURE;
  }
  
  // Create the circle buffer oriented to frame
  mem_queue_t * queue = (mem_queue_t *) mmap( NULL, sizeof( mem_queue_t ), PROT_WRITE | PROT_READ , MAP_ANONYMOUS | MAP_SHARED , -1 , 0 );  
  if( MAP_FAILED == queue ){
    perror("mmap");
    cleanup( serial, NULL );
    return EXIT_FAILURE;
  }
  memset( queue, 0, sizeof( mem_queue_t ) );
  sem_init( &queue->sem, 1, 1 );
  
  int8_t result;

  // Create the multiprocess
  pid_t proc = fork( );
  if( !proc ){
    printf("[%d] Capture from NIC process...\n", getpid( ) );
    
    const uint8_t limiter = 0x00;
    const uint8_t segment_size = 32;
    const uint8_t payload_sz = segment_size - 1 - 1 - 1 - 1;
    //                            SOF Limiter_/  /    \   \_ EOF Limiter
    //                                 Window  _/      \_ COBS overhead
    buffer_t frame;
    
    for( ; ; ){
      for( ; ; ){
        frame.len = (size_t) read( nic, frame.data, sizeof(frame.data) ); 
        if( 0 < frame.len ){
          if( 1 > ( result = et_encode( &frame ) ) ){
            if( -1 == result ){
              perror("et_encode");
              cleanup( serial, queue );
              return EXIT_FAILURE;
            }
            printf("[%d] Failed to encode ethernet frame...\n", getpid( ) );
            break;   
          }
          
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
              if( NMAX_FRAME <= queue->head + 1 )
                queue->head = 0;
              else
                queue->head ++;

              if( NMAX_FRAME >= queue->nframes + 1 )
                queue->nframes ++;
              
              fflush( stdout );
            }

            sem_post( &queue->sem );
          }

          
        }

        
      }
    }

  }

  printf("[%d] Write to serial port process...\n", getpid( ) );

  const uint32_t pollrate = 10e3;
  uint8_t nframes = 0;

  for( ; ; ){
    nframes = 0;
    for( ; ; ){
      sem_wait( &queue->sem );  
      
      if( 0 < queue->nframes ){
        nframes = ( queue->head - queue->tail + NMAX_FRAME ) % NMAX_FRAME ;
        if( !nframes )
          nframes = NMAX_FRAME;
        
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
      
      if( NMAX_FRAME <= queue->tail + 1 )
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
