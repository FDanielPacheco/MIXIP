/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * Introduction
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/

/**********************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************//**
 * @file      dualcom.c
 * 
 * @version   1.0
 *
 * @date      17-03-2025
 *
 * @brief     Creating a layer 2 switch, considering biderctional communication.
 *            Objective enabling pinging between two stations  
 *  
 * @author    Fábio D. Pacheco, 
 * @email     fabio.d.pacheco@inesctec.pt or pacheco.castro.fabio@gmail.com
 *
 * @copyright Copyright (c) [2025] [Fábio D. Pacheco]
 * 
 * @note      Manuals:
 * 
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * Imported libraries
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <math.h>
#include <errno.h>
#include <sys/epoll.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <semaphore.h>

#include <sys/socket.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <netinet/ether.h>
#include <linux/if_packet.h> 
#include <net/if.h>

#include <argp.h>
#include <time.h>

#include <libserialposix.h>

#include "cobs.h"
#include "ringbuffer.h"

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * Argument and program identification
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/

const char *argp_program_version     = "dualcom 1.0";
const char *argp_program_bug_address = "fabio.d.pacheco@inesctec.pt";
static char doc[ ]                   = "Dualcom a program that enables two computer to speak using IP stack, using a serial cable and ethernet cable";
static char args_doc[]               = "INTERFACE_ETHERNET SERIAL_TTY";

static struct argp_option options[ ] = {
  {"ethernet", 'e', "<interface>"     , 0, "The ethernet interface for the program to interact", 0 },
  {"serial",   's', "<tty-file>"      , 0, "The serial port file for the program to interact"  , 0 },
  {"method",   'm', "<method-number>" , 0, "The method number, (1) there is no data segmentation only a stream of information (2) segmentation but no flow control (3) segmentation with flow control"  , 0 },
  {"direction",'d', "<direction>"     , 0, "The medium capabilities, (1) unidirectional (2) bidirectional", 0 },
  { 0 }
};

struct arguments{
  char    *ether_ifc;              // Ethernet interaface, e.g, eth0
  char    *serial_ifc;             // Serial port inferface path name, e.g., ttyUSB0
  uint8_t method;
  uint8_t direction;
};

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * Definitions
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/

#define ETH_PCKT_SIZE      65535
#define SER_PCKT_SIZE      32
#define CHK_SIZE           256
#define RING_SIZE          4096

#define S_SOF              0x00
#define S_EOF              S_SOF
#define E_SOF              S_SOF
#define E_EOF              S_SOF

#define TIMEOUT            5000

#define SER_SZ_MAX         256
#define NMAX_SEGME         256
#define NMAX_FRAME         16

// Pause frame definitions
#define PAUSE_OPCODE       0x0001
#define PAUSE_PAYLOAD_LEN  42

#define READY_READ         1
#define READY_WRITE        2

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * Data structures
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/

typedef struct{
  uint8_t sr_packet:1;
  uint8_t et_packet:1;
  uint8_t ongoing:1;
  uint8_t sr_first:1;
  uint8_t et_first:1;
  uint8_t sr_failed:1;
  uint8_t et_pause:1;
  uint8_t et_suc:1;
} flag_t ;
 
typedef struct{
  int                fd;
  struct epoll_event ev;
} serial_event_t;

typedef struct{
  serial_t        * sr;
  serial_event_t    ep;
} serial_manager_t;

typedef struct{
  uint8_t      data[ SER_SZ_MAX ];
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

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * Prototypes
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/

 static error_t    parseArgs( int key, char * arg, struct argp_state *state );
 serial_manager_t  * serial_connect( const char * pathname );
 int               rawsocket( const char * interface );
 
/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * Global variables
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/

static struct argp argp = { options, parseArgs, args_doc, doc, NULL, NULL, NULL };

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * Functions
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
static error_t
parseArgs( int key, char * arg, struct argp_state *state ){
  struct arguments *arguments = state->input;
  switch( key ){
    case 'e':
      arguments->ether_ifc = arg;
      break;

    case 's':
      arguments->serial_ifc = arg;
      break;

    case 'm':
      arguments->method = (uint8_t) atoi(arg);
      break;

    case 'd':
      arguments->direction = (uint8_t) atoi(arg);
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

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
serial_manager_t *
serial_connect( const char * pathname ){
  if( !pathname ){
    perror( "pathname" );
    return NULL;
  }

  serial_manager_t * mn = (serial_manager_t *) mmap( NULL, sizeof( serial_manager_t ), PROT_WRITE | PROT_READ , MAP_ANONYMOUS | MAP_SHARED , -1 , 0 );
  if( MAP_FAILED == mn ){
    perror("mmap");
    return NULL;
  }

  serial_config_t cfg;
  serial_default_config( &cfg );
  cfg.baudrate = B19200;
  cfg.timeout = 0;
  cfg.minBytes = 0;

  mn->sr = serial_open( pathname, 0, &cfg );
  if( !mn->sr ){
    perror( "serial_open" );
    return NULL;
  }

  mn->ep.fd = epoll_create1( 0 );
  if( -1 == mn->ep.fd ){
    perror("epoll_create");
    serial_close( mn->sr );
    return NULL;
  }

  mn->ep.ev.events = EPOLLIN | EPOLLOUT;
  mn->ep.ev.data.fd = mn->ep.fd;
  int result = epoll_ctl( mn->ep.fd, EPOLL_CTL_ADD, mn->sr->fd, &(mn->ep.ev) );
  if( -1 == result ){
    perror("epoll_ctl");
    serial_close( mn->sr );
    return NULL;
  }

  serial_print( 1, mn->sr );
  printf( "Packet timeout: %d ms\n", TIMEOUT );
  return mn;  
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int
serial_wait( serial_manager_t * mn ){
  int nfds = epoll_wait( mn->ep.fd, &(mn->ep.ev), 1, TIMEOUT );
  if( -1 == nfds ){
    perror("epoll_wait");
    return -1;
  }

  if( EPOLLIN & mn->ep.ev.events )
    return READY_READ;

  if( EPOLLOUT & mn->ep.ev.events )
    return READY_WRITE;

  return -1;
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

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int 
rawsocket( const char * interface ){
  if( !interface ){
    perror( "interface" );
    return -1;
  }

  struct sockaddr_ll socket_address;

  // Create a raw socket using SOCK_RAW and ETH_P_ALL protocol.
  // SOCK_RAW: Creates a socket that receives raw packets.
  // ETH_P_ALL: Receives all Ethernet protocols (not just IP).
  int rs = socket( AF_PACKET, SOCK_RAW, htons(ETH_P_ALL) );
  if( 0 > rs ){
    perror("Socket creation failed");
    return -1;
  }  

  // Get the NIC index, ifr.ifr_ifindex will be filled upon success
  int ifindex = (int) if_nametoindex( interface );
  if( !ifindex ){
    perror("if_nametoindex failed");
    close(rs);
    return -1;
  }

  // Bind the socket to the interface.
  memset( &socket_address, 0, sizeof(socket_address) );
  socket_address.sll_family = AF_PACKET;
  socket_address.sll_protocol = htons(ETH_P_ALL); 
  socket_address.sll_ifindex = ifindex;
  if( 0 > bind( rs, (struct sockaddr *) &socket_address, sizeof(socket_address) ) ){
    perror("Socket bind failed");
    close( rs );
    return -1;
  }

  return rs;
}

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * Main function
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/

int
main( int argc, char **argv ){
  struct arguments arguments = { 
    .ether_ifc  = "usb0", 
    .serial_ifc = "/dev/ttyUSB0",
    .method = 3,
    .direction = 2, 
  };

  argp_parse( &argp, argc, argv, 0, 0, &arguments );

  serial_manager_t * mn = serial_connect( arguments.serial_ifc );
  if( !mn )
    return EXIT_FAILURE;
  
  int sraw = rawsocket( arguments.ether_ifc );
  // Add another process for reading the data coming from the socket
  // and check if the packet is sent from the device it self (check MAC)

  printf("Listening on %s...\n", arguments.ether_ifc);
  printf("Listening on %s...\n", arguments.serial_ifc);

  // Creation of the serial ring buffer
  ring_buffer_t * serial_ring = (ring_buffer_t *) mmap( NULL, RING_SIZE, PROT_READ | PROT_WRITE, MAP_ANONYMOUS | MAP_SHARED, -1, 0 );
  uint8_t       * serial_buffer = (uint8_t *) mmap( NULL, RING_SIZE, PROT_READ | PROT_WRITE, MAP_ANONYMOUS | MAP_SHARED, -1, 0 );
  ring_buffer_init( serial_ring, (char *) serial_buffer, RING_SIZE );

  // Output buffer
  mem_queue_t * queue = (mem_queue_t *) mmap( NULL, sizeof( mem_queue_t ), PROT_WRITE | PROT_READ , MAP_ANONYMOUS | MAP_SHARED , -1 , 0 );  
  memset( queue, 0, sizeof( mem_queue_t ) );
  sem_init( &queue->sem, 1, 1 );

  // Variables
  uint8_t chunk[ CHK_SIZE ];
  size_t  chunk_len;

  uint8_t serial_packet[ SER_PCKT_SIZE ];
  size_t  serial_packet_len = 0;

  uint8_t ethernet_packet[ ETH_PCKT_SIZE ];
  size_t  ethernet_packet_len = 0;

  flag_t  flag = {0};

  // Sliding window protocol (GoBackN)
  uint8_t const window_n = 2;
  uint8_t window_size = (uint8_t) pow( 2, window_n) - 1;
  uint8_t window_selector = 0;
  uint8_t window_ack = 0;

  // Ring size threshold
  size_t ring_size_warning = (size_t) ceil( (float) RING_SIZE * 0.8 );

  // Read and Write process
  pid_t proc = fork( );

  if( !proc ){
    proc = fork( );

    if( !proc ){
      printf("Reading from serial port process with PID: %d...\n", getpid( ) );
      for( ; ; ){
        while( READY_READ != serial_wait( mn ) )
          usleep( 1000 );
    
        while( 0 < ( chunk_len = serial_available( mn->sr ) ) ){
          if( CHK_SIZE < chunk_len )
            chunk_len = serial_read( (char *) chunk, sizeof(chunk), 0, CHK_SIZE, mn->sr );
          else
            chunk_len = serial_read( (char *) chunk, sizeof(chunk), 0, chunk_len, mn->sr );

          if( !chunk_len ){
            printf("Unkown error...\n");
            exit( EXIT_FAILURE );
          }
          
          ring_buffer_queue_arr( serial_ring, (char *) chunk, chunk_len );    
 
          if( ring_size_warning < ring_buffer_num_items( serial_ring ) ){
            usleep( 10000 );       
            printf("Waiting ...\n");
          }
          
        }
      }          
    }

    printf("Processing serial port information process with PID: %d...\n", getpid( ) );
    size_t sz = 0;
    
    for( ; ; ){
      while( !( sz = ring_buffer_num_items( serial_ring ) ) ) 
        usleep( 1000 );

      for( size_t i = 0 ; i < sz ; ++i ){   
        ring_buffer_dequeue( serial_ring, (char *) &chunk[0] );
        
        if( !flag.sr_packet && S_SOF == chunk[0] ){
          flag.sr_packet = 1;
          flag.sr_first = 1;
          serial_packet_len = 0;
        }

        if( flag.sr_packet && serial_packet_len < SER_PCKT_SIZE ){
          serial_packet[ serial_packet_len++ ] = chunk[0];
        }

        if( flag.sr_packet && S_EOF == chunk[0] && serial_packet_len == 2 ){
          printf("Out of sync, trying to sync...\n");
          serial_packet_len --;
          flag.sr_first = 1;
        }
            
        if( flag.sr_packet && S_EOF == chunk[0] && !flag.sr_first ){     
          cobs_ret_t result = cobs_decode_tinyframe( &serial_packet[1], serial_packet_len - 1 );            

          if( COBS_RET_SUCCESS == result ){
            flag.sr_packet = 0;

            ( window_ack + 1 > window_size ) ? window_ack = 0 : window_ack++ ;

            serial_packet_len -= 4;
            memmove( serial_packet, &serial_packet[3], serial_packet_len );
            memset( &serial_packet[serial_packet_len], 0, 4 );                        

            for( uint8_t k = 0 ; k < serial_packet_len ; ++k ){
              if( !flag.et_packet && E_SOF == serial_packet[k] ){
                flag.et_packet = 1;
                flag.et_first = 1;
                flag.ongoing = 1;
                ethernet_packet_len = 0;   
              }

              if( flag.et_packet && ethernet_packet_len < ETH_PCKT_SIZE ){
                ethernet_packet[ ethernet_packet_len++ ] = serial_packet[k];
              }

              if( E_EOF == serial_packet[k] && flag.et_packet && !flag.et_first ){                
                size_t ethernet_packet_decoded_len;
                uint8_t ethernet_packet_decoded[ ethernet_packet_len - 3 ];
                result = cobs_decode( &ethernet_packet[1], ethernet_packet_len - 1, ethernet_packet_decoded, ethernet_packet_len - 1, &ethernet_packet_decoded_len );
                if( COBS_RET_SUCCESS == result ){
                  write( sraw, ethernet_packet_decoded, ethernet_packet_decoded_len );

                  flag.et_packet = 0;
                  flag.ongoing = 0;
                  ethernet_packet_len = 0;  
                }
                else{
                  printf("Error decoding the ethernet packet...\n");        
                  flag.et_packet = 0;
                  ethernet_packet_len = 0;
                }
              }

              if( flag.et_first )
                flag.et_first = 0;
  
            }          
          }              
          else{
            printf("Error decoding the serial packet...\n");

            // Necessary to request the packet that failed again
            flag.sr_packet = 0;
            flag.et_packet = 0;
            flag.ongoing = 0;
            ethernet_packet_len = 0;  
          } 
        }
        
        if( flag.sr_first )
          flag.sr_first = 0;

      }
    }
  }

  if( 1 < arguments.direction ){
    proc = fork( );

    if( !proc ){
      printf("Converting network interface information to serial port packets, process with PID: %d...\n", getpid( ) );
      for( ; ; ){
        ethernet_packet_len = (size_t) read( sraw, ethernet_packet, sizeof(ethernet_packet) );
        if( 0 < ethernet_packet_len ){
          // struct ether_header * eth_header = (struct ether_header *) ethernet_packet;
    
          size_t ethernet_packet_encoded_len = ethernet_packet_len + 3 + 50; // 50 is necessary because of larger packets
          uint8_t ethernet_packet_encoded[ ethernet_packet_encoded_len ];
          ethernet_packet_encoded[ 0 ] = 0x00;
          size_t ethernet_packet_encoded_len_ret = 0;
          cobs_ret_t result = cobs_encode( ethernet_packet, ethernet_packet_len, &ethernet_packet_encoded[1], ethernet_packet_encoded_len - 1, &ethernet_packet_encoded_len_ret );
          ethernet_packet_encoded_len = ethernet_packet_encoded_len_ret + 1;
    
          if( COBS_RET_SUCCESS == result ){      
            size_t ser_payload_size = SER_PCKT_SIZE - 4;
            serial_packet[ 0 ] = E_EOF;
        
            int ser_nsegm = (int) ceil( (float) ethernet_packet_encoded_len / (float) ser_payload_size );
            // printf( "Number of Frames: %d\n", ser_nsegm );
    
            if( 0 < ser_nsegm ){
              uint8_t serial_packets[ ser_nsegm ][ ser_payload_size ];
              
              for( int j = 0 ; j < ser_nsegm ; ++j ){
                size_t index = (size_t) j * ser_payload_size;
                if( j == ser_nsegm - 1 ){
                  size_t left = ethernet_packet_encoded_len - index;
                  memcpy( serial_packets[j], &ethernet_packet_encoded[index], left );
                  memset( &serial_packets[j][left], IGNCR, ser_payload_size - left );        
                }
                else
                  memcpy( serial_packets[j], &ethernet_packet_encoded[index], ser_payload_size );
              }

              sem_wait( &queue->sem );  
              for( int j = 0 ; j < ser_nsegm ; ++j ){
                serial_packet[ 1 ] = COBS_TINYFRAME_SENTINEL_VALUE;
                serial_packet[ SER_PCKT_SIZE - 1 ] = COBS_TINYFRAME_SENTINEL_VALUE;

                serial_packet[ 2 ] = ((window_ack & 0x07) << 3) | (window_selector & 0x07);
                ( window_selector + 1 > window_size ) ? window_selector = 0 : window_selector++ ;

                memcpy( &serial_packet[3], serial_packets[j], ser_payload_size );
                result = cobs_encode_tinyframe( &serial_packet[1], sizeof(serial_packet) - 1 );

                if( COBS_RET_SUCCESS == result ){
                  if( !j )
                    queue->frames[ queue->head ].nsegments = 0;

                  flag.et_suc = 1;
                  
                  memcpy( queue->frames[ queue->head ]
                    .frame[ queue->frames[ queue->head ].nsegments ]
                      .data,
                    serial_packet,
                    SER_PCKT_SIZE);

                  queue->frames[ queue->head ].frame[ queue->frames[ queue->head ].nsegments ].length = SER_PCKT_SIZE;
                  queue->frames[ queue->head ].nsegments ++;

                } 
                else
                  printf( "Failed (%d) to encode...\n", j );
              }

              if( flag.et_suc ){
                flag.et_suc = 0;
                
                if( NMAX_FRAME <= queue->head + 1 )
                  queue->head = 0;
                else
                  queue->head ++;
        
                if( NMAX_FRAME >= queue->nframes + 1 ){
                  queue->nframes ++;
                }
                
              }
              sem_post( &queue->sem );
              
            }
          }
        }
      }
    }

    printf("Sending serial port packets, process with PID: %d...\n", getpid( ) );
    for( ; ; ){
	    uint8_t nframes = 0;
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
        usleep( 10000 );
      }

      for( size_t i = 0 ; i < nframes ; ++i ){
        sem_wait( &queue->sem );
        eth_frame_t ef;
        memcpy( &ef, &(queue->frames[ queue->tail ]), sizeof(eth_frame_t) );

        if( 0 < queue->nframes )
          queue->nframes --;

        if( NMAX_FRAME == queue->tail + 1 )
          queue->tail = 0;
        else 
          queue->tail ++;        

        sem_post( &queue->sem );
           
        for( size_t j = 0 ; j < ef.nsegments ; ++j ){
          while( READY_WRITE != serial_wait( mn ) )
            usleep( 100 );
          
          // printPacket( ef.frame[ j ].data, ef.frame[ j ].length, "Info", 16 );
          size_t len = serial_write( mn->sr, ef.frame[ j ].data , ef.frame[ j ].length );
          fflush( mn->sr->fp );
          // printf("Wrote to the serial port %ld bytes, from frame %ld and segment %ld...\n", len, i, j );
          // usleep( 8000 );
        }
      }

	  printf("nframes:%d\n", queue->nframes );
      
    }
  }

  serial_close( mn->sr );
  close( sraw );

  return EXIT_SUCCESS;
}

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * End file
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
