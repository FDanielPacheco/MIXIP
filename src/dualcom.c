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

#include <sys/socket.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <netinet/ether.h>
#include <linux/if_packet.h> 
#include <net/if.h>

#include <argp.h>

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
  {"ethernet", 'e', "<interface>", 0, "The ethernet interface for the program to interact", 0 },
  {"serial",   's', "<tty-file>" , 0, "The serial port file for the program to interact"  , 0 },
  { 0 }
};

struct arguments{
  char *ether_ifc;              // Ethernet interaface, e.g, eth0
  char *serial_ifc;             // Serial port inferface path name, e.g., ttyUSB0
};

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * Definitions
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/

#define ETH_PCKT_SIZE      65535
#define SER_PCKT_SIZE      32
#define CHK_SIZE           256
#define RING_SIZE          256

#define S_SOF              0x00
#define S_EOF              S_SOF
#define E_SOF              S_SOF
#define E_EOF              S_SOF

#define TIMEOUT            5000

#define ACTION             1

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * Data structures
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/

typedef struct{
  uint8_t sr_packet:1;
  uint8_t et_packet:1;
  uint8_t ongoing:1;
  uint8_t sr_first:1;
  uint8_t et_first:1;
} flag_t ;
 
typedef struct{
  int                fd;
  struct epoll_event ev;
} serial_event_t;

typedef struct{
  serial_t        * sr;
  serial_event_t    ep;
} serial_manager_t;

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

  serial_manager_t * mn = (serial_manager_t *) malloc( sizeof( serial_manager_t ) );
  if( !mn ){
    perror("malloc");
    return NULL;
  }

  serial_config_t cfg;
  serial_default_config( &cfg );
  cfg.baudrate = B115200;
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

  mn->ep.ev.events = EPOLLIN;
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
  return (0 < nfds);
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
void
printPacket( const uint8_t * pk, const size_t len, const char * cover, int columnsize ){
  if( !pk ) return;

  int cont = columnsize;
  printf("<-----%s Packet----->\n", cover);
  for( size_t i = 0 ; i < len ; ++i ){
    if( !cont || (i + 1 == len) ){
      printf( "%03ld:%02X\n", i, pk[i] );
      cont = columnsize;
    }
    else {
      printf( "%03ld:%02X ", i, pk[i] );
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
  if( 0 > bind( rs, (struct sockaddr *)&socket_address, sizeof(socket_address) ) ){
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
    .serial_ifc = "/dev/ttyUSB0" 
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
  ring_buffer_t serial_ring;
  uint8_t       serial_buffer[ RING_SIZE ];
  ring_buffer_init( &serial_ring, (char *) serial_buffer, RING_SIZE );

  // Variables
  uint8_t chunk[ CHK_SIZE ];
  size_t  chunk_len;

  uint8_t serial_packet[ SER_PCKT_SIZE ];
  size_t  serial_packet_len = 0;

  uint8_t ethernet_packet[ ETH_PCKT_SIZE ];
  size_t  ethernet_packet_len = 0;

  flag_t  flag = {0};

  if( ACTION ){
    for( ; ; ){
      if( !serial_wait( mn ) ){
        if( flag.ongoing ){
          printf("Data discarted and ring buffer clean\n");
          flag.ongoing = 0;
          flag.et_packet = 0;
          flag.sr_packet = 0;
        }
      }
  
      if( 0 < ( chunk_len = serial_available( mn->sr ) ) ){
        chunk_len = serial_read( (char *) chunk, sizeof(chunk), 0, chunk_len, mn->sr );

        printf("Length:%d...\n", chunk_len );
        printPacket( chunk, chunk_len, "Chunk", 10 );

        if( !chunk_len ){
          printf("Unkown error...\n");
          exit( EXIT_FAILURE );
        }
          
        ring_buffer_queue_arr( &serial_ring, (char *) chunk, chunk_len );            
      
        size_t sz = ring_buffer_num_items( &serial_ring );
        for( size_t i = 0 ; i < sz ; ++i ){
          ring_buffer_dequeue( &serial_ring, (char *) &chunk[0] );
          
          if( !flag.sr_packet && S_SOF == chunk[0] ){
            flag.sr_packet = 1;
            flag.sr_first = 1;
            serial_packet_len = 0;
          }

          if( flag.sr_packet && serial_packet_len < SER_PCKT_SIZE ){
            serial_packet[ serial_packet_len++ ] = chunk[ 0 ];
          }
          
          if( flag.sr_packet && S_EOF == chunk[0] && !flag.sr_first ){     
            printPacket( serial_packet, serial_packet_len, "Serial Encode", 10 );
            cobs_ret_t result = cobs_decode_tinyframe( &serial_packet[1], serial_packet_len - 1 );            

            if( COBS_RET_SUCCESS == result ){
              flag.sr_packet = 0;
              
              serial_packet_len -= 3;
              memmove( serial_packet, &serial_packet[2], serial_packet_len );
              memset( &serial_packet[serial_packet_len], 0, 3 );                        

              printPacket( serial_packet, serial_packet_len, "Serial Decode", 10 );

              for( uint8_t k = 0 ; k < serial_packet_len ; ++k ){
                if( !flag.et_packet && E_SOF == serial_packet[k] ){
                  flag.et_packet = 1;
                  flag.ongoing = 1;
                  flag.et_first = 1;
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

                    printPacket( ethernet_packet_decoded, ethernet_packet_decoded_len, "Ethernet", 10 );
                    flag.et_packet = 0;
                    flag.ongoing = 0;
                    ethernet_packet_len = 0;  
                  }
                  else printf("Error decoding the ethernet packet...\n");        
                }

                if( flag.et_first )
                  flag.et_first = 0;
    
              }          
            }
            else{
              printf("Error decoding the serial packet...\n");
              exit( EXIT_FAILURE );
            } 

          }
          
          if( flag.sr_first )
            flag.sr_first = 0;

        }
      }
      
    }

  }

  for( ; ; ){
    ethernet_packet_len = (size_t) recvfrom( sraw, ethernet_packet, sizeof(ethernet_packet), 0, NULL, NULL );
    if( 0 < ethernet_packet_len ){
      struct ether_header * eth_header = (struct ether_header *) ethernet_packet;
      printf("Received packet: %zd bytes\n", ethernet_packet_len);

      printf("Source MAC: %02x:%02x:%02x:%02x:%02x:%02x\n", eth_header->ether_shost[0], eth_header->ether_shost[1], eth_header->ether_shost[2], eth_header->ether_shost[3], eth_header->ether_shost[4], eth_header->ether_shost[5]);
      printf("Destination MAC: %02x:%02x:%02x:%02x:%02x:%02x\n", eth_header->ether_dhost[0], eth_header->ether_dhost[1], eth_header->ether_dhost[2], eth_header->ether_dhost[3], eth_header->ether_dhost[4], eth_header->ether_dhost[5]);
      printf("Ethertype: 0x%04x\n", ntohs(eth_header->ether_type));
      printPacket( ethernet_packet, ethernet_packet_len, "Ethernet", 10 );

      size_t ethernet_packet_encoded_len = ethernet_packet_len + 3;
      uint8_t ethernet_packet_encoded[ ethernet_packet_encoded_len ];
      ethernet_packet_encoded[ 0 ] = 0x00;
      size_t ethernet_packet_encoded_len_ret = 0;
      cobs_ret_t result = cobs_encode( ethernet_packet, ethernet_packet_len, &ethernet_packet_encoded[1], ethernet_packet_encoded_len - 1, &ethernet_packet_encoded_len_ret );
      ethernet_packet_encoded_len = ethernet_packet_encoded_len_ret + 1;
      printPacket( ethernet_packet_encoded, ethernet_packet_encoded_len, "Ethernet Encoded", 10 );

      if( COBS_RET_SUCCESS == result ){      
        size_t ser_payload_size = SER_PCKT_SIZE - 3;
        serial_packet[ 0 ] = E_EOF;
    
        int ser_nframes = (int) ceil( (float) ethernet_packet_encoded_len / (float) ser_payload_size );
        printf( "Number of Frames: %d\n", ser_nframes );

        if( 0 < ser_nframes ){
          uint8_t serial_packets[ ser_nframes ][ ser_payload_size ];
          
          for( int j = 0 ; j < ser_nframes ; ++j ){
            size_t index = (size_t) j * ser_payload_size;
            if( j == ser_nframes - 1 ){
              size_t left = ethernet_packet_encoded_len - index;
              memcpy( serial_packets[j], &ethernet_packet_encoded[index], left );
              memset( &serial_packets[j][left], IGNCR, ser_payload_size - left );        
            }
            else
              memcpy( serial_packets[j], &ethernet_packet_encoded[index], ser_payload_size );
          }
      
          for( int j = 0 ; j < ser_nframes ; ++j ){
            serial_packet[ 1 ] = COBS_TINYFRAME_SENTINEL_VALUE;
            serial_packet[ SER_PCKT_SIZE - 1 ] = COBS_TINYFRAME_SENTINEL_VALUE;
            memcpy( &serial_packet[2], serial_packets[j], ser_payload_size );
            result = cobs_encode_tinyframe( &serial_packet[1], sizeof(serial_packet) - 1 );

            printPacket( serial_packet, sizeof(serial_packet), "Serial Encoded", 10 );

            if( COBS_RET_SUCCESS == result ){
              size_t len = serial_write( mn->sr, serial_packet, sizeof(serial_packet) );
              fflush( mn->sr->fp );
              printf( "Sending (%d) a serial packet encoded with %ld bytes...\n", j, len );
            } 
            else
            if( COBS_RET_ERR_BAD_ARG == result )
              printf( "Failed bad arg (%d) to encode...\n", j );
            else
            if( COBS_RET_ERR_BAD_PAYLOAD== result )
              printf( "Failed bad payload (%d) to encode...\n", j );
            else
              printf( "Failed (%d) to encode...\n", j );
          }
        }
      }

    }

  }

  serial_close( mn->sr );
  close( sraw );

  return EXIT_SUCCESS;
}
