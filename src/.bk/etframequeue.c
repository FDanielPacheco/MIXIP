#include <stdio.h>
#include <stdint.h>
#include <semaphore.h>
#include <unistd.h>
#include <sys/mman.h>
#include <math.h>
#include <string.h>
#include <time.h>

#define SER_SZ_MAX 256
#define NMAX_SEGME 64
#define NMAX_FRAME 1

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

int
main( void ){
  mem_queue_t * queue = (mem_queue_t *) mmap( NULL, sizeof( mem_queue_t ), PROT_WRITE | PROT_READ , MAP_ANONYMOUS | MAP_SHARED , -1 , 0 );  
  memset( queue, 0, sizeof( mem_queue_t ) );

  sem_init( &queue->sem, 1, 1 );

  pid_t proc = fork( );
  
  if( 0 < proc ){
    // Start a process that will be giving information to the buffer, for simuation terms this data will be received by the user
    printf("Process for reading data from the user and add it to the ring buffer...\n");

    char buf[ 200 ];
    
    const uint8_t ser_payload_size = 16;

    for( ; ; ){
      printf("Give data to simulate a ethernet packet...\n-> ");
      fgets( buf, sizeof(buf) - 1, stdin );
      printPacket( buf, strlen(buf), "Code", 15 );

      uint8_t et_frame_len = strlen( buf );
      uint8_t nsegments = (uint8_t) ceil( (float) et_frame_len / (float) ser_payload_size );
      printf("Divided the frame on %hhd serial port segments...\n", nsegments);

      if( 0 < nsegments ){
        queue->frames[ queue->head ].nsegments = 0;
        for( uint8_t i = 0 ; i < nsegments ; ++i ){
          uint8_t bf_pos = i * ser_payload_size;
          size_t chunk_len = (et_frame_len - bf_pos > ser_payload_size) ? ser_payload_size : et_frame_len - bf_pos;

          sem_wait( &queue->sem );

          memcpy( queue->frames[ queue->head ]
            .frame[ queue->frames[ queue->head ].nsegments ]
              .data, 
            &buf[ bf_pos ], 
            chunk_len);

          if( i + 1 == nsegments )
            memset( &(queue->frames[ queue->head ]
              .frame[ queue->frames[ queue->head ].nsegments ]
                .data[ chunk_len ]), 
              0, 
              ser_payload_size - chunk_len);

          queue->frames[ queue->head ].frame[ queue->frames[ queue->head ].nsegments ].length = ser_payload_size;

          printPacket( queue->frames[ queue->head ]
            .frame[ queue->frames[ queue->head ].nsegments ]
              .data, ser_payload_size, "Payload", 16 );
          printf("Wrote:%d:%d...\n", i, chunk_len);
          
          queue->frames[ queue->head ].nsegments ++;

          if( nsegments > NMAX_SEGME )
            nsegments = 1;  

          sem_post( &queue->sem );
        }

        if( NMAX_FRAME == queue->head + 1 )
          queue->head = 0;
        else
          queue->head ++;

        if( NMAX_FRAME != queue->nframes )
          queue->nframes ++;
          
      }
    } 
  } // Process 1
  
  for( ; ; ){
    printf("Getting information and releasing it on a slow channel...\n");

    while( !queue->nframes )
      sleep( 1 );  // Poll time

    uint8_t n = queue->nframes;
    for( uint8_t i = 0 ; i < n ; ++i ){
      sem_wait( &queue->sem );
      eth_frame_t ef;
      memcpy( &ef, &(queue->frames[ queue->tail ]), sizeof(eth_frame_t) );
      queue->nframes --;
      sem_post( &queue->sem );
      
      if( NMAX_FRAME == queue->tail + 1 )
        queue->tail = 0;
      else 
        queue->tail ++;

      for( uint8_t j = 0 ; j < ef.nsegments ; ++j ){
        printPacket( ef.frame[ j ].data, ef.frame[ j ].length, "Info", 16 );
        sleep( 4 ); // Simulate a slow network
      }
    }


  } // Process 2


}