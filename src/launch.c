/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * Introduction
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/

/**********************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************//**
 * @file      launch.c
 * 
 * @version   1.0
 *
 * @date      05-05-2025
 *
 * @brief     Launches the necessary files to provide MIXIP
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
#include <unistd.h>
#include <string.h>
#include <rawsoc.h>
#include <argp.h>
#include <signal.h>
#include <sys/wait.h>
#include <libxml2/libxml/parser.h>
#include <libxml2/libxml/tree.h>
#include <linux/limits.h>

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * Data structures
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/

struct arguments{
  char * path;
};

struct comms{
  char name[NAME_MAX];  
  char path[NAME_MAX];
  char driver[NAME_MAX];
};

struct device{
  struct comms def;
  struct comms tx;
  struct comms rx;
};

struct interface{
  char def[NAME_MAX];
  char rx[NAME_MAX];
  char tx[NAME_MAX];
};

typedef struct{
  struct interface nic;
  int              nicopt;             // 0 -> default, 1 -> tx rx 
  struct device    dev;
  int              devopt;             // 0 -> default, 1 -> tx rx
} cfgxml_t;

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * Prototypes
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/

static error_t parseArgs( int key, char * arg, struct argp_state *state );
void signalhandler( int signum );
int get_xml_data( const char *filename, cfgxml_t * data );

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * Global variables
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/

const char  * argp_program_version     = "mixip 1.0";
const char  * argp_program_bug_address = "fabio.d.pacheco@inesctec.pt";
static char   doc[ ]                   = "Multimodal Interface eXtension for IP";
static char   args_doc[ ]              = "Descriptive-file-xml";

static struct argp_option options[ ] = {
  {"path",              'p', "<path>" , 0, "The path to descriptive xml file" , 0 },
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
    case 'p':
      arguments->path = arg;
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
signalhandler( int signum __attribute__((unused)) ){
  signal_flag = 1;
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int
get_xml_data( const char *filename, cfgxml_t * data ){
  xmlDoc * docfile = xmlReadFile( filename, NULL, 0 );
  if( !docfile ){
    perror("xmlReadFile");
    return -1;
  }
  
  xmlNode * root_element = xmlDocGetRootElement(docfile);
  if( !root_element ){
    xmlFreeDoc( docfile );
    return -1;
  }

  xmlNode * serial_node = NULL;
  xmlNode * network_node = NULL;

  if( !strcmp( (char *) root_element->name , "mixip" ) ){
    for( xmlNode * current_node = root_element->children ; current_node != NULL ; current_node = current_node->next ){
      if( XML_ELEMENT_NODE == current_node->type ){
        if( !strcmp( (char *)current_node->name, "serial" ) )
          serial_node = current_node;
        if( !strcmp( (char *)current_node->name, "network" ) )              
          network_node = current_node;
      }
    }
  }

  for( ; ; ){
    xmlNode * tx_node = NULL;
    xmlNode * rx_node = NULL;
    xmlNode * def_node = NULL;

    if( serial_node ){
      for( xmlNode * current_node = serial_node->children ; current_node != NULL ; current_node = current_node->next ){
        if( XML_ELEMENT_NODE == current_node->type ){
          if( !strcmp( (char *)current_node->name, "default" ) )
            def_node = current_node;
          if( !strcmp( (char *)current_node->name, "tx" ) )
            tx_node = current_node;
          if( !strcmp( (char *)current_node->name, "rx" ) )              
            rx_node = current_node;
        }
      }
      
      if( def_node ){
        xmlNode * name_node = NULL;
        xmlNode * device_node = NULL;
        xmlNode * driver_node = NULL;

        for( xmlNode * current_node = def_node->children ; current_node != NULL ; current_node = current_node->next ){
          if( XML_ELEMENT_NODE == current_node->type ){
            if( !strcmp( (char *)current_node->name, "name" ) )
              name_node = current_node;
            if( !strcmp( (char *)current_node->name, "device" ) )
              device_node = current_node;
            if( !strcmp( (char *)current_node->name, "driver" ) )
              driver_node = current_node;
          }
        }

        if( !name_node || !device_node || !driver_node ){
          xmlFreeDoc( docfile );
          return 0;      
        }
        else{
          xmlChar * content = NULL; 
        
          content = xmlNodeGetContent( name_node );
          if( NULL != content )
            strncpy( data->dev.def.name, (const char *) content, sizeof( data->dev.def.name ) );

          content = xmlNodeGetContent( device_node );
          if( NULL != content )
            strncpy( data->dev.def.path, (const char *) content, sizeof( data->dev.def.path ) );
 
          content = xmlNodeGetContent( driver_node );
          if( NULL != content )
            strncpy( data->dev.def.driver, (const char *) content, sizeof( data->dev.def.driver ) );
  
          xmlFree( content );
          data->devopt = 0;
          break;         
        }
          
      }
      
      xmlNode * nodes[ ] = { tx_node, rx_node };
      struct comms * comms[ ] = { &data->dev.tx, &data->dev.rx };
      int both = 0;

      for( int i = 0 ; i < 2 ; ++i ){
        if( nodes[ i ] ){
          xmlNode * name_node = NULL;
          xmlNode * device_node = NULL;
          xmlNode * driver_node = NULL;

          for( xmlNode * current_node = nodes[ i ]->children ; current_node != NULL ; current_node = current_node->next ){
            if( XML_ELEMENT_NODE == current_node->type ){
              if( !strcmp( (char *)current_node->name, "name" ) )
                name_node = current_node;
              if( !strcmp( (char *)current_node->name, "device" ) )
                device_node = current_node;
              if( !strcmp( (char *)current_node->name, "driver" ) )
                driver_node = current_node;
            }
          }

          if( !name_node || !device_node || !driver_node ){
            xmlFreeDoc( docfile );
            return 0;      
          }
          else{
            xmlChar * content = NULL; 
    
            content = xmlNodeGetContent( name_node );
            if( NULL != content )
              strncpy( comms[ i ]->name, (const char *) content, sizeof( comms[ i ]->name ) );

            content = xmlNodeGetContent( device_node );
            if( NULL != content )
              strncpy( comms[ i ]->path, (const char *) content, sizeof( comms[ i ]->path ) );

            content = xmlNodeGetContent( driver_node );
            if( NULL != content )
              strncpy( comms[ i ]->driver, (const char *) content, sizeof( comms[ i ]->driver ) );
                
            xmlFree( content );
            both ++;
          }
            
        }
      }
      if( 2 == both ){
        data->devopt = 1;
        break;
      }
      else
        return 0;
    }
    else
      return 0;
  }

  for( ; ; ){
    xmlNode * tx_node = NULL;
    xmlNode * rx_node = NULL;
    xmlNode * def_node = NULL;

    if( network_node ){
      for( xmlNode * current_node = network_node->children ; current_node != NULL ; current_node = current_node->next ){
        if( XML_ELEMENT_NODE == current_node->type ){
          if( !strcmp( (char *)current_node->name, "default" ) )
            def_node = current_node;
          if( !strcmp( (char *)current_node->name, "tx" ) )
            tx_node = current_node;
          if( !strcmp( (char *)current_node->name, "rx" ) )              
            rx_node = current_node;
        }
      }
      
      if( def_node ){
        xmlNode * name_node = NULL;

        for( xmlNode * current_node = def_node->children ; current_node != NULL ; current_node = current_node->next ){
          if( XML_ELEMENT_NODE == current_node->type ){
            if( !strcmp( (char *)current_node->name, "name" ) )
              name_node = current_node;
          }
        }

        if( !name_node ){
          xmlFreeDoc( docfile );
          return 0;      
        }
        else{
          xmlChar * content = NULL; 

          content = xmlNodeGetContent( name_node );
          if( NULL != content )
            strncpy( data->nic.def, (const char *) content, sizeof( data->nic.def) );

          xmlFree( content );
          data->nicopt = 0;
          break;         
        }
          
      }
      
      xmlNode * nodes[ ] = { tx_node, rx_node };
      char * comms[ ] = { data->nic.tx, data->nic.rx };
      int both = 0;
      
      for( int i = 0 ; i < 2 ; ++i ){
        if( nodes[ i ] ){
          xmlNode * name_node = NULL;

          for( xmlNode * current_node = nodes[ i ]->children ; current_node != NULL ; current_node = current_node->next ){
            if( XML_ELEMENT_NODE == current_node->type ){
              if( !strcmp( (char *)current_node->name, "name" ) )
                name_node = current_node;
            }
          }

          if( !name_node ){
            xmlFreeDoc( docfile );
            return 0;      
          }
          else{
            xmlChar * content = NULL; 
  
            content = xmlNodeGetContent( name_node );
            if( NULL != content )
              strncpy( comms[ i ], (const char *) content, sizeof( data->nic.tx ) );
  
            xmlFree( content );
            both ++;
          }
            
        }
      }
      if( 2 == both ){
        data->nicopt = 1;
        break;
      }
    }
    else
      return 0;
  }

  xmlFreeDoc(docfile);
  xmlCleanupParser( );
  return 1;
}

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * Main function
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/

int
main( int argc, char **argv ){
  struct arguments arguments;
  memset( &arguments, '\0', sizeof( struct arguments ) );
  argp_parse( &argp, argc, argv, 0, 0, &arguments );

  // Check the current parameters to decide the flow of launch
  cfgxml_t cfg;
  if( 1 > get_xml_data( arguments.path, &cfg ) ){
    perror("failed to parse");
    return -1;
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

  pid_t opened_proc[ 4 ];
  int status, n_proc = 0;

  const char path_serbroker[ ] = "serbroker.out";
  if( cfg.devopt ){
    printf("Have to initialize two serial interface instances ...\n");
    printf("tx(%s): %s, rx(%s): %s ...\n", cfg.dev.tx.name, cfg.dev.tx.path, cfg.dev.rx.name, cfg.dev.rx.path );

    printf("Opening serbroker: ./%s %s %s %s %s %s %s\n", path_serbroker, "-s", cfg.dev.tx.path, "-n", cfg.dev.tx.name, "-d", cfg.dev.tx.driver );
    opened_proc[ n_proc ] = fork( );
    if( !opened_proc[ n_proc ] ){
      if( -1 == execl( path_serbroker,
                       path_serbroker,
                       "-s",
                       cfg.dev.tx.path,
                       "-n",
                       cfg.dev.tx.name,
                       "-d",
                       cfg.dev.tx.driver,
                       (char *)NULL
      )){
        printf("[%d] ", getpid( ));
        fflush( stdout );
        perror( "execl" );
      }
      return EXIT_FAILURE;    
    }
    n_proc ++;

    printf("Opening serbroker: ./%s %s %s %s %s %s %s\n", path_serbroker, "-s", cfg.dev.rx.path, "-n", cfg.dev.rx.name, "-d", cfg.dev.rx.driver );
    opened_proc[ n_proc ] = fork( );
    if( !opened_proc[ n_proc ] ){
      if( -1 == execl( path_serbroker,
                       path_serbroker,
                       "-s",
                       cfg.dev.rx.path,
                       "-n",
                       cfg.dev.rx.name,
                       "-d",
                       cfg.dev.rx.driver,
                       (char *)NULL
      )){
        printf("[%d] ", getpid( ));
        fflush( stdout );
        perror( "execl" );
      }
      return EXIT_FAILURE;    
    }    
    n_proc ++;
  }
  else{
    printf("Have to initialize one serial interface instances ...\n");
    printf("default(%s): %s ...\n", cfg.dev.def.name, cfg.dev.def.path );
    printf("Opening serbroker: ./%s %s %s %s %s %s %s\n", path_serbroker, "-s", cfg.dev.def.path, "-n", cfg.dev.def.name, "-d", cfg.dev.def.driver );
    opened_proc[ n_proc ] = fork( );
    if( !opened_proc[ n_proc ] ){
      if( -1 == execl( path_serbroker, 
                       path_serbroker,
                       "-s",
                       cfg.dev.def.path,
                       "-n",
                       cfg.dev.def.name,
                       "-d",
                       cfg.dev.def.driver,
                       (char *)NULL
      )){
        printf("[%d] ", getpid( ));
        fflush( stdout );
        perror( "execl" );
      }
      return EXIT_FAILURE;    
    }    
    n_proc ++;
  }

  const char path_ser2eth[ ] = "ser2eth.out";
  const char path_eth2ser[ ] = "eth2ser.out";
  int nic1, nic2;
  char name1[ NAME_MAX ], name2[ NAME_MAX ];

  if( cfg.nicopt ){
    printf("Have to initialize two network interface instances ...\n");
    printf("tx: %s, rx: %s ...\n", cfg.nic.tx, cfg.nic.rx );

    int nic_tx = rawsocket( cfg.nic.tx );
    if( -1 == nic_tx ){
      perror("rawsocket");
    }

    int nic_rx = rawsocket( cfg.nic.rx );
    if( -1 == nic_rx ){
      perror("rawsocket");
    }
    
    if( cfg.devopt ){
      nic1 = nic_tx;
      nic2 = nic_rx;
      strncpy( name1, cfg.dev.tx.name, sizeof(name1) );
      strncpy( name2, cfg.dev.rx.name, sizeof(name2) );
    }
    else{
      nic1 = nic_tx;
      nic2 = nic_rx;
      strncpy( name1, cfg.dev.def.name, sizeof(name1) );
      strncpy( name2, cfg.dev.def.name, sizeof(name2) );
    }
  }
  else{
    printf("Have to initialize one network interface instance ...\n");
    printf("default: %s ...\n", cfg.nic.def );

    int nic = rawsocket( cfg.nic.def );
    if( -1 == nic ){
      perror("rawsocket");
    }    
    
    nic1 = nic;
    nic2 = nic;   

    if( cfg.devopt ){
      strncpy( name1, cfg.dev.tx.name, sizeof(name1) );
      strncpy( name2, cfg.dev.rx.name, sizeof(name2) );
    }
    else{
      strncpy( name1, cfg.dev.def.name, sizeof(name1) );
      strncpy( name2, cfg.dev.def.name, sizeof(name2) );
    }
  }
  char snic1 [ 16 ];
  snprintf( snic1, sizeof(snic1), "%d", nic1 );
  printf("Opening eth2ser: ./%s %s %s %s %s\n", path_eth2ser, "-i", snic1, "-n", name1 );

  opened_proc[ n_proc ] = fork( );
  if( !opened_proc[ n_proc ] ){
    if( -1 == execl( path_eth2ser, 
                     path_eth2ser,
                     "-i",
                     snic1,
                     "-n",
                     name1,
                     (char *)NULL
    )){
      printf("[%d] ", getpid( ));
      fflush( stdout );
      perror( "execl" );
    }
    return EXIT_FAILURE;    
  }    
  n_proc ++;

  char snic2 [ 16 ];
  snprintf( snic2, sizeof(snic2), "%d", nic2 );
  printf("Opening ser2eth: ./%s %s %s %s %s\n", path_ser2eth, "-i", snic2, "-n", name2 );
  opened_proc[ n_proc ] = fork( );
  if( !opened_proc[ n_proc ] ){
    if( -1 == execl( path_ser2eth,
                     path_ser2eth, 
                     "-i",
                     snic2,
                     "-n",
                     name2,
                     (char *)NULL
    )){
      printf("[%d] ", getpid( ));
      fflush( stdout );
      perror( "execl" );
    }
    return EXIT_FAILURE;    
  }    
  n_proc ++;    

  for( ; ; ){
    sleep( 10 );
    waitpid( -1, &status, WNOHANG );

    if( signal_flag ){
      for( int i = 0 ; i < n_proc ; ++i ){
        printf("[%d] Killing the process %d...\n", getpid( ), opened_proc[i] );
        if( -1 == kill( opened_proc[i], SIGTERM ) )
          perror( "kill" );
        else
          waitpid( opened_proc[i], &status, 0 );
      }
      break;
    }
  }

  close( nic1 );
  close( nic2 );
  return EXIT_SUCCESS;
}
