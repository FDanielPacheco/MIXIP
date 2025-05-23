#include <mixip.h> 
#include <serialposix.h> 
 
int dsetup( serial_manager_t * serial ){
  // Runs first and once...
  serial_set_baudrate( B19200, &serial->sr );
  return 0; 
}
 
int dloop( flow_t * flow ){
  // Runs in loop, in a separeted process, consider limiting the CPU poll with a sleep...
  // To stop the other process (read/write) use halt_network( flow ), and continue_network( flow )
  
  return 0; 
}
 
int dread( buffer_t * buf ){
  
  return 0; 
}
 
int dwrite( buffer_t * buf ){
  
  return 0; 
}
