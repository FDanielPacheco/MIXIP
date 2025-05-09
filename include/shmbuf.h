/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * Introduction
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/

/**********************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************//**
 * @file      shmbuf.h
 * 
 * @version   1.0
 *
 * @date      03-05-2025
 *
 * @brief     Shared memory segment abstraction.  
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
 * Definition file
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/

#ifndef SHMBUF_H
#define SHMBUF_H
 
/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * External C++ extern macro
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
 
#ifdef __cplusplus  
extern "C" {        
#endif

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * Libraries
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/

#include <stdio.h>

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * Prototypes
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/

/**********************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************//**
 * @brief Initializes a memory segment like shm_open but instead of returning the file descriptor, returns the memory pointer.
 *  
 * @param[in] path The path to the shared memory, e.g., mem1 corresponds to /dev/shm/mem1.
 * @param[in] size The amount of bytes that will be allocated by the memory segment.
 * @param[in] oflag Is a bit mask created by ORing together exactly one of O_RDONLY or O_RDWR and any of the other flags @see(shm_open).
 * @param[in] mode The file permissions, e.g., 0666.
 * 
 * @return Upon success, performing the process, it returns a pointer to the memory segment. \n
 *         Otherwise, NULL is returned and `errno` is set to indicate the error.
 * 
 * @b Example \n
 *  Create a memory segment with int size\n
 * @code{.c}
 *  int * a = (int *) shm_open2( pathname, sizeof(int), O_CREAT | O_RDWR, 0666 );
 *  if( !a )
 *   // handle error ...
 *  *a = 5;
 *  // ...
 * @endcode
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
void * shm_open2( const char * path, const size_t size, int oflag, mode_t mode );

/**********************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************//**
 * @brief Unmaps a shared memory pointer, and if path is passed as argument, unlink the shared memory segment.
 *  
 * @param[in] path (Nullable*) The path to the shared memory, if the path is not Null, the shared memory will be unlinked.
 * @param[in] mem The pointer to the segment of memory to unmap. 
 * @param[in] size The amount of bytes that will be deallocated from the memory segment.
 * 
 * @return Upon success, performing the process, it returns 0. \n
 *         Otherwise, -1 is returned and `errno` is set to indicate the error.
 * 
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int shm_close2( const char * path, void * mem, const size_t size );

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * External C++ extern macro
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
 
#ifdef __cplusplus  
}
#endif
 
/*************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************** 
 * Definition file
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/

#endif
 
/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * End file
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
