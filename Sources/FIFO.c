/*!
**  @file FIFO.c
**
**  @brief Contains functions for initializing and manipulating data in FIFO arrays
**         Initializes a FIFO by resetting values to 0 and also allows for the input and output
**         of data to the UART module using a 256 bit shifting array.
*/
/*!
**  @addtogroup main_module main module documentation
**
**  @author Thanit Tangson & Emile Fadel
**  @{
*/
/* MODULE FIFO */

#include "FIFO.h"
#include "Cpu.h"
#include "OS.h"

// Binary semaphore ensuring only one thread accesses the semaphore at a time
static OS_ECB* FIFOAccess;


/*!
 * Setting all parameters of the FIFO to 0 for initialization.
 */
void FIFO_Init(TFIFO * const FIFO)
{
  FIFOAccess = OS_SemaphoreCreate(1); // creating semaphore

  FIFO->Start     = 0;
  FIFO->End       = 0;
  FIFO->UsedBytes = OS_SemaphoreCreate(0);
  FIFO->FreeBytes = OS_SemaphoreCreate(FIFO_SIZE);
}



bool FIFO_Put(TFIFO * const FIFO, const uint8_t data)
{
  // If the FIFO has room, it places some data in to it and returns true.
  OS_SemaphoreWait(FIFOAccess,0); // wait to get exclusive access

  // Increments the number of bytes
  OS_SemaphoreWait(FIFO->FreeBytes,0);
  OS_SemaphoreSignal(FIFO->UsedBytes);

  // Put data in and increment end index
  FIFO->Buffer[FIFO->End] = data;
  FIFO->End++;

  if (FIFO->End >= FIFO_SIZE) // If the FIFO is full, it loops back around
    FIFO->End = 0;

  // Relinquish exclusive access
  OS_SemaphoreSignal(FIFOAccess);

  return true;
}



bool FIFO_Get(TFIFO * const FIFO, uint8_t * const dataPtr)
{
  OS_SemaphoreWait(FIFOAccess,0); // wait to get exclusive access

  // Decrements the number of bytes
  OS_SemaphoreWait(FIFO->UsedBytes,0);
  OS_SemaphoreSignal(FIFO->FreeBytes);

  // Take data out and increment start index
  *dataPtr = FIFO->Buffer[FIFO->Start];
  FIFO->Start++;

  // If the FIFO is now full, resets the start position
  if (FIFO->Start >= FIFO_SIZE)
    FIFO->Start = 0;

  // Relinquish exclusive access
  OS_SemaphoreSignal(FIFOAccess);

  return true;
}



/* END FIFO */
/*!
** @}
*/
