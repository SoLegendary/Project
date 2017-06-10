/*!
**  @file UART.c
**
**  @brief Contains functions for using the K70 Tower UART (Universal Asynchronous Receiver Transmitter)
**         Initializes the UART via register and baud rate settings as well as polling and
**         manipulating the Transfer and Receive FIFOs to send and receive data from the PC.
*/
/*!
**  @addtogroup main_module main module documentation
**
**  @author Thanit Tangson & Emile Fadel
**  @{
*/
/* MODULE UART */

#include "UART.h"
#include "FIFO.h"
#include "MK70F12.h"
#include "Cpu.h"
#include "OS.h"

#define THREAD_STACK_SIZE 1024

static TFIFO TxFIFO, RxFIFO; // Transfer & Receiver FIFO declaration

// Stacks for FIFO threads
static uint32_t FIFOPutThreadStack[THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));
static uint32_t FIFOGetThreadStack[THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));

// Semaphores for FIFO threads
static OS_ECB* FIFOPutSemaphore;
static OS_ECB* FIFOGetSemaphore;



/*! @brief Thread to put data into the FIFO
 */
static void FIFOPutThread(void* pData)
{
  for (;;)
  {
    OS_SemaphoreWait(FIFOPutSemaphore,0);
    FIFO_Put(&RxFIFO, UART2_D); // Put a character into the receive FIFO
    UART2_C2 |= UART_C2_RIE_MASK;
  }
}


/*! @brief Thread to take data out of the FIFO
 */
static void FIFOGetThread(void* pData)
{
  for (;;)
  {
    OS_SemaphoreWait(FIFOGetSemaphore,0);
    FIFO_Get(&TxFIFO, (uint8_t *)&UART2_D); // Get a character from the transfer FIFO
    UART2_C2 |= UART_C2_TIE_MASK; // Sets the TIE flag to indicate there's a character in the TxFIFO
  }
}



/*!
 * Initializing the UART
 * send parameters: baudRate = 115200, moduleClk = CPU_BUS_CLK_HZ (20,971,520 Hz)
 */
bool UART_Init(const uint32_t baudRate, const uint32_t moduleClk)
{
  FIFO_Init(&TxFIFO); //Initialization of Transfer FIFO
  FIFO_Init(&RxFIFO); //Initialization of Receiver FIFO

  // initialising semaphores
  FIFOPutSemaphore = OS_SemaphoreCreate(0);
  FIFOGetSemaphore = OS_SemaphoreCreate(0);

  OS_ERROR error; // error object for RTOS

  // creating FIFO threads
  error = OS_ThreadCreate(FIFOPutThread,
          NULL,
          &FIFOPutThreadStack[THREAD_STACK_SIZE - 1],
	  2);

  error = OS_ThreadCreate(FIFOGetThread,
          NULL,
          &FIFOGetThreadStack[THREAD_STACK_SIZE - 1],
	  3);

  // UART2 Clock Gate - Enabled to turn on the UART2 module.
  SIM_SCGC4 |= SIM_SCGC4_UART2_MASK;
  // PORTE Clock Gate - Enabled to turn on PORT E.
  SIM_SCGC5 |= SIM_SCGC5_PORTE_MASK;
  // Setting the MUX on Pin Control Register 16 to the value of 3
  PORTE_PCR16 |= PORT_PCR_MUX(0x3);
  // Setting the MUX on Pin Control Register 17 to the value of 3
  PORTE_PCR17 |= PORT_PCR_MUX(0x3);
	
	
  // Setting Baud Rate based on parameters
  // The formula for SBR and BRFD bit setting is: sbr + (brfa/32) = moduleClk / (16 x baudRate)
	
  // sets sbr to be the integer (rounded down) of the right side of the formula, ignoring brfa for now
  uint16union_t sbr;
  sbr.l = moduleClk/(16*baudRate);
	
  // sbr maximum value is 8191 (13-bit) so return false if the parameters require an impossible baud rate
  if(sbr.l > 8191)
    return false;

  // sbr is a 13-bit number but the register is across BDH(5) and BDL(8)
  // so most/least significant bits are separated via bit masking

  // Bit mask with binary number (1 1111 0000 0000), shifted right by 8 places
  sbr.s.Hi = (sbr.l & 0x1F00) >> 8;
  // Bit mask with binary number (0 0000 1111 1111)
  sbr.s.Lo = (sbr.l & 0xFF);
	
  // Getting brfa (Baud Rate Fine Adjust) value which is equal to the unrounded sbr value minus the integer sbr value
  uint8_t brfa = (((32*moduleClk)/(16*baudRate))-(32*sbr.l));
	
	
  // Assigning most significant bits of the SBR register
  UART2_BDH = UART_BDH_SBR(sbr.s.Hi);
  // Assigning least significant bits of the SBR register
  UART2_BDL = UART_BDL_SBR(sbr.s.Lo);
  // Setting Baud Rate Fine Adjust value
  UART2_C4 |= UART_C4_BRFA(brfa);
	
  // Setting transfer and receive bits
  UART2_C2 |= UART_C2_TE_MASK;
  UART2_C2 |= UART_C2_RE_MASK;

  // Allowing RDRF flag to generate interrupts
  UART2_C2 |= UART_C2_RIE_MASK;

  // Setting up NVIC for UART2 see K70 manual pg 97
  // Vector=66, IRQ=49
  // NVIC non-IPR=1 IPR=12
  // Clear any pending interrupts on UART2
  NVICICPR1 = (1 << 17); // 49mod32 = 17
  // Enable interrupts from UART2
  NVICISER1 = (1 << 17);

  return true;
}



bool UART_InChar(uint8_t * const dataPtr)
{
  return FIFO_Get(&RxFIFO, dataPtr); // Gets a character out of the receive FIFO
}



bool UART_OutChar(const uint8_t data)
{
  return (FIFO_Put(&TxFIFO, data)); // Attempts to put a character into the transfer FIFO
 // {
    //EnterCritical(); // Nesting-compatible Interrupt Disable
    //UART2_C2 |= UART_C2_TIE_MASK; // Sets the TIE flag to indicate there's a character in the TxFIFO
    //ExitCritical(); // Nesting-compatible Interrupt Enable
    //return true;
  //}
  //return false; // Returns false if there was a problem putting a char into TxFIFO
}



void UART_Poll(void)
{
  if (UART2_S1 & UART_S1_RDRF_MASK) // If PC->Tower data is waiting to be read, put a character into the receive FIFO
    FIFO_Put(&RxFIFO, UART2_D);
  if (UART2_S1 & UART_S1_TDRE_MASK) // If Tower->PC data is waiting to be read, get a character out of the transfer FIFO
    FIFO_Get(&TxFIFO, (uint8_t *)&UART2_D);
}



void __attribute__ ((interrupt)) UART_ISR(void)
{
  OS_ISREnter();
	
  if (UART2_C2 & UART_C2_RIE_MASK) // If the interrupt was due to receiving a character
  {
    if (UART2_S1 & UART_S1_RDRF_MASK) // If PC->Tower data is waiting to be read, put a character into the receive FIFO
    {
      UART2_C2 &= ~UART_C2_RIE_MASK;
      OS_SemaphoreSignal(FIFOPutSemaphore);
    }
  }

  if (UART2_C2 & UART_C2_TIE_MASK) // If the interrupt was due to transmitting a character
  {
    if (UART2_S1 & UART_S1_TDRE_MASK) // If Tower->PC data is waiting to be read, get a character out of the transfer FIFO
    {
      UART2_C2 &= ~UART_C2_TIE_MASK;
      OS_SemaphoreSignal(FIFOGetSemaphore);
    }
  }
  
  OS_ISRExit();
}



/* END UART */
/*!
** @}
*/
