/*!
**  @file PIT.c
**
**  @brief Routines for controlling Periodic Interrupt Timer (PIT) on the TWR-K70F120M.
**         This contains the functions for operating the periodic interrupt timer (PIT).
*/
/*!
**  @addtogroup main_module main module documentation
**
**  @author Thanit Tangson & Emile Fadel
**  @{
*/
/* MODULE PIT */

#include "PIT.h"
#include "MK70F12.h"
#include "OS.h"
#include "Cpu.h"

// Private global variable for the PIT thread semaphore
static OS_ECB* PITSemaphore[3];

// Mutexes for functions called by multiple threads in main
static OS_ECB* SetAccess;
static OS_ECB* GetTimeAccess;


bool PIT_Init(const uint32_t moduleClk, OS_ECB* semaphore[])
{
  // initialise mutexes
  SetAccess     = OS_SemaphoreCreate(0);
  GetTimeAccess = OS_SemaphoreCreate(0);

  // saving semaphores for use in the ISR
  for (uint8_t i = 0; i < 3; i++)
    PITSemaphore[i] = semaphore[i];

  // Enabling clock gate for PIT
  SIM_SCGC6 |= SIM_SCGC6_PIT_MASK;
	
  // Clearing the Module Disable bit to enable the PIT
  PIT_MCR &= ~PIT_MCR_MDIS_MASK;
  
  // Freezes the PIT while debugging
  PIT_MCR |= PIT_MCR_FRZ_MASK;
  
  // Note that there are 4 timers that can be used simultaneously (0-3), but only 2 are needed
  // write 1 to clear the TIF bit to avoid an unwanted interrupt during initialization
  PIT_TFLG0  |= PIT_TFLG_TIF_MASK;
  PIT_TFLG1  |= PIT_TFLG_TIF_MASK;

  
  // Sets Timer to a 500ms period (as specified in software requirements)
  // LDVAL = (period*moduleClk) - 1
  PIT_LDVAL0 = (moduleClk/2) - 1;

  // PIT1 is set to be a free-running timer (maximum possible timeout)
  PIT_LDVAL1 = 0xFFFFFFFF;

  // Timer Interrupt Enable and Timer Enable
  PIT_TCTRL0 |= PIT_TCTRL_TIE_MASK;
  PIT_TCTRL0 |= PIT_TCTRL_TEN_MASK;
  PIT_TCTRL1 |= PIT_TCTRL_TIE_MASK;
  PIT_TCTRL1 |= PIT_TCTRL_TEN_MASK;


  // Clear any pending interrupts on PIT0
  NVICICPR2 = (1 << 4); // 68mod32 = 4
  // Enable interrupts from the PIT
  NVICISER2 = (1 << 4);

  // Clear any pending interrupts on PIT1
  NVICICPR2 = (1 << 5); // 69mod32 = 5
  // Enable interrupts from the PIT
  NVICISER2 = (1 << 5);

  return true;
}



void PIT_Set(const uint32_t period, const bool restart)
{
  OS_SemaphoreWait(SetAccess,0);
	
  // *10^9 to convert to seconds from nanoseconds, then /50*10^6 as moduleClk should be 50Mhz
  // This works out to be division by 20
  uint32_t ldVal = (period/20) - 1;
	
  if (restart)
  {
    PIT_TCTRL0 &= ~PIT_TCTRL_TEN_MASK; // new period is enacted immediately, restarting the timer
    PIT_LDVAL0 = ldVal;
    PIT_TCTRL0 |= PIT_TCTRL_TEN_MASK;
  }
  else
    PIT_LDVAL0 = ldVal; // new period is enacted after the next interrupt

  OS_SemaphoreSignal(SetAccess);
}


uint32_t PIT_GetTime(void)
{
  OS_SemaphoreWait(GetTimeAccess,0);
  
  // calculate time since PIT1 was started in miliseconds
  uint32_t time = (0xFFFFFFFF - PIT_CVAL1) / (CPU_BUS_CLK_HZ/1000);
  
  OS_SemaphoreSignal(GetTimeAccess);
  
  return time;
}


void PIT_Enable(const bool enable)
{
  if (enable)
    PIT_TCTRL0 |= PIT_TCTRL_TEN_MASK;
  else
    PIT_TCTRL0 &= ~PIT_TCTRL_TEN_MASK;
}



void __attribute__ ((interrupt)) PIT_ISR(void)
{
  OS_ISREnter();
	
  // PIT0 is used for DOR timing (ISR does nothing in response except clear the flag)
  if (PIT_TFLG1)
    PIT_TFLG1 |= PIT_TFLG_TIF_MASK;

  // PIT0 is used for semaphore signalling
  if (PIT_TFLG0)
  {
    PIT_TFLG0 |= PIT_TFLG_TIF_MASK;

    // Signal the semaphores, allowing the threads in main.c to run
    for (uint8_t i = 0; i < 3; i++)
      OS_SemaphoreSignal(PITSemaphore[i]);
  }
  
  OS_ISRExit();
}



/* END PIT */
/*!
** @}
*/
