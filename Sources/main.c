/* ###################################################################
 **     Filename    : main.c
 **     Project     : Lab6
 **     Processor   : MK70FN1M0VMJ12
 **     Version     : Driver 01.01
 **     Compiler    : GNU C Compiler
 **     Date/Time   : 2015-07-20, 13:27, # CodeGen: 0
 **     Abstract    :
 **         Main module.
 **         This module contains user's application code.
 **     Settings    :
 **     Contents    :
 **         No public methods
 **
 ** ###################################################################*/
/*!
 ** @file main.c
 ** @version 6.0
 ** @brief
 **         Main module.
 **         This module contains user's application code.
 */
/*!
 **  @addtogroup main_module main module documentation
 **  @{
 */
/* MODULE main */

#include "types.h"
#include "math.h"
#include "stdlib.h"

// CPU module - contains low level hardware initialization routines
#include "Cpu.h"

// ADC and DAC converter
#include "analog.h"

// Simple OS
#include "OS.h"

// Analog functions
// Packet/UART functionality from labs
#include "packet.h"
#include "UART.h"

// Non-volatile memory
#include "Flash.h"

// Periodic Interrupt Timer
#include "PIT.h"


// NOTES FROM LAB TESTING

/* Analog_Put outputs a flat DC signal forever at a value of 16000 -> 5V (make sure DSO is set to DC coupling)
 * Use 50 Ohms output on the far right of the function generator to get the input signal
 * Analog_Get result may need to be adjusted (probably divide by 16000)
 */



// ----------------------------------------
// PRIVATE GLOBAL VARIABLES
// ----------------------------------------

// Tower Protocol command bytes
#define CMD_ANALOGGET 0x11
#define CMD_DORVALUES 0x12
#define CMD_SETCHAR   0x13
#define CMD_SETMODE   0x14

// Output signals (16000 corresponds to 5V which will trip the breaker at Irms > 1.03)
const static int16_t TIMING_SIGNAL_LOW  = 0;
const static int16_t TIMING_SIGNAL_HIGH = 16000;
const static int16_t TRIP_SIGNAL_LOW    = 0;
const static int16_t TRIP_SIGNAL_HIGH   = 16000;

// Private global for toggling fault-sensitive mode
static bool SensitiveMode = false;

// Private global array to track which channels faulted in the last thread trips
static uint8_t ChannelFault[3] = {0};

// Stored in flash - Characteristic in use - 1:inverse, 2:very inverse, 3:extremely inverse
static uint8_t *CharType = NULL;

// Stored in flash - number of times the DOR has sent out a trip signal
static uint16_t *TimesTripped = NULL;


// Lookup tables for trip timers for each of the 3 characteristic curves
// Based on the formula t = k/(I^a - 1) -> results in ms
// Sample calculations done at I = 1.03 and then every integer value up to 20

// k = 0.14, a = 0.02
const static uint32_t TripTimeInv[20]     = {236746,10029,6302,4980,4280,3837,3528,3296,3116,2971,
                                             2850,2748,2660,2583,2516,2455,2401,2352,2308,2267};
// k = 13.5, a = 1
const static uint32_t TripTimeVeryInv[20] = {450000,13500,6750,4500,3375,2700,2250,1929,1688,
                                             1500,1350,1227,1125,1038,964,900,844,794,750,711};
// k = 80, a = 2
const static uint32_t TripTimeExtInv[20]  = {1313629,26667,10000,5333,3333,2286,1667,1270,1000,
                                             808,667,559,476,410,357,314,278,248,222,201};

										

// ----------------------------------------
// THREAD SETUPS
// ----------------------------------------
// Arbitrary thread stack size - big enough for stacking of interrupts and OS use.
#define THREAD_STACK_SIZE 100
#define NB_ANALOG_CHANNELS 3

// Thread stacks
static uint32_t InitModulesThreadStack[THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));
static uint32_t TripThreadStack[NB_ANALOG_CHANNELS][THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));
static uint32_t SamplingThreadStack[NB_ANALOG_CHANNELS][THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));
static uint32_t PacketThreadStack[THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));

// Semaphores
static OS_ECB* SamplingThreadSemaphore[NB_ANALOG_CHANNELS];
static OS_ECB* TripThreadSemaphore[NB_ANALOG_CHANNELS];

// Thread priorities
// 0 = highest priority
const uint8_t TRIP_THREAD_PRIORITY[NB_ANALOG_CHANNELS]     = {1, 2, 3};
const uint8_t SAMPLING_THREAD_PRIORITY[NB_ANALOG_CHANNELS] = {4, 5, 6};
const uint8_t PACKET_THREAD_PRIORITY                       = 7;



// ----------------------------------------
// DATA STRUCTURES
// ----------------------------------------

// shortcut macro to improve readability in the sampling thread
#define AnalogInputs Analog_Inputs[*(uint8_t*)pData]

/* typedef struct   < extern'd from analog.h
{
  float value;                         < The current "processed" analog value (the user updates this value).
  float oldValue;                      < The previous "processed" analog value (the user updates this value).
  int16_t values[ANALOG_WINDOW_SIZE];  < An array of sample values to create a "sliding window". 
  uint32_t time[ANALOG_WINDOW_SIZE];   /*!< Time stamp of the samples taken using PIT_GetTime() in milliseconds
  int16_t* putPtr;                     < A pointer into the array of the last sample taken. 
} TAnalogInput;

extern TAnalogInput Analog_Inputs[ANALOG_NB_INPUTS];  < Analog input channels. */

// NOTE: values[] stores newest data at [0] and oldest at [15]




// ----------------------------------------
// PRIVATE DOR FUNCTIONS
// ----------------------------------------

/*! @brief  Returns the true Vrms value of an array of analog data of size 'samples' taking into account analog normalisation
 *  @param  array of data samples
 *  @param  number of samples to get the RMS for
 *  @return true RMS value of the set of data
 *  */
static float GetRMS(int16_t data[], uint16_t samples)
{
  // converting to voltage
  for (uint8_t i = 0; i < samples; i++)
    data[i] = data[i] / 3200;
	
  // find total of all values squared
  uint16_t squaredTotal = 0;
  for (uint8_t i = 0; i < samples; i++)
    squaredTotal += (data[i]*data[i]);
	
  return sqrt(squaredTotal/samples);
}


/*! @brief  Returns the frequency given a set of sinusoid samples - note that this will only work for a sinusoid with zero DC offset
 *  @param  array of data samples
 *  @return frequency of the waveform based on the data
 * */
static float GetFreq(const int16_t data[])
{
  uint32_t zeroCrossTime[2] = {0}; // time in milliseconds at which the signal is 0
  uint8_t crossingsFound    = 0;   // crossings found so far (ranges from 0-2)
  int16_t t1=0, t2=0, t3=0, t4=0;  // indexes to: sample just before 1st crossing, sample just after 1st crossing,
                                   // sample just before 2nd crossing, and sample just after 2nd crossing respectively

  // If crossingsFound is >= 2, we already found two zero-crossings, so end the loop
  for (uint8_t i = 0; (i < ANALOG_WINDOW_SIZE-1) && (crossingsFound < 2); i++)
  {
    // crossing in between samples
    if ((data[i] > 0 && data[i+1] < 0) ||
	      (data[i] < 0 && data[i+1] > 0))
	  {
	    switch (crossingsFound)
	    {
		    case 0:
		      t1 = i;
	  	    t2 = i+1;
		      crossingsFound++;
		      break;
		    case 1:
		      t3 = i;
		      t4 = i+1;
		      crossingsFound++;
	    	  break;
	    }
	  }
	
    // sample was exactly 0
    else if (data[i] == 0)
	  {
	    zeroCrossTime[crossingsFound] = data[i];
	    crossingsFound++;
	  }
  }
  
  // Calculating zeroCrossTime, unless they were found from a perfect sample at exactly 0. For example:
  // If we have two samples on either side of a zero crossing at times t1 and t2 on a sin function x(t), then
  //                  (t2 - t1) * |x(t1)|
  // zeroCrossTime =  -------------------
  //                   |x(t1)| + |x(t2)|
  //
  // In the code below, x(t) is represented as data[t] and absolute values as the function abs()
  // Times are in milliseconds, but converted back to seconds here by division by 1000
  
  if (!zeroCrossTime[0])
  	zeroCrossTime[0] = (((t2 - t1) * abs(data[t1])) / (abs(data[t1]) + abs(data[t2]))) / 1000;
  
  if (!zeroCrossTime[1])
    zeroCrossTime[1] = (((t4 - t3) * abs(data[t3])) / (abs(data[t3]) + abs(data[t4]))) / 1000;


  // difference between the above zero crossings is half the period, and 1/period = frequency so 1/(2*difference) = frequency
  return (1 / (2*(zeroCrossTime[1] - zeroCrossTime[0])));
}



// ----------------------------------------
// PACKET HANDLING FUNCTIONS
// ----------------------------------------

/*! @brief Handles a packet to get the instantaneous value of current from one of the ADC channels
 *
 *  Parameter1 = channel number (0 - 3)
 *  Parameter2 = 0x0, Parameter3 = 0x0
 * 
 *  @return bool - TRUE if the packet was handled successfully, FALSE if channelNb out of range
 */
bool HandleAnalogGetPacket(void)
{
  int16_t voltage;
  Analog_Get(Packet_Parameter1, &voltage);
  
  // convert to current from analog voltage data
  int16_t current = ((voltage*20)/7) / 3200;
  
  // getting sign of current
  bool sign = true;
  if (current < 0)
	sign = false;
  
  // returns channel number, current value (absolute) and sign of current value (FALSE = negative, TRUE = positive or 0)
  return Packet_Put(CMD_ANALOGGET, Packet_Parameter1, current, sign);
}

/*! @brief Handles a packet to get various values about the DOR not already detailed in the tower protocol
 *
 *  Parameter1 = 0x0 (CharType), 0x1 (current freq.), 0x2 (TimesTripped), 0x3 (lastFaultType), 0x4 (SensitiveMode)
 *  Parameter2 = 0x0, Parameter3 = channelNb (for freq)
 * 
 *  @return bool - TRUE if the packet was handled successfully, FALSE if parameter1 out of range
 */
bool HandleDORValuesPacket(void)
{
  float freq;
  uint8_t freqMSB, freqLSB; // whole number and decimal component of frequency
  uint8_t lastFaultType;    // sum of trip threads' fault variables

  switch (Packet_Parameter1)
  {
    case 0: // Characteristic in use - stored in flash
      return Packet_Put(CMD_DORVALUES, *CharType, 0, 0);

    case 1: // Frequency of currents
	    freq = GetFreq(Analog_Inputs[Packet_Parameter2].values);
	    freqMSB = freq;                  // whole number frequency
	    freqLSB = (freq - freqMSB) * 10; // first decimal point
      return Packet_Put(CMD_DORVALUES, freqMSB, freqLSB, Packet_Parameter3);

    case 2: // Number of times tripped - stored in flash
      return Packet_Put(CMD_DORVALUES, *TimesTripped, 0, 0);

    case 3: // Type of last fault detected (3,2,1-phase)
      lastFaultType = ChannelFault[0] + ChannelFault[1] + ChannelFault[2];
      return Packet_Put(CMD_DORVALUES, lastFaultType, 0, 0);

    case 4: // Sensitive mode
      return Packet_Put(CMD_DORVALUES, SensitiveMode, 0, 0);

    default: return false;
  }
}

/*! @brief Handles a packet to set the characteristic in use by the DOR
 *
 *  Parameter1 = 0x0 (inverse), 0x1 (very inverse), 0x2 (extremely inverse)
 *  Parameter2 = 0, Parameter3 = 0
 *
 *  @return bool - TRUE if the packet was handled successfully.
 */
bool HandleSetCharPacket(void)
{
  // Returns false if out of range
  if (Packet_Parameter1 < 0 || Packet_Parameter1 > 2)
    return false;
	
  Flash_Write8((uint8_t*)CharType, Packet_Parameter1);
  return true;
}

/*! @brief Handles a packet to change the DOR between sensitive and nonsensitive fault mode
 *
 *  Parameter1 = 0x0 (nonsensitive) or 0x1 (sensitive), 
 *  Parameter2 = 0x0, Parameter3 = 0x0
 *
 *  @return bool - TRUE if the packet was handled successfully, FALSE if parameter1 out of range
 */
bool HandleSetModePacket(void)
{
  if (Packet_Parameter1 < 0 || Packet_Parameter1 > 1)
    return false;
  
  SensitiveMode = Packet_Parameter1;
  return true;
}

/*! @brief Handles the packet by first checking to see what type of packet it is and processing it
 *  as per the Tower Serial Communication Protocol document.
 */
void HandlePacket(void)
{
  bool success = false; // Holds the success or failure of the packet handlers
  bool ackReq  = false; // Holds whether an Acknowledgment is required

  if (Packet_Command & PACKET_ACK_MASK) // Check if ACK bit is set
  {
    ackReq = true;
    Packet_Command &= 0x7F; //Strips the top bit of the Command Byte to ignore ACK bit
  }

  switch (Packet_Command)
  {
	  case CMD_ANALOGGET:
	    success = HandleAnalogGetPacket();
      break;
    case CMD_DORVALUES:
	    success = HandleDORValuesPacket();
      break;
	  case CMD_SETCHAR:
	    success = HandleSetCharPacket();
	    break;
	  case CMD_SETMODE:
	    success = HandleSetModePacket();
    default:
      success = false;
      break;
  }

  /*!
   * Check if the handling of the packet was a success and an ACK packet was requested
   * If that checks out, set the ACK bit to 1
   * Else, if the handling of the packet failed and an ACK packet was requested
   * Clear the ACK bit in order to indicate a NAK (command could not be carried out)
   * Finally, return the ACK packet to the Tower
   */

  if (ackReq)
  {
    if (success)
      Packet_Command |= PACKET_ACK_MASK; // Set the ACK bit
    else
      Packet_Command &= ~PACKET_ACK_MASK; // Clear the ACK bit

    Packet_Put(Packet_Command, Packet_Parameter1, Packet_Parameter2, Packet_Parameter3); // Send the ACK/NAK packet back out
  }
}


// ----------------------------------------
// RTOS THREADS IN ORDER OF PRIORITY
// ----------------------------------------

/*! @brief Initialises modules.
 */
static void InitModulesThread(void* pData)
{
  // Creating semaphores
  for (uint8_t i = 0; i < NB_ANALOG_CHANNELS; i++)
  {
    SamplingThreadSemaphore[i] = OS_SemaphoreCreate(0);
    TripThreadSemaphore[i]     = OS_SemaphoreCreate(0);
  }
  
  // Module inits
  Flash_Init();
  UART_Init(115200, CPU_BUS_CLK_HZ);
  (void)Analog_Init(CPU_BUS_CLK_HZ);

  // Initialises the PIT to 20ms period (for 50Hz by default)
  PIT_Init(CPU_BUS_CLK_HZ, SamplingThreadSemaphore);
  PIT_Set(20000000, true);
  PIT_Enable(true);

  // Allocating private globals in non-volatile memory
  Flash_AllocateVar((void*)&CharType, sizeof(*CharType));
  Flash_AllocateVar((void*)&TimesTripped, sizeof(*TimesTripped));

  Flash_Write8((uint8_t*)CharType, 1); // Inverse characteristic

  // Start outputs at 0
  Analog_Put(1, TIMING_SIGNAL_LOW);
  Analog_Put(2, TRIP_SIGNAL_LOW);

  // We only do this once - therefore delete this thread
  OS_ThreadDelete(OS_PRIORITY_SELF);
}



/*! @brief Thread to send a tripping signal through the DAC after a timeout period
 *  Tracks time elapsed in a local variable and keeps updating the new timer with each
 *  loop; waiting occurs at the end of each do-while loop, allowing for more samples
 *  to update the timer; if a new sample is Irms < 1.03, end timing and reset everything
 */
void TripThread(void* pData)
{
  static uint32_t timeElapsed;       // Time elapsed since the current thread loop started in milliseconds
  static uint32_t newTripTimer;      // New timer based on the IMDT curves in milliseconds
  static uint32_t tripTimer;         // Time until next trip in milliseconds
  static uint32_t startingTime;      // time at the start of a thread loop in milliseconds

  uint32_t idmtCurve[20]; // temporary allocation of the trip timer IDMT curves based on the characteristic in use
  float fractionalIrms; // fractional component of Irms

  for (;;)
  {
    startingTime = PIT_GetTime();
	
	  // OS_TimeGet gets the clock ticks since the thread loop has started last
	  // Then, if enough time hasnt yet passed on this check, then wait for the semaphore
	  do
	  {
	    // Updating time elapsed for this loop. Uses difference between current time and the time at
	    // the start of the loop so we don't need to use set a new timer which can mess up GetFreq()
	    timeElapsed = (PIT_GetTime() - startingTime);
	  
	  
	    // Assign correct IDMT lookup table to use for the timing calculation
	    switch (*CharType)
	    {
	      case 1:
	        for (uint8_t i = 0; i < 20; i++)
	          idmtCurve[i] = TripTimeInv[i];
		      break;
		
	      case 2:
          for (uint8_t i = 0; i < 20; i++)
            idmtCurve[i] = TripTimeVeryInv[i];
		      break;
		
	      case 3:
          for (uint8_t i = 0; i < 20; i++)
            idmtCurve[i] = TripTimeExtInv[i];
	        break;
  	  }


	    // interpolation of intermediate values in the lookup table
	    // eg. for 15.2, newtripTimer = idmtCurve[16] + ((idmtCurve[16] - idmtCurve[15]) * 0.2)
	    if (AnalogInputs.value < 20)
	    {
	      newTripTimer   = idmtCurve[(uint8_t)AnalogInputs.value + 1]; // Index is Irms rounded up
	      fractionalIrms = AnalogInputs.value - (uint8_t)AnalogInputs.value;
	      newTripTimer  += (newTripTimer - idmtCurve[(uint8_t)AnalogInputs.value]) * fractionalIrms;
	    }
	    else // minimum timeout
	      newTripTimer = idmtCurve[20];

	  
	    // new timeout period is less, so the circuit breaker should be tripped immediately
	    if (newTripTimer < tripTimer)
		    break;

	    // new timeout period is greater, but some time has already elapsed so only continue for a % of the new time
	    else if (newTripTimer > tripTimer)
	    {
		    tripTimer   = ((float)(1 - (timeElapsed/tripTimer)) * newTripTimer);
		    timeElapsed = 0;
	    }

	    // else, trip timer has not changed, so continue as normal
	
	  
	    // Blocks the thread (timing out after the TripTimer because if it
	    // takes that long to get back then it should have passed anyway
	    // If Irms < 1.03 then reset the timer and keep blocking

	    // If sensitive mode is on and the frequency is not within the
	    // 47.5 -> 52.5 Hz range then also block the thread in the same way
	    do
	    {
	      OS_SemaphoreWait(TripThreadSemaphore[*(uint8_t*)pData], tripTimer);

	      if (AnalogInputs.value < 1.03 ||
	         (SensitiveMode && (
	         (GetFreq(Analog_Inputs[*(uint8_t*)pData].values) < 47.5) ||
	         (GetFreq(Analog_Inputs[*(uint8_t*)pData].values) > 52.5))))
	      {
	        startingTime = PIT_GetTime();
	        timeElapsed  = 0;
	      }
	    }
	    while (AnalogInputs.value < 1.03);
	  }
	  while (timeElapsed < tripTimer);
	
	
    // Timer has expired, output the signal to trip the circuit breaker
	  Analog_Put(2, TRIP_SIGNAL_HIGH);

	  // Increment the number of times tripped in flash
	  Flash_Write16((uint16_t*)TimesTripped, (*TimesTripped) + 1);

	  // indicate a fault for this phase
	  ChannelFault[*(uint8_t*)pData] = 1;
  }
}


/*! @brief Thread to collect samples at least 16 times per cycle (based on frequency 47.5-52.5Hz)
 *  and parse their RMS value to determine the correct timeout period for TripThread.
 *  Should be duplicated 3 times for each channel (3 phases) each with different arguments to distinguish channels
 */
void SamplingThread(void* pData)
{
  float Vrms, Irms;  // RMS values of voltage and current
	float freq;

  for (;;)
  {
	  OS_SemaphoreWait(SamplingThreadSemaphore[*(uint8_t*)pData], 0);

	  // Shift data up the arrays
	  for (uint8_t i = 15; i > 0; i--)
	    AnalogInputs.values[i] = AnalogInputs.values[i-1];
  
	  // Get newest non-RMS voltage from the ADC for this channel
	  Analog_Get(*(uint8_t*)pData, &AnalogInputs.values[0]);
	  // Time stamp the sample for frequency detection
	  AnalogInputs.time[0] = PIT_GetTime();
	
	  // Parse new true RMS current value and convert from analog values (3200 -> 1V)
	  Vrms = GetRMS(AnalogInputs.values, ANALOG_WINDOW_SIZE);
	  Irms = (Vrms*20)/7; // Circuitry is such that when Vrms = 350mV, Irms = 1
	
	  // Update global structure
	  AnalogInputs.oldValue = AnalogInputs.value;
	  AnalogInputs.value    = Irms;
	
	  // Finds the frequency for the new set of data and sets the new PIT accordingly (in nanoseconds)
	  // 2nd parameter = false as we don't want to waste time resetting the clock
	  freq = GetFreq(Analog_Inputs[*(uint8_t*)pData].values);
	  PIT_Set(1000000000/freq, false);


	  // If we are timing at all, then output a TRIP signal - only start timing if Irms > 1.03
	  if (AnalogInputs.value > 1.03)
	    Analog_Put(1, TIMING_SIGNAL_HIGH);
	  else
	    Analog_Put(1, TIMING_SIGNAL_LOW);

	  // Even if not timing, we will still go into TripThread, but it will reset itself if Irms < 1.03s
	  OS_SemaphoreSignal(TripThreadSemaphore[*(uint8_t*)pData]);
  }
}


/*! @brief Packet handling thread - MAKE SURE THIS IS ACTUALLY POSSIBLE TO TRIGGER
 */
void PacketThread(void* pData)
{
  for (;;)
  {
    if (Packet_Get()) // If a packet is received.
      HandlePacket(); // Handle the packet appropriately.
  }
}







/*lint -save  -e970 Disable MISRA rule (6.3) checking. */
int main(void)
/*lint -restore Enable MISRA rule (6.3) checking. */
{
  OS_ERROR error;

  // Initialise low-level clocks etc using Processor Expert code
  PE_low_level_init();

  // Initialize the RTOS
  OS_Init(CPU_CORE_CLK_HZ, true);

  // Create module initialisation thread
  error = OS_ThreadCreate(InitModulesThread,
          NULL,
          &InitModulesThreadStack[THREAD_STACK_SIZE - 1],
          0); // Highest priority

  // Create threads for 3 'Trip' + 'Timing' outputs
  for (uint8_t threadNb = 0; threadNb < NB_ANALOG_CHANNELS; threadNb++)
  {
    error = OS_ThreadCreate(TripThread,
						&threadNb,
            &TripThreadStack[threadNb][THREAD_STACK_SIZE - 1],
            TRIP_THREAD_PRIORITY[threadNb]);
  }
  
  // Create threads for 3 analog sampling channels
  for (uint8_t threadNb = 0; threadNb < NB_ANALOG_CHANNELS; threadNb++)
  {
    error = OS_ThreadCreate(SamplingThread,
						&threadNb,
            &SamplingThreadStack[threadNb][THREAD_STACK_SIZE - 1],
            SAMPLING_THREAD_PRIORITY[threadNb]);
  }
  
  error = OS_ThreadCreate(PacketThread,
					NULL,
          &PacketThreadStack[THREAD_STACK_SIZE - 1],
	        PACKET_THREAD_PRIORITY);
  
  // Start multithreading - never returns!
  OS_Start();
}

/*!
 ** @}
 */
