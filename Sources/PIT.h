/*! @file
 *
 *  @brief Routines for controlling Periodic Interrupt Timer (PIT) on the TWR-K70F120M.
 *
 *  This contains the functions for operating the periodic interrupt timer (PIT).
 *
 *  @author PMcL
 *  @date 2015-08-22
 */

#ifndef PIT_H
#define PIT_H

// new types
#include "types.h"
#include "OS.h"

/*! @brief Sets up the PIT before first use.
 *
 *  Enables the PIT and freezes the timer when debugging.
 *  @param moduleClk The module clock rate in Hz.
 *  @param pointer to array of semaphores for signaling in the ISR
 *  @return bool - TRUE if the PIT was successfully initialized.
 *  @note Assumes that moduleClk has a period which can be expressed as an integral number of nanoseconds.
 */
bool PIT_Init(const uint32_t moduleClk, OS_ECB* semaphore[]);

/*! @brief Sets the value of the desired period of the PIT.
 *
 *  @param period The desired value of the timer period in nanoseconds.
 *  @param restart TRUE if the PIT is disabled, a new value set, and then enabled.
 *                 FALSE if the PIT will use the new value after a trigger event.
 *  @note The function will enable the timer and interrupts for the PIT.
 */
void PIT_Set(const uint32_t period, const bool restart);

/*! @brief gets the PIT1 timer value (used for DOR timestamping). Since the PIT timer decrements instead of
 *  incrementing, we will take the inverse of the time (ie. (2^32)-1 - PIT_CVAL1) for the sake of clarity in the
 *  DOR inverse timing (since real time goes up not down!)
 *
 *  @return returns the current time elapsed in ms
 */
uint32_t PIT_GetTime(void);

/*! @brief Enables or disables the PIT.
 *
 *  @param enable - TRUE if the PIT is to be enabled, FALSE if the PIT is to be disabled.
 */
void PIT_Enable(const bool enable);

/*! @brief Interrupt service routine for the PIT.
 *
 *  The periodic interrupt timer has timed out.
 *  The user callback function will be called.
 *  @note Assumes the PIT has been initialized.
 */
void __attribute__ ((interrupt)) PIT_ISR(void);

#endif
