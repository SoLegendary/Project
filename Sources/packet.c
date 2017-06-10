/*!
**  @file Packet.c
**
**  @brief Contains functions for handling packets in accordance with the Tower Communication Protocol document
**         Packet parameters are sent to and taken out of the UART module via these functions as well
**         having parameters such as checksum verified to ensure valid packets are being read
*/
/*!
**  @addtogroup main_module main module documentation
**
**  @author Thanit Tangson & Emile Fadel
**  @{
*/
/* MODULE packet */

#include "packet.h"
#include "UART.h"
#include "FTM.h"
#include "LEDs.h"
#include "OS.h"


TPacket Packet; // Declaration of new packet structure as of lab 2
const uint8_t PACKET_ACK_MASK = 0x80; // Acknowledgment Bit Mask in Hex



bool Packet_Init(const uint32_t baudRate, const uint32_t moduleClk)
{
  return UART_Init(baudRate, moduleClk); // Simply send parameters along to UART_Init
}


// Might not be able to use a while loop for lab 5?

bool Packet_Get(void)
{
  static uint8_t packetArray[5] = {0}; // Array to temporarily hold packet parameters before they form a full packet
  static uint8_t packetIndex    = 0; // Index to the packet array

  // As long as there are parameters in the FIFO, keep taking them out
  while (UART_InChar(&packetArray[packetIndex]))
  {

    // Once we have all 5 elements of a packet we can begin testing for validity and using it
    if (packetIndex == 4)
    {
      // XOR of all previous parameters is the valid checksum
      uint8_t validChecksum = (packetArray[0] ^ packetArray[1]) ^ (packetArray[2] ^ packetArray[3]);

      if (packetArray[4] == validChecksum) // If the checksum is valid copy the packet across
      {
        Packet_Command    = packetArray[0];
        Packet_Parameter1 = packetArray[1];
        Packet_Parameter2 = packetArray[2];
        Packet_Parameter3 = packetArray[3];
        Packet_Checksum   = packetArray[4];

        // Concatenated parameters
        Packet_Parameter12 = (((uint16_t)(Packet_Parameter1 << 8)) || ((uint16_t)(Packet_Parameter2)));
        Packet_Parameter23 = (((uint16_t)(Packet_Parameter2 << 8)) || ((uint16_t)(Packet_Parameter3)));

	packetIndex = 0; // Reset packetIndex to allow a new packet to be built

	// Upon receiving a valid packet from the PC, turn on the Blue LED for 1s
	TFTMChannel FTM0Channel0; // Struct to start a timer in Channel 0
	FTM0Channel0.channelNb  = 0;
	FTM0Channel0.delayCount = 1;

	return true;
      }
      else // If the checksum fails to validate shift the packet along so it can eventually re-sync
      {
        for (uint8_t i = 0 ; i < 4 ; i++)
          packetArray[i] = packetArray[i+1];
      }
    }
    else // Increment the index so that we can get the next byte in the packet from the FIFO
      packetIndex++;
  }

  return false; // If there is nothing in the FIFO, return false
}



bool Packet_Put(const uint8_t command, const uint8_t parameter1, const uint8_t parameter2, const uint8_t parameter3)
{
  // Creating the checksum, which is the XOR of all previous parameters
  uint8_t checksum = (command ^ parameter1) ^ (parameter2 ^ parameter3);

  // Return the entire packet, one parameter at a time
  return ((UART_OutChar(command)) &&
          (UART_OutChar(parameter1)) &&
          (UART_OutChar(parameter2)) &&
          (UART_OutChar(parameter3)) &&
          (UART_OutChar(checksum)));
}



/* END packet */
/*!
** @}
*/
