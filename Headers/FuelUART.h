/****************************************************************************

  Header file for template Flat Sate Machine
  based on the Gen2 Events and Services Framework

 ****************************************************************************/

#ifndef FuelUART_H
#define FuelUART_H

// Event Definitions
#include "ES_Configure.h" /* gets us event definitions */
#include "ES_Types.h"     /* gets bool type for returns */

// typedefs for the states
// State definitions for use with the query function
typedef enum
{
  InitPState, Idle, Transmitting, Receiving
}FuelUARTState_t;

// Public Function Prototypes

bool InitFuelUART(uint8_t Priority);
bool PostFuelUART(ES_Event_t ThisEvent);
ES_Event_t RunFuelUART(ES_Event_t ThisEvent);
FuelUARTState_t QueryFuelUART(void);

void UART_ISR ( void );     // UART1 interrupt service routine 

#endif /* FuelUART_H */

