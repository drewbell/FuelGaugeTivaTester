/****************************************************************************
 Module
   Fuel_UART.c

 Revision
   1.0.1

 Description
   This is a template file for implementing flat state machines under the
   Gen2 Events and Services Framework.

 Notes

 History
 When           Who     What/Why
 -------------- ---     --------
 01/15/12 11:12 jec      revisions for Gen2 framework
 11/07/11 11:26 jec      made the queue static
 10/30/11 17:59 jec      fixed references to CurrentEvent in RunTemplateSM()
 10/23/11 18:20 jec      began conversion from SMTemplate.c (02/20/07 rev)
****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
/* include header files for this state machine as well as any machines at the
   next lower level in the hierarchy that are sub-machines to this machine
*/
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "FuelUART.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "ES_DeferRecall.h"
#include "ES_ShortTimer.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"	// Define PART_TM4C123GH6PM in project
#include "driverlib/gpio.h"
#include "inc/hw_pwm.h"
#include "inc/hw_timer.h"
#include "inc/hw_nvic.h"
#include "inc/hw_uart.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_sysctl.h"

/*----------------------------- Module Defines ----------------------------*/

#define UART_RX_PIN GPIO_PIN_0	// Defined in AFSEL Table on page 1351
#define UART_TX_PIN	GPIO_PIN_1	// Defined in AFSEL Table on page 1351
#define BITS_PER_NIBBLE 4
#define FUEL_QUERY 0xAA
#define FUEL_CHECK 0x08         // bit 3 set if tank is fueld
#define FUEL_LEVEL_MASK 0x07         // bit 3 set if tank is fueld

/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this machine.They should be functions
   relevant to the behavior of this state machine
*/
void TransmitISR(void);
void ReceiveISR(void);

/*---------------------------- Module Variables ---------------------------*/
// everybody needs a state variable, you may need others as well.
// type of state variable should match htat of enum in header file
static FuelUARTState_t CurrentState;

// with the introduction of Gen2, we need a module level Priority var as well
static uint8_t MyPriority;

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitFuelUART

 Parameters
     uint8_t : the priorty of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Saves away the priority, sets up the initial transition and does any
     other required initialization for this state machine
 Notes

 Author
     J. Edward Carryer, 10/23/11, 18:55
****************************************************************************/
bool InitFuelUART(uint8_t Priority)
{
  ES_Event_t ThisEvent;

  MyPriority = Priority;
  // put us into the Initial PseudoState
  CurrentState = InitPState;
  // post the initial transition event
  ThisEvent.EventType = ES_INIT;
  if (ES_PostToService(MyPriority, ThisEvent) == true)
  {
    return true;
  }
  else
  {
    return false;
  }
}

/****************************************************************************
 Function
     PostFuelUART

 Parameters
     EF_Event ThisEvent , the event to post to the queue

 Returns
     boolean False if the Enqueue operation failed, True otherwise

 Description
     Posts an event to this state machine's queue
 Notes

 Author
     J. Edward Carryer, 10/23/11, 19:25
****************************************************************************/
bool PostFuelUART(ES_Event_t ThisEvent)
{
  return ES_PostToService(MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunFuelUART

 Parameters
   ES_Event : the event to process

 Returns
   ES_Event, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
   add your description here
 Notes
   uses nested switch/case to implement the machine.
 Author
   J. Edward Carryer, 01/15/12, 15:23
****************************************************************************/
ES_Event_t RunFuelUART(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT; // assume no errors
  
  //printf("\n\rRun Fuel UART.");
  
  switch (ThisEvent.EventType){
    case ES_INIT:
      // Initialize UART TX & RX Interrupts: Pages 924 and 925  		
      HWREG(UART1_BASE + UART_O_IM) |= UART_IM_TXIM;  
      HWREG(UART1_BASE + UART_O_IM) |= UART_IM_RXIM;
      break;
    case ES_FUEL_QUERY:  //If event is event one
      // If TXFE is set there is room to transfer a byte as the fifo as diabled (See Pages 911 and 912 of DataSheet)
      if ( HWREG(UART1_BASE + UART_O_FR) & UART_FR_TXFE ) {
        HWREG(UART1_BASE + UART_O_DR) = (HWREG(UART1_BASE + UART_O_DR) & 0xffffff00) + FUEL_QUERY;
        printf("\n\rTX to Fuel Gauge: 0x%x", FUEL_QUERY);
      }
      else printf("\n\rTX to Fuel Gauge: ERROR, no space in UART1");
      
      break;
    
    case ES_UART_RXMSG:
    {
      uint8_t rx_msg = ThisEvent.EventParam;
      if( ((0xF0 & ~rx_msg) >> 4) == (0x0F & rx_msg)){   // perform error checking per spec
        if(FUEL_CHECK & rx_msg)
          printf("\n\rRX from Fuel Gauge. State: FUELED, Level %d/7", rx_msg & FUEL_LEVEL_MASK);
        else
          printf("\n\rRX from Fuel Gauge. State: EMPTY");
      }
      else printf("\n\rRX from Fuel Gauge: bitwise error check FAILED, data: 0x%x", rx_msg);
      break;    
    }
    
    case ES_UART_BAD_RXMSG: 
      printf("\n\rUART Error in receiving message from FuelGauge.");
      break;
    
    case ES_BAD_FUEL_QUERY:
      printf("\n\rBAD Fuel Query Key Received.");
      break;
    
    case ES_TRANSMIT_COMPLETE:
      //printf("\n\rUART 1 Transmit Complete");
      break; 
   
    default:
      printf("\n\rUnhandled event in RunFuelUART with enum %d and parameter %d", 
          ThisEvent.EventType, ThisEvent.EventParam);
  }

                                // end switch on Current State
  return ReturnEvent;
}

/****************************************************************************
 Function
     QueryFuelUART

 Parameters
     None

 Returns
     TemplateState_t The current state of the Template state machine

 Description
     returns the current state of the Template state machine
 Notes

 Author
     J. Edward Carryer, 10/23/11, 19:21
****************************************************************************/
FuelUARTState_t QueryFuelUART(void)
{
  return CurrentState;
}

/***************************************************************************
 private functions
 ***************************************************************************/

/*===========================================================================
 Function: InitializeUART
 Parameters: None
 Returns: Boolean to indicate success or failure of initialization
 Description: Initializes  Uart1 on Tiva: full duplex, 8 bit data, 9600 Baud, single Byte FIFO
 Author: Michal Rittikaidachar 5/9/17
 Notes: -Code adapted from and based off of skeleton files by Ed Carryer
				- Refer to section 14 of Tiva Datasheet
				- Configuration steps listed on pages 902-903
===========================================================================*/
bool InitializeUART (void){ 
	
		// Enable the clock to the UART1 module using the RCGCUART (run time gating clock control) register
		HWREG(SYSCTL_RCGCUART) |= SYSCTL_RCGCUART_R1; 	//Enable UART1
	
		// Wait for the UART to be ready using the PRUART (peripheral ready register)
		while ((HWREG(SYSCTL_PRUART) & SYSCTL_PRUART_R1) != SYSCTL_PRUART_R1); // Check UART1 Bit in the ready register
	
		// Enable the clock to the appropriate GPIO module via the RCGCGPIO
		HWREG(SYSCTL_RCGCGPIO) |= SYSCTL_RCGCGPIO_R1; 	//Enable GPIO Port B for UART1
		
		// Wait for the GPIO module to be ready (PRGPIO)
		while ((HWREG(SYSCTL_PRGPIO) & SYSCTL_PRGPIO_R1) != SYSCTL_PRGPIO_R1);
	
		// Configure the GPIO pins for RX and TX lines defined in AFSEL Table on page 1351
		HWREG(GPIO_PORTB_BASE + GPIO_O_DEN) |= (UART_TX_PIN | UART_RX_PIN);	// Enable pins as digital
		HWREG(GPIO_PORTB_BASE + GPIO_O_DIR) &= ~(UART_RX_PIN);	// Set as input
		HWREG(GPIO_PORTB_BASE + GPIO_O_DIR) |= (UART_TX_PIN);		// Set as ouptut
		
		// Select the Alternate function for the UART pins (AFSEL)
		// Set bits corresponding to RX and TX lines in port B 
		HWREG(GPIO_PORTB_BASE + GPIO_O_AFSEL) |= (UART_TX_PIN | UART_RX_PIN);
		
		// Configure the PMCn fields in the GPIOPCTL register to assign the UART pins 
		// GPIOPCTL Register on page 689- Configuration for pin 0 and 1 for RX TX lines in lowest 2 nibbles corresponding to mask 0xfffffff0
		// AFSEL table on pg 1351- write 1 to corresponding nibbles in GPIOPCTL register
		HWREG(GPIO_PORTB_BASE + GPIO_O_PCTL) = (HWREG(GPIO_PORTB_BASE + GPIO_O_PCTL) & 0xfffffff0) + (1); //write 1 to PB0 to select U1Rx as the alternate function
		HWREG(GPIO_PORTB_BASE + GPIO_O_PCTL) = (HWREG(GPIO_PORTB_BASE + GPIO_O_PCTL) & 0xffffff0f) + (1<<(1*BITS_PER_NIBBLE)); //write 1 to PB1 to select U1Tx as the alternate function
		
		// Disable the UART by clearing the UARTEN bit in the UARTCTL register (Pages 918-921)
		//Do nothing because the UARTEN bit is already disabled upon reset
	
		// Set Baud Rate: SYSCLCK/(16*BaudRate)
		// Write the integer portion of the BRD to the UARTIBRD register 
		HWREG(UART1_BASE + UART_O_IBRD) = HWREG(UART1_BASE + UART_O_IBRD) + 0x104;  // value to write is decimal 260 or hex 0x104
		

		// Write the factional portion of the BRD to the UARTFBRD register Round(DecimalRemainder*64+.5) 
		HWREG(UART1_BASE + UART_O_FBRD) = HWREG(UART1_BASE + UART_O_FBRD) + 0x1B; // value to write is decimal 27 or hex 0x1B
		
		// Write the desired serial parameters to the UARTLCRH register
		//One stop bit is the default for this register, so we don't need to do anything to configure the stop bits
		
		//Set the word length to 8 bits by writing 0x3 to bits <6:5> to set WLEN to 0x3 (Pages 916-917)
		HWREG(UART1_BASE + UART_O_LCRH) = HWREG(UART1_BASE + UART_O_LCRH) + UART_LCRH_WLEN_8;
	
		// Configure the UART operation using the UARTCTL register
		//Do nothing because RXE and TXE are already enabled, and HSE comes out of reset as 0, which is what you want
	
		// Enable the UART by setting the UARTEN bit in the UARTCTL register
		HWREG(UART1_BASE + UART_O_CTL) |= (UART_CTL_UARTEN);
    
    // Enable loop back mode
		//HWREG(UART1_BASE + UART_O_CTL) |= (UART_CTL_LBE);
			
		//Enable nvi
		HWREG(NVIC_EN0) |= BIT6HI;
		
		//Enable Global Interrupts
		__enable_irq();
		
		#ifdef TESTING_CONTROLLER_MAIN
				printf("\rCNM: Finished UART Initialization \n");
		#endif
		return true;

} // End Initialize UART Function


/*===========================================================================
 Function: UART_ISR
 Parameters: None
 Returns: ISR for UART that calls transmit and receive specific ISRs
 Description: ISR routine for UART
 Author: Michal Rittikaidachar 5/10/17
 Notes: -Code adapted from and based off of skeleton files by Ed Carryer

===========================================================================*/

void UART_ISR ( void ) {
  //printf("\n\rUART ISR");

	// If TXMIS is set we have a transmit interrupt and can now send another byte (Page  930 and 931)
	if (HWREG(UART1_BASE + UART_O_MIS) & UART_MIS_TXMIS) {
			TransmitISR();
	}
	else {      // else we got a receive interrupt 
			ReceiveISR();
	} //end if receive interrupt
}


/*===========================================================================
 Function: TransmitISR
 Parameters: None
 Returns: None
 Description: ISR for UART TX
 Author: Michal Rittikaidachar 5/10/17
 Notes: 
===========================================================================*/
void TransmitISR ( void ) {
  //printf("\n\rTransmitISR");
  // clear the TX intrerrupt:  Set TXIC in UARTICR to clear tx interrupt (Page 933 & 934)
  HWREG(UART1_BASE + UART_O_ICR) |= UART_ICR_TXIC;
  
  HWREG(UART1_BASE + UART_O_IM) &= ~UART_IM_TXIM;     // disable TX interrupts 

  // Post ES_EV_TRANSMIT_COMPLETE to this service
  ES_Event_t ThisEvent;
  ThisEvent.EventType = ES_TRANSMIT_COMPLETE;
  PostFuelUART( ThisEvent );
} 

/*===========================================================================
 Function: ReceiveISR
 Parameters: None
 Returns: None
 Description: ISR for UART RX
 Author: Michal Rittikaidachar 5/10/17
 Notes: - This ISR is partially specific to the zigbee protocol as it will skip 
===========================================================================*/
void ReceiveISR(void){
    //printf("\n\rReceiveISR");
		//Clear the receive interrupt source in the UARTICR (UART Masked Interrupt Clear Register)
		HWREG(UART1_BASE + UART_O_ICR) |= UART_ICR_RXIC;
	
		//Read the data register (UARTDR) and store in ReceivedByte
		uint8_t ReceivedByte = HWREG(UART1_BASE + (UART_O_DR));
	
		// Check the Error Flags
		uint8_t OERR = HWREG(UART1_BASE + (UART_O_RSR)) & (UART_RSR_OE);
		uint8_t BERR = HWREG(UART1_BASE + (UART_O_RSR)) & (UART_RSR_BE);
		uint8_t PERR = HWREG(UART1_BASE + (UART_O_RSR)) & (UART_RSR_PE);
		uint8_t FERR = HWREG(UART1_BASE + (UART_O_RSR)) & (UART_RSR_FE);
		
    ES_Event_t RxEvent;
		if(!OERR && !BERR && !PERR && !FERR){
      RxEvent.EventType = ES_UART_RXMSG;  // if no errors
      RxEvent.EventParam = ReceivedByte;
		} 
		else{   //Else, notify of errors
      RxEvent.EventType = ES_UART_BAD_RXMSG;
      //clear the UARTRSR error flags by writing any value of the UARTECR register
      HWREG(UART1_BASE + UART_O_ECR) |= (UART_ECR_DATA_M);
		}
    PostFuelUART(RxEvent);
	
} //end ReceiveISR

