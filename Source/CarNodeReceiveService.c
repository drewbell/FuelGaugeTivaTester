/*
 ============================================================================
 Name: ControllerReceiveService.c
 Author: Michal Rittikaidachar
 Version:1.0
 Date Created: 5/90/17
 Description: Service to implement receiveing on xbee through uart
 Notes: -Code adapted from and based off of skeleton files by Ed Carryer
 
 
					Software History
 When           Who     What/Why
 -------------- ---     --------
 10/25/17				MR			Started Skeleton
 
 
 ============================================================================
*/
/*----------------------------- Include Files -----------------------------*/
/* include header files for the framework and this service */

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

#include "Constants.h"
#include "CarNodeReceiveService.h"
#include "CarNodeMainService.h"


/*----------------------------- Module Defines ----------------------------*/
// these times assume a 1.000mS/tick timing
#define ONE_SEC 1000
#define FIFTH_SECOND 200
#define HALF_SEC (ONE_SEC/2)
#define TWO_SEC (ONE_SEC*2)
#define FIVE_SEC (ONE_SEC*5)
#define DEFFERRAL_QUE_SIZE 4   
#define SEC_PER_MIN 60
#define PRINT_DELAY 100 //in ms
#define TICKS_PER_MS 40000
#define MS_PER_MIN 60000
#define ALL_BITS (0xff<<2)
#define BITS_PER_NIBBLE 4
#define PORT_A BIT0HI
#define PORT_B BIT1HI
#define PORT_C BIT2HI
#define PORT_D BIT3HI
#define PORT_E BIT4HI
#define PORT_F BIT5HI
#define UART_RX_PIN GPIO_PIN_0	// Defined in AFSEL Table on page 1351
#define UART_TX_PIN	GPIO_PIN_1	// Defined in AFSEL Table on page 1351

#define TESTING_FARMER_RECEIVE

// Packet Types - See Constants.h


#define FRAME_DATA_START 3
#define DATA_OVERHEAD 4



/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this service.They should be functions
   relevant to the behavior of this service */
void ReceiveISR ( void );
static uint8_t CalculateCheckSum (void);

/*---------------------------- Module Variables ---------------------------*/
// with the introduction of Gen2, we need a module level Priority variable
static uint8_t MyPriority;
// add a deferral queue for up to 3 pending deferrals +1 to allow for overhead
static ES_Event DeferralQueue[DEFFERRAL_QUE_SIZE];
static ReceiveState_t CurrentState;
static bool CheckSumError=false;

static uint8_t FrameDataSize;
static uint16_t ReceivedPacketSize=0;

static uint8_t XbeeCheckSum; // XBEE Calcluated checksum (last byte of transfer)
static uint8_t ReceiveCheckSum;  // Checksum calculated from received data see page 43 of XBEE datasheet for calculation instructions

//The ReceivedPacket array stores the incoming data: [API Identifier (index 0), ....., RF Data]
static uint8_t ReceivedPacket[5][LONGEST_PACKET];  // Statically allocated to provide enough space for the largest expected packet (the encryption key)
static uint8_t ReceiveBuffer[LONGEST_PACKET];		// buffer for receiving packets
static uint8_t WritePacketNumber=0;
static uint8_t ReadPacketNumber=0;
/*------------------------------ Module Code ------------------------------*/
/*===========================================================================
 Function: initFarmerReceiveService
 Parameters: uint8_t : the priorty of this service
 Returns: bool, false if error in initialization, true otherwise
 Description:  Saves away the priority, and does any other required initialization for this service
 Author: Michal Rittikaidachar 11/3/16
 Notes: Code adapted from and based off of skeleton files by Ed Carryer
===========================================================================*/
bool InitCarNodeReceiveService ( uint8_t Priority ){
		#ifdef TESTING_CAR_NODE_RECEIVE
				printf("\rCR: Initializing Farmer Receive Service Hardware\n");
		#endif
	
		ES_Event ThisEvent;
		MyPriority = Priority;
 
		// Initialize Defferal Que
		ES_InitDeferralQueueWith(DeferralQueue, ARRAY_SIZE(DeferralQueue)); 
		

	
	// post the initial transition event
		ThisEvent.EventType = ES_INIT;
		if (ES_PostToService( MyPriority, ThisEvent) == true){  
				return true;
		}
		else{
			return false;
		}
}

/*
 ============================================================
 Function: PostFarmerReceiveService
 Parameters: EF_Event ThisEvent ,the event to post to the queue
 Returns: bool false if the Enqueue operation failed, true otherwise
 Description: Posts an event to this state machine's queue
 Author: Michal Rittikaidachar 11/3/16
 Notes: Code adapted from and based off of skeleton files by Ed Carryer
 ============================================================
*/

bool PostCarNodeReceiveService( ES_Event ThisEvent )
{
  return ES_PostToService( MyPriority, ThisEvent);
}

/*===========================================================================
 Function: RunFarmerReceiveService
 Parameters: ES_Event : the event to process
 Returns: ES_Event, ES_NO_EVENT if no error ES_ERROR otherwise
 Description: Runs State Machine for Farmer Service.  
 Author: Michal Rittikaidachar 11/3/16
 Notes: Code adapted from and based off of skeleton files by Ed Carryer
===========================================================================*/

ES_Event RunCarNodeReceiveService( ES_Event ThisEvent )
{

  ES_Event ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT; // assume no errors
  			#ifdef TESTING_CAR_NODE_RECEIVE
	printf("\r\nCR: Entering Run Farmer ReceiveService\n");
				#endif
	switch (CurrentState){

			case InitReceive:
					if ( ThisEvent.EventType == ES_INIT ){
							#ifdef TESTING_CAR_NODE_RECEIVE
									printf("\rCR: Initializing Farmer Receive Service State Machine\n");
							#endif
						
							// switch states to Run
							CurrentState = Run;	
						
							#ifdef TESTING_CAR_NODE_RECEIVE
								printf("\rCR: Switched FarmerReceive Service to WaitingFor0x7E\n\r");
							#endif
	
						}
			break;
						
				case Run:
					
					// If we get a receive complete event
					if ( ThisEvent.EventType == ES_RECEIVE_COMPLETE){
							
							#ifdef TESTING_CAR_NODE_RECEIVE
									printf("\rCR: RECEIVE_COMPLETE event\n");
									printf("\rCR:  Received %d Bytes \n", ReceivedPacketSize);
									for(int i=0; i<ReceivedPacketSize; i=i+1){
											printf("\rCR: 0x%4x\n",ReceivedPacket[i]);
									}
									printf("\r \n");
							#endif		
							

							//If our Check Sums Match and they are correct 
							if(CheckSumError==false){ 
							
									//post receivedpacket to farmer Receive
									ThisEvent.EventType = ES_RECEIVED_PACKET;
									ThisEvent.EventParam= ReceivedPacketSize;
									PostCarNodeMainService(ThisEvent);
									#ifdef TESTING_CAR_NODE_RECEIVE
											printf("\rCR: Good check sum posting even to main\n");		
									#endif
							}
							
							// else our checksum is bad so do nothing							
							else{
								printf("\rCR: ***************************************************\n");
								printf("\rCR: Bad Check Sum\n");		
								printf("\rCR: ***************************************************\n");
							}
					} //end if EV_RECEIVE Complete
	
					else{
						
						printf("\rCR: ***************************************************\n");	
						printf("\rCR: ERROR: Event not handeled in receiving frame data\n");	
						printf("\rCR: ***************************************************\n");
					}
					
			break;  // break run state 
					
			default:
						printf("\rCR: ***************************************************\n");	;	
						printf("\rCR: ERROR: State Not handeled\n");	
						printf("\rCR: ***************************************************\n");	
			break;
					
		} // end switch case for state machine
			
  	#ifdef TESTING_CAR_NODE_RECEIVE
				printf("\rCR:  Exiting Run Farmer Receive Service\n");
		#endif		
		
return ReturnEvent;

}// end run ControllerReceiveService


/*===========================================================================
 Function: GetReceivedPacket
 Parameters: None
 Returns: uint8_t Checksum:  returns pointer to received packet array
 Description: 
 Author: Michal Rittikaidachar 5/12/17
 Notes: 
===========================================================================*/
uint8_t *GetReceivedPacket (void){ 
uint8_t returnVal=ReadPacketNumber;
ReadPacketNumber=(ReadPacketNumber+1)%5;
	//printf("\rRead:%d\n",returnVal);

	return  ReceivedPacket[returnVal];
	
} // End CalculateCheckSum Function

/*----------------------------- Private Functions -----------------------------*/


/*===========================================================================
 Function: CalculateCheckSum
 Parameters: None
 Returns: uint8_t Checksum:  returns the calculated checksum value
 Description: calculates and returrns the checksum of current transmit message and ret
 Author: Michal Rittikaidachar 5/10/17
 Notes: 
===========================================================================*/
static uint8_t CalculateCheckSum (void){ 
		uint8_t Index;
		uint8_t	RollingSum=0;
		uint8_t	CheckSum;
	
	
		FrameDataSize=ReceivedPacketSize-1-FRAME_DATA_START;
		//itrerate through frame data and calculate sum 
		for ( Index = FRAME_DATA_START; Index < FRAME_DATA_START + FrameDataSize ; Index=Index+1 ) {
				// Add current receivedPacket byte to rolling sum
				RollingSum = RollingSum+ ReceiveBuffer[Index];
			
	} 
	//subtract rolling sum from 0xff to get checksum
	CheckSum=0xff-RollingSum;
	return CheckSum;
	
} // End CalculateCheckSum Function




/*===========================================================================
 Function: ReceiveISR
 Parameters: None
 Returns: None
 Description: ISR for UART RX
 Author: Michal Rittikaidachar 5/10/17
 Notes: - This ISR is partially specific to the zigbee protocol as it will skip 
===========================================================================*/
void ReceiveISR ( void ){
static uint16_t PacketLength;
static uint16_t BytesReceived=0;	
	//printf("\r\nCR: ISR\n");
		//Clear the receive interrupt source in the UARTICR (UART Masked Interrpt Clear Register)
		HWREG(UART1_BASE + UART_O_ICR) |= UART_ICR_RXIC;
	
		//Read the data register (UARTDR) and store in ReceivedByte
		uint8_t ReceivedByte = HWREG(UART1_BASE + (UART_O_DR));
	
		// Check the Error Flags
		uint8_t OverrunErrorFlag = HWREG(UART1_BASE + (UART_O_RSR)) & (UART_RSR_OE);
		uint8_t BreakErrorFlag = HWREG(UART1_BASE + (UART_O_RSR)) & (UART_RSR_BE);
		uint8_t ParityErrorFlag = HWREG(UART1_BASE + (UART_O_RSR)) & (UART_RSR_PE);
		uint8_t FramingErrorFlag = HWREG(UART1_BASE + (UART_O_RSR)) & (UART_RSR_FE);
		
		//If no errors send event to transmit service
		if((OverrunErrorFlag==0) && (BreakErrorFlag==0) && (ParityErrorFlag==0) && (FramingErrorFlag==0)){

				if (ReceivedByte== 0x7E && BytesReceived==0){

						//save new byte
						ReceiveBuffer[0]=ReceivedByte;
						
						// set current packet size
						BytesReceived=1;
						
					
				}
				
				// if its the second byte must be Length MSB
				else if (BytesReceived==1){
					
						PacketLength=ReceivedByte;
						PacketLength=ReceivedByte<<8;
						
						//save new byte
						ReceiveBuffer[BytesReceived]=ReceivedByte;
						
						// set current packet size
						BytesReceived=BytesReceived+1;	
				}
				
				// if its the third byte must be length LSB
				else if (BytesReceived==2){

						PacketLength=PacketLength+ReceivedByte;
					
						//save new byte
						ReceiveBuffer[BytesReceived]=ReceivedByte;
						
						// set current packet size
						BytesReceived=BytesReceived+1;	

				}
					
				else if (BytesReceived>2 && BytesReceived< PacketLength+DATA_OVERHEAD-1){

						//save new byte
						ReceiveBuffer[BytesReceived]=ReceivedByte;
						
						// set current packet size
						BytesReceived=BytesReceived+1;	
	
				}
				
				//else we are on last byte
				else{
						//save new byte
						ReceiveBuffer[BytesReceived]=ReceivedByte;
					
						// set current packet size
						BytesReceived=BytesReceived+1;
					
						ReceivedPacketSize=BytesReceived;
						BytesReceived=0;
				
						//determine if packet is valid by checksum
						XbeeCheckSum=ReceiveBuffer[ReceivedPacketSize-1];
						ReceiveCheckSum=CalculateCheckSum();
						//If our Check Sums Match and they are correct 
						if(XbeeCheckSum==ReceiveCheckSum){ 
								CheckSumError=false;
								//post receive complete event to main service		
								ES_Event ThisEvent;
								ThisEvent.EventType = ES_RECEIVED_PACKET;
								ThisEvent.EventParam= ReceivedPacketSize;
								PostCarNodeMainService(ThisEvent);
							
								// Copy to ringbuffer
								memcpy(ReceivedPacket[WritePacketNumber], ReceiveBuffer, 41);
							
							
								// Make sure that we have do not overwrite data 
								if(((WritePacketNumber+1)%5)!=ReadPacketNumber){
										WritePacketNumber=(WritePacketNumber+1)%5;
										//printf("\rWRITE:%d\n",WritePacketNumber);
								}
								
								// We have received more data than can be placed in buffer
								else{
									//printf("\rERROR: RING BUFFER TOO SMALL \n",WritePacketNumber);
								}
						} // end if received xbee checksum matches calculated checksum
						
						else{
								CheckSumError=true;
													printf("\rCR: ***************************************************\n");		
													printf("\rCR: Check Sum Srror d\n");	
													printf("\rCR: ***************************************************\n");	
						}// else checksum was bad so set flag
		
				} // end else last byte
			
		} // end if no errors
		
		//TODO Error handling on receive errors
		else{//Else, respond to the error
				//clear the UARTRSR error flags by writing any value of the UARTECR register
				HWREG(UART1_BASE + UART_O_ECR) |= (UART_ECR_DATA_M);
		}//end if we have errors
	
} //end ReceiveISR

/*------------------------------- Footnotes -------------------------------*/
/*------------------------------ End of file ------------------------------*/

