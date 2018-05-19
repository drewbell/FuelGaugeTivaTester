

/*
 ============================================================================
 Name: CarNodeTransmitService.c
 Author: Michal Rittikaidachar
 Version:1.0
 Date Created: 5/9/17
 Description: Service to transmit from CarNode to Controller with XBEE Raadios via UART. 
 Notes: Code adapted from and based off of skeleton files by Ed Carryer
 
 
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
#include "CarNodeTransmitService.h"
#include "CarNodeMainService.h"
#include "CAN_Conversion.h"
#include "CanService.h"




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

// Packet Types - See Constants.h

// Frame Data Defines
#define START_DELIMTER 0x7E
#define API_ID 0x01									// transmit Request (16 Bit Address) see page 45 of xbee dtat sheet for description
#define FRAME_DATA_OVERHEAD	5				// API ID (1 BYTE),Frame ID (1 BYTE), DEST Address (2 BYTES), Options (1 BYTE)
#define OPTIONS 0x00							// keep acknowledge enabled: See page 45 of xbee datasheet
#define LENGTH_PACKET_MSB 0x00			
#define RF_DATA_START	8							// index to where RF data starts 	includes all bits 
#define FRAME_DATA_START 3
#define PACKET_SIZE_OVERHEAD 9	

// RF Data sizes
#define STATUS_SIZE 4																	// PacketType (1 BYTE), Status Bytes (3)
#define PAIR_ACK_SIZE 21																	// PacketType (1 BYTE), VIN Number (17 BYTES),  Status Bytes (3 BYTE) 


/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this service.They should be functions
   relevant to the behavior of this service */
	 
static void ConstructTransmitHeader (void);
static uint8_t CalculateCheckSum (void);
bool ConstructTransmitPacket (uint8_t PacketType);
static bool BeginTransmit (void);
uint8_t getStatusDataBytes(uint8_t byteNumber);


/*---------------------------- Module Variables ---------------------------*/
// with the introduction of Gen2, we need a module level Priority variable
static uint8_t MyPriority;
// add a deferral queue for up to 3 pending deferrals +1 to allow for overhead
static ES_Event DeferralQueue[DEFFERRAL_QUE_SIZE];
static TransmitState_t CurrentState;

// Bytes to specify length of packet see page 44 of xbee data sheet for calculation instructions
// Length [Bytes] = API Identifier + Frame ID + AT Command + Parameter Value
static uint8_t LengthPacketLSB;
static uint8_t FrameDataSize;
static uint8_t FrameID=0x00;
static uint8_t DestinationAddressMSB=0x00;
static uint8_t DestinationAddressLSB=0x00;
static uint8_t PacketSize;

static uint8_t TransmitCheckSum;  // Checksum to be included with transmit packet see page 43 of XBEE datasheet for calculation instructions
static uint8_t BytesSent;

//The TransmitPacket array stores the outgoing data: [API Identifier (index 0), ....., RF Data]
static uint8_t TransmitPacket[LONGEST_PACKET];  // Staticly allocated to provide enough space for the largest expected packet (PAIR_ACK)
static uint8_t LastPacketSent[LONGEST_PACKET];

uint8_t packetNumber=0;

//Status Byte variables;
static uint8_t defrostState;
static int16_t vehicleSpeedInBits;

/*------------------------------ Module Code ------------------------------*/
/*===========================================================================
 Function: InitCarNodeTransmitService
 Parameters: uint8_t : the priorty of this service
 Returns: bool, false if error in initialization, true otherwise
 Description:  Saves away the priority, and does any other required initialization for this service
 Author: Michal Rittikaidachar 11/3/16
 Notes: Code adapted from and based off of skeleton files by Ed Carryer
===========================================================================*/
bool InitCarNodeTransmitService ( uint8_t Priority ){
		#ifdef TESTING_CONTROLLER_TRANSMIT
				printf("\rCNT: Initializing CarNode Transmit Service Hardware\n");
		#endif
	
		ES_Event ThisEvent;
		MyPriority = Priority;
 
		// Initialize Defferal Queue
		ES_InitDeferralQueueWith(DeferralQueue, ARRAY_SIZE(DeferralQueue)); 
	
		// Set Initial State
		CurrentState =  InitTransmit;	
		
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
 Function: PostCarNodeTransmitService
 Parameters: EF_Event ThisEvent ,the event to post to the queue
 Returns: bool false if the Enqueue operation failed, true otherwise
 Description: Posts an event to this state machine's queue
 Author: Michal Rittikaidachar 11/3/16
 Notes: Code adapted from and based off of skeleton files by Ed Carryer
 ============================================================
*/

bool PostCarNodeTransmitService( ES_Event ThisEvent )
{
  return ES_PostToService( MyPriority, ThisEvent);
}

/*===========================================================================
 Function: RunCarNodeTransmitService
 Parameters: ES_Event : the event to process
 Returns: ES_Event, ES_NO_EVENT if no error ES_ERROR otherwise
 Description: Runs State Machine for Car Node Transmit Service.  
 Author: Michal Rittikaidachar 11/3/16
 Notes: Code adapted from and based off of skeleton files by Ed Carryer
===========================================================================*/

ES_Event RunCarNodeTransmitService( ES_Event ThisEvent )
{

  ES_Event ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT; // assume no errors
                #ifdef TESTING_CONTROLLER_TRANSMIT
					printf("\r\nCNT: Entering Run CarNode Transmit Service\n");
				#endif
	switch (CurrentState){

			case InitTransmit:
					if ( ThisEvent.EventType == ES_INIT ){
							#ifdef TESTING_CONTROLLER_TRANSMIT
									printf("\rCNT: Initializing Car Node Transmit Service State Machine\n");
							#endif
						
							// switch states to unpaired
							CurrentState = WaitingToStart;	
							#ifdef TESTING_CONTROLLER_TRANSMIT
								printf("\rCNT: Switched Car Node Transmit Service to Waiting2Start\n\r");
							#endif
						}
			break;
						
			case WaitingToStart:
				
					// If we get a Begin_Transmit Event
					if ( ThisEvent.EventType == ES_BEGIN_TRANSMIT ){
						
							#ifdef TESTING_CONTROLLER_TRANSMIT
									printf("\rCNT: Received Begin Transmit in Car Node Transmit\n");
							#endif
							uint8_t PacketType=ThisEvent.EventParam;
						
							//if transmit packet constructed reset number of bytes sent and begin transmit
							if(ConstructTransmitPacket(PacketType)){
									BytesSent=0; //reset bytes sent (This is also the index into the next byte to transmit)
									BeginTransmit();
									CurrentState=Transmitting;
							} // End if packet constructed
							
							// if packet not constructed	
							else{
									printf("\r CNT: ERROR-TRANSMIT PACKET NOT CREATED\n");
							}
					} // End if begin transmit event
                    
					else if( ThisEvent.EventType == ES_RESEND_PACKET ){

							#ifdef TESTING_CONTROLLER_TRANSMIT
									printf("\r!!!!!!!!!!CNT: Resending Last Packet\n!!!!!!!!!");
							#endif

							BytesSent=0; //reset bytes sent (This is also the index into the next byte to transmit
							// set packet to send to last packet
							memcpy(TransmitPacket,LastPacketSent, 41*sizeof(uint8_t));
							PacketSize= FRAME_DATA_OVERHEAD-1+(TransmitPacket[1]<<8)+TransmitPacket[2];
							BeginTransmit();
							CurrentState=Transmitting;
					}
					
					else{
                            #ifdef TESTING_CONTROLLER_TRANSMIT
									printf("\r!!!!!!!!!!CNT: Event Not Handeled - Waiting to Start\n!!!!!!!!!");
							#endif	
					}
					
				break; //Break Case waiting to start
					
				case Transmitting:
					// we get a transmission complete event
					if ( ThisEvent.EventType == ES_TRANSMIT_COMPLETE ){
							#ifdef TESTING_CONTROLLER_TRANSMIT
									printf("\rCNT: Received Transmit Complete event \n");
									printf("\rCNT: Transmited %d Bytes \n", PacketSize);
									for(int i=0; i<PacketSize; i=i+1){
										printf("\rCNT: 0x%4x\n",TransmitPacket[i]);
									}
									printf("\rCNT: Moved state to waiting to start \n");
							#endif
							
							//Reset state to waiting to start		
							CurrentState = WaitingToStart;

					} // End if transmit complete
                    
                    else{
							#ifdef TESTING_CONTROLLER_TRANSMIT
									printf("\r!!!!!!!!!!CNT: Event Not Handeled - Transmitting\n!!!!!!!!!");
							#endif	
					}
					
				break; //Break Case Transmitting
					
					
				default:
						printf("\rFR: ***************************************************\n");	;	
						printf("\rFR: ERROR: State Not handeled\n");	
						printf("\rFR: ***************************************************\n");	
				break;
			
		} // end switch case for state machine
			
        #ifdef TESTING_CONTROLLER_TRANSMIT
				printf("\rCNT: Exiting Run Car Node Transmit Service\n");
		#endif		
		
return ReturnEvent;

}

/*----------------------------- Private Functions -----------------------------*/

/*===========================================================================
 Function: ConstructTransmitPacket
 Parameters: PacketType-  Type of packet to be created
 Returns: Boolean to indicate success or failure of packet creation
 Description: Creates transmit packet 
 Author: Michal Rittikaidachar 5/10/17, Drew Bell 11/3/2017
 Notes: -Code adapted from and based off of skeleton files by Ed Carryer
				- Refer to ME 218C 2017 Protocol for packet construction
===========================================================================*/
bool ConstructTransmitPacket (uint8_t PacketType){ 
		bool PacketConstructed = false; //reset to false to test if packet was actually created
        char* myVIN = NULL; 
    
	switch (PacketType){
		
			case PAIR_ACK_PACKET :
					#ifdef TESTING_CONTROLLER_TRANSMIT
							printf("\rCNT: Creating_Pair ACK\n");
					#endif

			
					// Update data sizes for packet type
					PacketSize = PACKET_SIZE_OVERHEAD + PAIR_ACK_SIZE;      // total number of bytes sent over UART
					FrameDataSize = FRAME_DATA_OVERHEAD + PAIR_ACK_SIZE;    // 5 bits overhead + pair ack size (21)
					
					// Update Destination address to current paired dog
					DestinationAddressMSB = GetPairedControllerMSB() ;
					DestinationAddressLSB = GetPairedControllerLSB() ;
			
					// ************* FOR DEBUGGING **********
					//Address of CarNode Xbee
					//DestinationAddressMSB=0x21;		
					//DestinationAddressLSB=0x88;	

                    //Address of Controller Xbee
					//DestinationAddressMSB=0x20;		
					//DestinationAddressLSB=0x88;
					// ************* FOR DEBUGGING **********
			
					LengthPacketLSB = FrameDataSize;
			
					// Construct Packet Header
					ConstructTransmitHeader();
			
					TransmitPacket[RF_DATA_START] = PacketType;
					
                    // add VIN number
                    myVIN = getMyVIN();
					for( uint8_t i = 1; i <= VIN_LENGTH; i++){
							TransmitPacket[RF_DATA_START + i] = myVIN[i-1];
					}
                    
                    // add status packets, careful not to add the status header
					TransmitPacket[RF_DATA_START + VIN_LENGTH + 1] = getStatusDataBytes(0);
					TransmitPacket[RF_DATA_START + VIN_LENGTH + 2] = getStatusDataBytes(1);
					TransmitPacket[RF_DATA_START + VIN_LENGTH + 3] = getStatusDataBytes(2);
					
					// Calculate and set Checksum
					TransmitCheckSum=CalculateCheckSum();
					TransmitPacket[FRAME_DATA_START+FrameDataSize]=TransmitCheckSum;
					
					PacketConstructed=true;	
					
					#ifdef TESTING_CONTROLLER_TRANSMIT
							printf("\rCNT: Created Pair_ACK Packet\n\r");
					#endif
	
			break; // Break Pair Request Packet
			
		
			case STATUS_PACKET:
					#ifdef TESTING_CONTROLLER_TRANSMIT
							printf("\rCNT: Creating Status Packet\n");
					#endif
							

					// Update data sizes for packet type
					PacketSize = PACKET_SIZE_OVERHEAD + STATUS_SIZE;
					FrameDataSize = FRAME_DATA_OVERHEAD + STATUS_SIZE;
					
					// Update Destination address to current paired dog
					DestinationAddressMSB=GetPairedControllerMSB();
					DestinationAddressLSB=GetPairedControllerLSB();
		
			
					LengthPacketLSB = FrameDataSize;
			
					// Construct Packet Header
					ConstructTransmitHeader();

            
					// Create transmit packet
					TransmitPacket[RF_DATA_START] = PacketType;
					TransmitPacket[RF_DATA_START+1] = getStatusDataBytes(0); 		// DIGITAL fields 
					TransmitPacket[RF_DATA_START+2] = getStatusDataBytes(1);		// Vehicle Speed MSB
					TransmitPacket[RF_DATA_START+3] = getStatusDataBytes(2);		// Vehicle Speed LSB
							
					// Calculate and set Checksum
					TransmitCheckSum = CalculateCheckSum();
					TransmitPacket[FRAME_DATA_START+FrameDataSize] = TransmitCheckSum;
					
					PacketConstructed = true;	
					
					#ifdef TESTING_CAR_NODE_TRANSMIT
							printf("\rCNT: Created control packet\n\r");
					#endif
	
			break; // Control packet
			
			
			default:
						printf("\rCNT: *****************************\n");	
						printf("\rCNT: UNKNOWN PACKET TYPE\n");	
						printf("\rCNT: *****************************\n");
			break;

	} //end switch case packet type
	
	packetNumber=packetNumber+1;
    
	return PacketConstructed;
} // end ConstrucTransmitPacket function


/*===========================================================================
 Function: ConstructTransmitHeader
 Parameters: None
 Returns: Boolean to indicate success or failure of header Creation
 Description: Creates transmit packet header
 Author: Michal Rittikaidachar 5/10/17
 Notes: -Code adapted from and based off of skeleton files by Ed Carryer
				- Refer to ME 218C Protocol for packet construction
				- Before calling must updtae destination address and framedatasize
===========================================================================*/

static void ConstructTransmitHeader (void){ 

		TransmitPacket[0] = START_DELIMTER;
		TransmitPacket[1] = LENGTH_PACKET_MSB;				// packet sizes stay in 1 byte range so MSB byte unused
		TransmitPacket[2] = LengthPacketLSB; 				// packet sizes stay in 1 byte range so LSB size is actual size of frame
		TransmitPacket[3] = API_ID;			
		FrameID=FrameID+1;
		if (FrameID==0){
				FrameID=1;
		}
		TransmitPacket[4] = FrameID;									// Currently not tracking frames may add in future
		TransmitPacket[5] = DestinationAddressMSB; 	
		TransmitPacket[6] = DestinationAddressLSB; 	
		TransmitPacket[7] = OPTIONS;
	
}

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
	
	//itrerate through frame data and calculate sum 
	for ( Index = FRAME_DATA_START; Index < FRAME_DATA_START + FrameDataSize ; Index=Index+1 ) {
		// Add current TxPacket byte to rolling sum
		RollingSum = RollingSum+ TransmitPacket[Index];
		#ifdef TESTING_CAR_NODE_TRANSMIT
			//printf("\rCNT: Transmit Rolling Sum: 0x%x\n", RollingSum);
		#endif
	} 
	
	//subtract rolling sum from 0xff to get checksum
	CheckSum=0xff-RollingSum;
	#ifdef TESTING_CAR_NODE_TRANSMIT
		//printf("\rCNT: Transmit Check Sum: 0x%x\n", CheckSum);
	#endif
	return CheckSum;
	
} // End CalculateCheckSum Function


/*===========================================================================
 Function: BeginTransmit
 Parameters: None
 Returns: None
 Description: Function to initiate transmit process
 Author: Michal Rittikaidachar 5/10/17
 Notes: -Will enable interrupts for UART1 TX 
===========================================================================*/

static bool BeginTransmit (void){ 

	#ifdef TESTING_CONTROLLER_TRANSMIT
		printf("\rCNT: Beginning Transmit in CarNode Transmit Service\n");
	#endif
	
	// If TXFE is set there is room to transfer a byte as the fifo as diabled (See Pages 911 and 912 of DataSheet)
	if ( HWREG(UART1_BASE + UART_O_FR) & UART_FR_TXFE ) {
				
			// Write new byte to single byte fifo
			HWREG(UART1_BASE + UART_O_DR) = (HWREG(UART1_BASE + UART_O_DR) & 0xffffff00) + TransmitPacket[BytesSent];

			// Increment index	
			BytesSent=BytesSent+1;
		
		#ifdef TESTING_CONTROLLER_TRANSMIT
				printf("\rCNT: Wrote First Byte\n");
		#endif
			return true;
	} // end if fifo is empty
	
	//Else FIFO not empty so we cant write data
	else{
			#ifdef TESTING_CONTROLLER_TRANSMIT
						printf("\rCNT: FIFO Full\n");
			#endif
			return false;
	} // end if fifo not empty

} // end begin transmit function

/*===========================================================================
 Function: TransmitISR
 Parameters: None
 Returns: None
 Description: ISR for UART TX
 Author: Michal Rittikaidachar 5/10/17
 Notes: 
===========================================================================*/
void TransmitISR ( void ) {

		// clear the TX intrerrupt:  Set TXIC in UARTICR to clear tx interrupt (Page 933 & 934)
			HWREG(UART1_BASE + UART_O_ICR) |= UART_ICR_TXIC;
			
			// If we've sent all bytes of the packet
			if (BytesSent == PacketSize) {

					// Post ES_EV_TRANSMIT_COMPLETE to this service
					ES_Event ThisEvent;
					ThisEvent.EventType = ES_TRANSMIT_COMPLETE;
					PostCarNodeTransmitService( ThisEvent );
					// save last packet in case we have to resend
					memcpy(LastPacketSent, TransmitPacket, 41*sizeof(uint8_t));
			} // end if we sent all the bytes
			
			//else we still have bytes to send
			else {
				
					// Write new data to register (UARTDR: UART data register) 
					HWREG(UART1_BASE + UART_O_DR) = (HWREG(UART1_BASE + UART_O_DR) & 0xffffff00) + TransmitPacket[BytesSent];
					
					// Increment number iof bytes sent
					BytesSent=BytesSent+1;
			} // end else we have more bytes to send

	
} //end TransmitISR

/*===========================================================================
 Function: getStatusDataBytes
 Parameters: uint8_t byte number (0 to 2)
 Returns: uint8_t byte
 Description: function to build status data bytes. Does not return the header value.
 Author: Michal Rittikaidachar 5/10/17
 Notes: 
===========================================================================*/
uint8_t getStatusDataBytes( uint8_t byteNumber ) {
		uint8_t byte2return = 0;
    
    vehicleSpeedInBits = physical2CAN(getVehicleSpeed(), GM_SPEED_FACTOR);
		
    // switch on byteNumber
    switch(byteNumber){
      
        // case 0: return digital byte - Defrost in Bit 2 and Transmission State in Bits 0:1
        case 0: 
            byte2return = (defrostState << DEFROST_S) & DEFROST_M ;
            byte2return = byte2return | ((getTransmissionState() << TRANS_STATE_S) & TRANS_STATE_XBEE_M);
            //printf("\n\rBTR: %i\n\r", byte2return);
        break;
        
        // case 1: return MSB of vehicle speed
        case 1: 
            byte2return = ( (vehicleSpeedInBits>>8) & VEHICLE_SPEED_M); 
        break;
        
        // case 2: return LSB of vehicle speed
		case 2:
            byte2return = ( vehicleSpeedInBits & VEHICLE_SPEED_M);
        break;
            
        // otherwise throw and errror and sent all 1's
        default:
            printf("ERROR: Bad Status Byte Number \n\r");
            byte2return = 0xFF; 
      }
    return byte2return;
} //end TransmitISR




/*------------------------------- Footnotes -------------------------------*/
/*------------------------------ End of file ------------------------------*/
