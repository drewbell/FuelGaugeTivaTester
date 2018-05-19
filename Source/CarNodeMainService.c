

 /*==========================================================================
 Name: CarNodeMainService.c
 Author: Michal Rittikaidachar
 Version:1.0
 Date Created: 10/25/17
 Description: Main Service to control the controller
 Notes: 
 
  						PIN List
 PORT			PIN#	 USE
 -------------- ---	 --------
 
  When			Who	 What/Why
 -------------- ---	 --------
 10/25/17				MR			Started Skeleton
 ==========================================================================*/

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
#include "CarNodeMainService.h"
#include "CarNodeTransmitService.h"
#include "CarNodeReceiveService.h"
#include "ShiftRegisterWrite.h"
#include "CanService.h"
#include "CAN_Conversion.h"



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
#define CONNECTION_TIMEOUT TWO_SEC
#define CONTROL_UPDATE_TIMEOUT FIFTH_SECOND

// Packet Types - See Constants.h

#define MAX_PAIR_ATTEMPTS 3

// API IDs
#define TRANSMIT_STATUS_API 0X89
#define RECEIVE_PACKET_API 0X81


//COMMUNICATION DEFINES
#define API_ID_BYTE 3
#define SOURCE_ADDRESS_MSB_BYTE 4
#define SOURCE_ADDRESS_LSB_BYTE 5
#define RECEIVE_PACKET_TYPE_BYTE 8
#define ACK_STATUS_BYTE 5
#define VIN_BYTE_START 9
#define DIGITAL_CTL_BYTE 9
#define STEERING_MSB 10
#define STEERING_LSB 11
#define LONG_ACCEL_0 12
#define LONG_ACCEL_1 13


/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this service.They should be functions
	relevant to the behavior of this service */
	 
bool InitializeUART (void);
void UART_ISR(void);
void TryToPair(void);
void SendStatusPacket(void);
bool checkVIN(void);
void saveControlData(uint8_t * ReceivedPacket);
void resetControlVariables(void);
float clampAccelerationCustom(float requested, float upper, float lower);
float speed_control(float requested_accel);

/*---------------------------- Module Variables ---------------------------*/
// with the introduction of Gen2, we need a module level Priority variable
static uint8_t MyPriority;

// add a deferral queue for up to 3 pending deferrals +1 to allow for overhead
static ES_Event DeferralQueue[DEFFERRAL_QUE_SIZE];
static CarNodeState_t CurrentState;
static bool ControlUpdateTimerActive = false; // 200ms timer for communication delay flag
static bool ConnectionTimerActive = false; // ms timer for communication connection flag
//static bool CarNodeResponse;        //***unused***
static uint8_t ReceivedPacketType;
static uint8_t *ReceivedPacket;
static uint8_t ApiId;
static uint8_t ReceivedPacketSize;
static uint8_t PairedControllerAddressMSB=0x00;
static uint8_t PairedControllerAddressLSB=0x00;
static uint8_t ReceivedControllerAddressMSB;
static uint8_t ReceivedControllerAddressLSB;
static uint8_t PairAttemptCount=0;
static char VinNumber[VIN_LENGTH] = "123456789abcdefgh";					// char array to hold 17 character VIN
static char RequestedVin[VIN_LENGTH];

// Control Data Variables
static float steeringAngle = 0;
static float acceleration = 0;
static bool wiperState = false;		 // start with wipers off
static bool hornState = false;		  // start with horn off
static bool dirSelection = false;		// start with direction forward
static bool safetyInterlock = false;	// start with interlock disengaged

// Status Data Variables
static volatile float vehicleSpeed = 0;          // Conversion is E = N * 0.03125 km/hr (always positive regardless of direction of travel)
static volatile uint8_t transmissionState = 0;  //enum: 0x00 PARK, 0x01 FWD, 0x02 REV
static volatile uint8_t defrostState = 0;       // 0x00 off, 0x01 on

// Init success var
static bool carNodeInitialized = false;

/*------------------------------ Module Code ------------------------------*/
/*===========================================================================
 Function: initControllerMainService
 Parameters: uint8_t : the priorty of this service
 Returns: bool, false if error in initialization, true otherwise
 Description:  Saves away the priority, and does any other required 
							 initialization for this service
 Author: Michal Rittikaidachar 10/25/17
 Notes: Code adapted from and based off of skeleton files by Ed Carryer
===========================================================================*/
bool InitCarNodeMainService ( uint8_t Priority ){
		#ifdef TESTING_CONTROLLER_MAIN
				printf("\rCNM: Initializing Controller Main Service Hardware\n");
		#endif
	
		ES_Event ThisEvent;
		MyPriority = Priority;
 
		// Initialize Defferal Que
		ES_InitDeferralQueueWith(DeferralQueue, ARRAY_SIZE(DeferralQueue)); 
		
		// Initialize UART1 for xbee communication
		InitializeUART ();
	
		// TODO Initiallize UI Service
	
		// Set Initial State
		CurrentState =  InitCarNode;	
	
		// Post the initial transition event
		ThisEvent.EventType = ES_INIT;
    
        // Init shift register
        updateCarNodeLEDs();
		
		if (ES_PostToService( MyPriority, ThisEvent) == true){  
			carNodeInitialized = true;      // indicates successful init for LED display	
            return true;      
		}
		
		else{
			return false;
		}
}

/*
 ============================================================
 Function: PostCarNodeMainService
 Parameters: EF_Event ThisEvent ,the event to post to the queue
 Returns: bool false if the Enqueue operation failed, true otherwise
 Description: Posts an event to this state machine's queue
 Author: Michal Rittikaidachar 10/25/17
 Notes: Code adapted from and based off of skeleton files by Ed Carryer
 ============================================================
*/

bool PostCarNodeMainService( ES_Event ThisEvent )
{
  return ES_PostToService( MyPriority, ThisEvent);
}

/*===========================================================================
 Function: RunControllerMainService
 Parameters: ES_Event : the event to process
 Returns: ES_Event, ES_NO_EVENT if no error ES_ERROR otherwise
 Description: Runs State Machine for Controller Main Service.  
 Author: Michal Rittikaidachar 10/25/17
 Notes: Code adapted from and based off of skeleton files by Ed Carryer
===========================================================================*/

ES_Event RunCarNodeMainService( ES_Event ThisEvent )
{

    ES_Event ReturnEvent;
    ReturnEvent.EventType = ES_NO_EVENT; // assume no errors
    
    #ifdef TESTING_CONTROLLER_MAIN
            printf("\r\nCNM: Entering Run CarNode Main Service\n");
    #endif
	
    switch (CurrentState){
			
        case InitCarNode :
            if ( ThisEvent.EventType == ES_INIT ){
                #ifdef TESTING_CONTROLLER_MAIN
                printf("\rCNM: Initializing CarNode MainService State Machine\n");
                #endif
            
                //TODO Do any initialization here
            
                // switch states to selectingVehicle
                CurrentState =  WaitingToPair;	
            
                #ifdef TESTING_CONTROLLER_MAIN
                   printf("\rCNM: Switched ControllerMainService to Waiting to Pair\n\r");
                #endif

            }
        break;
								
						
        case  WaitingToPair :
            // If we get a pair request and the communication delay timer is inactive
            // note check timer status as we will not try to repair if we currenlty are (maybe change to restart timer instead)
            if ( ThisEvent.EventType == ES_RECEIVED_PACKET ){
                        
                #ifdef TESTING_CONTROLLER_MAIN
                printf("\rCNM: Received New Packet event while waiting to pair\n");
                #endif
         
                ReceivedPacket = GetReceivedPacket();     // get received packet
                ReceivedPacketSize = ThisEvent.EventParam;       // get package size
      
                #ifdef TESTING_CONTROLLER_MAIN
                    printf("\rCNM: Received %d Bytes \n", ReceivedPacketSize);
                    for(int i=0; i<ReceivedPacketSize; i=i+1){
                        printf("\rCNM: 0x%4x\n",ReceivedPacket[i]);
                    }
                #endif
                  
                //determine the API ID
                ApiId=ReceivedPacket[API_ID_BYTE];
            
                // get address of sender 
                ReceivedControllerAddressMSB=ReceivedPacket[SOURCE_ADDRESS_MSB_BYTE];
                ReceivedControllerAddressLSB=ReceivedPacket[SOURCE_ADDRESS_LSB_BYTE];
                            
                // if we get a Receive packet API lets check to see if its a pair request for us
                if (ApiId==RECEIVE_PACKET_API){
                    ReceivedPacketType=ReceivedPacket[RECEIVE_PACKET_TYPE_BYTE];

                    // if we get a pair request packet
                    if ((ReceivedPacketType == PAIR_REQUEST_PACKET) & checkVIN() ){
                        #ifdef TESTING_CONTROLLER_MAIN
                        printf("\rCNM: Received Pair Request\n");
                        #endif
                         
                        // save away paired radio address 
                        PairedControllerAddressMSB=ReceivedControllerAddressMSB;
                        PairedControllerAddressLSB=ReceivedControllerAddressLSB;
                               
                        //Send pair ack
                        ES_Event ThisEvent;
                        ThisEvent.EventType = ES_BEGIN_TRANSMIT;
                        ThisEvent.EventParam = PAIR_ACK_PACKET;
                        PostCarNodeTransmitService(ThisEvent);
                        CurrentState = Paired;
                                        
                        //Start communication Connection timer 
                        ES_Timer_InitTimer( CONNECTION_TIMER, CONNECTION_TIMEOUT);
                        ConnectionTimerActive = true;												
                        #ifdef TESTING_CONTROLLER_MAIN
                            printf("\rCNM: moved State to Paired\n");
                        #endif
                    } // end if our vins match and its a pair request
                        
                    //Not a pair request 
                    else{
                       #ifdef TESTING_CONTROLLER_MAIN
                            printf("\rCNM: Not a pair request Packet\n");
                       #endif
                    }
                } // end if its a recieved packet API
            } // end if we get communication packet 
         
            else if(ThisEvent.EventType == ES_NO_EVENT){
                #ifdef TESTING_CONTROLLER_MAIN
                printf("\rCNM: ***************************************************\n");	;	
                printf("\rCNM: ERROR: ES_NO_EVENT WHILE WAITING TO PAIR!\n");	
                printf("\rCNM: ***************************************************\n");
                printf("\rCNM: EVENT Type: %d \n", ThisEvent.EventType);
                printf("\rCNM: EVENT Param: %d \n", ThisEvent.EventParam);
                #endif
            }
            else if(ThisEvent.EventType == ES_ERROR){
                #ifdef TESTING_CONTROLLER_MAIN
                printf("\rCNM: ***************************************************\n");	;	
                printf("\rCNM: ERROR: ES_ERROR WHILE WAITING TO PAIR!\n");	
                printf("\rCNM: ***************************************************\n");
                printf("\rCNM: EVENT Type: %d \n", ThisEvent.EventType);
                printf("\rCNM: EVENT Param: %d \n", ThisEvent.EventParam);
                #endif
            }
            else if(ThisEvent.EventType == ES_INIT){
            #ifdef TESTING_CONTROLLER_MAIN
                printf("\rCNM: ***************************************************\n");	;	
                printf("\rCNM: ERROR: ES_INIT WHILE WAITING TO PAIR!\n");	
                printf("\rCNM: ***************************************************\n");
                printf("\rCNM: EVENT Type: %d \n", ThisEvent.EventType);
                printf("\rCNM: EVENT Param: %d \n", ThisEvent.EventParam);
            #endif
            }					
            // Event Not Handeled
            else{
                #ifdef TESTING_CONTROLLER_MAIN
                    printf("\rCNM: ***************************************************\n");	;	
                    printf("\rCNM: ERROR: EVENT NOT  HANDELED WHILE WAITING TO PAIR!\n");	
                    printf("\rCNM: ***************************************************\n");
                    printf("\rCNM: EVNET Type: %d \n", ThisEvent.EventType);
                    printf("\rCNM: EVNET Param: %d \n", ThisEvent.EventParam);
                #endif
            }
      
        break; //Break Case waiting to pair
							

        case Paired :		

            // CONNECTION TIMEOUT
            if ( ThisEvent.EventType == ES_TIMEOUT && ThisEvent.EventParam == CONNECTION_TIMER){
                #ifdef TESTING_CONTROLLER_MAIN
                    printf("\rCNM: Received Connection Timeout while paired\n");
                #endif
                    
                // Set timer active flag to false so that we can receive pair events again
                // Also, clear control variables
                ConnectionTimerActive = false;
                ES_Timer_StopTimer( CONNECTION_TIMER);
                ES_Timer_StopTimer( CONTROL_UPDATE_TIMER);
                resetControlVariables();
                
                // move state to waiting to pair
                CurrentState = WaitingToPair;	
                #ifdef TESTING_CONTROLLER_MAIN
                    printf("\rCNM: Moved state to waiting to pair\n");
                #endif
            } // End if Connection timeout event
                        

                // RECEIVED PACKET EVENT
            else if (ThisEvent.EventType == ES_RECEIVED_PACKET ){
                #ifdef TESTING_CONTROLLER_MAIN
                    printf("\rCNM: Received New Packet event while paired \n");
                #endif
                    
                // get received packet
                ReceivedPacket = GetReceivedPacket();
                    
                // get package size
                ReceivedPacketSize=ThisEvent.EventParam;
                
                #ifdef TESTING_CONTROLLER_MAIN
                    printf("\rCNM: Received %d Bytes \n", ReceivedPacketSize);
                    for(int i=0; i<ReceivedPacketSize; i=i+1){
                        printf("\rCNM: 0x%4x\n",ReceivedPacket[i]);
                    }
                #endif
                                
                //determine the API ID
                ApiId=ReceivedPacket[API_ID_BYTE];
                        
                // get address of sender 
                ReceivedControllerAddressMSB=ReceivedPacket[SOURCE_ADDRESS_MSB_BYTE];
                ReceivedControllerAddressLSB=ReceivedPacket[SOURCE_ADDRESS_LSB_BYTE];
                                
                // if we get a Receive packet API lets check the address and only respond to our paired controller
                if (ApiId==RECEIVE_PACKET_API){
                    ReceivedPacketType=ReceivedPacket[RECEIVE_PACKET_TYPE_BYTE];
                    
                    // if its our controller address lets see what they want
                    if(ReceivedControllerAddressMSB==PairedControllerAddressMSB && 
                        ReceivedControllerAddressLSB==PairedControllerAddressLSB ){
                                
                        // if we get a controller control packet
                        if (ReceivedPacketType == CONTROL_PACKET){
                            #ifdef TESTING_CONTROLLER_MAIN
                                printf("\rCNM: Received Control Packet\n");
                            #endif
                                    
                            saveControlData(ReceivedPacket);    // Save control data away  
                            SendStatusPacket();     // send status packet to the controller
                                                    
                            //Start communication Connection timer for 1 sec 
                            ES_Timer_InitTimer( CONNECTION_TIMER, CONNECTION_TIMEOUT);
                            ConnectionTimerActive = true;
                                        
                        }
                                
                        // if we get a controller unpair request
                        else if (ReceivedPacketType == UNPAIR_REQUEST_PACKET){
                            #ifdef TESTING_CONTROLLER_MAIN
                                printf("\rCNM: Received unpair request Packet\n");
                            #endif

                            // turn off timers and reset states
                            ES_Timer_StopTimer( CONNECTION_TIMER);
                            ConnectionTimerActive = false;
                            ControlUpdateTimerActive = false;
                            CurrentState = WaitingToPair;
                            resetControlVariables();
                            
                            #ifdef TESTING_CONTROLLER_MAIN
                                printf("\rCNM: Moved States to Waiting to Pair\n");
                            #endif
                            
                        }
                                    
                        // else if we get another paired even from our controller while we are still paired	
                        else if (ReceivedPacketType == PAIR_REQUEST_PACKET){
                            #ifdef TESTING_CONTROLLER_MAIN
                                printf("\rCNM: Received Pair Request while paired\n");
                            #endif
                
                            //Send pair ack
                            ES_Event ThisEvent;
                            ThisEvent.EventType = ES_BEGIN_TRANSMIT;
                            ThisEvent.EventParam = PAIR_ACK_PACKET;
                            PostCarNodeTransmitService(ThisEvent);

                            //Start communication Connection timer 
                            ES_Timer_InitTimer( CONNECTION_TIMER, CONNECTION_TIMEOUT);
                            ConnectionTimerActive = true;												
                            #ifdef TESTING_CONTROLLER_MAIN
                                        printf("\rCNM: moved State to Paired\n");
                            #endif
                        } // end if we get a pair request while already paired 
                                    
                    } // end if its our dog trying to talk to us
                        
                    //else its not our dog so do nothing
                    else{
                        #ifdef TESTING_CONTROLLER_MAIN
                            printf("\rCNM: Received packet from Controller im not paired with");
                            printf("\rCNM: ADDRESS MSB: 0x%4x ",ReceivedControllerAddressMSB);
                            printf("\rCNM: ADDRESS LSB: 0x%4x ",ReceivedControllerAddressLSB);
                        #endif
                    } 
                        
                    //TO DO add any other responses not sure if we need to though
                    
                } // end if we get a RECEIVE_PACKET API		
            } // end if we get received packet packet from receive service
               
            // catch other Events we aren't expecting
            else if(ThisEvent.EventType == ES_NO_EVENT){
                #ifdef TESTING_CONTROLLER_MAIN
                    printf("\rCNM: ***************************************************\n");	;	
                    printf("\rCNM: ERROR: ES_NO_EVENT WHILE PAIRED!\n");	
                    printf("\rCNM: ***************************************************\n");
                    printf("\rCNM: EVENT Type: %d \n", ThisEvent.EventType);
                    printf("\rCNM: EVENT Param: %d \n", ThisEvent.EventParam);
                #endif
            }
            else if(ThisEvent.EventType == ES_ERROR){
                #ifdef TESTING_CONTROLLER_MAIN
                    printf("\rCNM: ***************************************************\n");	;	
                    printf("\rCNM: ERROR: ES_ERROR WHILE PAIRED!\n");	
                    printf("\rCNM: ***************************************************\n");
                    printf("\rCNM: EVENT Type: %d \n", ThisEvent.EventType);
                    printf("\rCNM: EVENT Param: %d \n", ThisEvent.EventParam);
                #endif
            }
            else if(ThisEvent.EventType == ES_INIT){
                #ifdef TESTING_CONTROLLER_MAIN
                    printf("\rCNM: ***************************************************\n");	;	
                    printf("\rCNM: ERROR: ES_INIT WHILE PAIRED!\n");	
                    printf("\rCNM: ***************************************************\n");
                    printf("\rCNM: EVENT Type: %d \n", ThisEvent.EventType);
                    printf("\rCNM: EVENT Param: %d \n", ThisEvent.EventParam);
                #endif
            }
            else{
                #ifdef TESTING_CONTROLLER_MAIN
                    printf("\rCNM: ***************************************************\n");	;	
                    printf("\rCNM: ERROR: EVENT NOT  HANDELED WHILE PAIRED!\n");	
                    printf("\rCNM: ***************************************************\n");
                    printf("\rCNM: EVNET Type: %d \n", ThisEvent.EventType);
                    printf("\rCNM: EVNET Param: %d \n", ThisEvent.EventParam);
                #endif
            }                 
                
        break; //Break Case paired
                
                
        default:
            #ifdef TESTING_CONTROLLER_MAIN
                printf("\rFR: ***************************************************\n");	;	
                printf("\rFR: ERROR: State Not handeled\n");	
                printf("\rFR: ***************************************************\n");	
            #endif
        break;	
        
    } // end switch case for state machine

  	#ifdef TESTING_CONTROLLER_MAIN
        printf("\rCNM: Exiting Run CarNode Main Service\n");
    #endif		
		
    return ReturnEvent;

}


/*===========================================================================
 Function: GetPairedControllerMSB
 Parameters: None
 Returns: uint8_t 
 Description: getter function to retreive paried Controller destination address
 Author: Michal Rittikaidachar 5/10/17
 Notes: 

===========================================================================*/

uint8_t GetPairedControllerMSB ( void ) {
	return PairedControllerAddressMSB;
}


/*===========================================================================
 Function: GetPairedControllerLSB
 Parameters: None
 Returns: uint8_t 
 Description: getter function to retreive paired controller destination address
 Author: Michal Rittikaidachar 5/10/17
 Notes: 

===========================================================================*/

uint8_t GetPairedControllerLSB ( void ) {
	return PairedControllerAddressLSB;
}


/*----------------------------- Private Functions -----------------------------*/




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
		
					
		// Initialize UART TX Interrupt: Pages 924 and 925
		HWREG(UART1_BASE + UART_O_IM) |= UART_IM_TXIM;
			
		// Initialize UART RX Interrupt: Pages 924 and 925
		HWREG(UART1_BASE + UART_O_IM) |= UART_IM_RXIM;
			
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

	// If TXMIS is set we have a transmit interrupt and can now send another byte (Page  930 and 931)
	if (HWREG(UART1_BASE + UART_O_MIS) & UART_MIS_TXMIS) {
			TransmitISR();
	} // end if transmit interrupt
	
	// else we got a receive interrupt 
	else {
			ReceiveISR();
	} //end if receive interrupt
	
}


/*===========================================================================
 Function: TryToPair
 Parameters: None
 Returns: None
 Description: Routine for trying to pair to CarNode
 Author: Michal Rittikaidachar 10/25/17
 Notes: -Code adapted from and based off of skeleton files by Ed Carryer

===========================================================================*/
void TryToPair( void ) {
	
	//Broadcast that we are trying to pair
	ES_Event ThisEvent;
	ThisEvent.EventType = ES_BEGIN_TRANSMIT;
	ThisEvent.EventParam = PAIR_REQUEST_PACKET;
	PostCarNodeTransmitService(ThisEvent);
						
	//Start communication connection timer 
	ES_Timer_InitTimer( CONNECTION_TIMER, CONNECTION_TIMEOUT);
	ConnectionTimerActive = true;
						
	// Set communication delay timer
	//ES_Timer_InitTimer( CONTROL_UPDATE_TIMER, CONTROL_UPDATE_TIMEOUT);						
	//ControlUpdateTimerActive = true;		
						
	// Set Acknowledge flag to false
	//CarNodeResponse=false;            //***unused***
								
	// increment pair attempt counter
	PairAttemptCount=PairAttemptCount+1;
	
}

/*===========================================================================
 Function: SendControlPacket
 Parameters: None
 Returns: None
 Description: Routine for Sending a control packet to the transmit service
 Author: Michal Rittikaidachar 10/25/17
 Notes: -Code adapted from and based off of skeleton files by Ed Carryer

===========================================================================*/
void SendStatusPacket( void ) {
	//Send status packet
	ES_Event ThisEvent;
	ThisEvent.EventType = ES_BEGIN_TRANSMIT;
	ThisEvent.EventParam = STATUS_PACKET;
	PostCarNodeTransmitService(ThisEvent);
										
	//Start communication Connection timer for 1 sec to determine if we dont have connection
	ES_Timer_InitTimer( CONNECTION_TIMER, CONNECTION_TIMEOUT);
	ConnectionTimerActive = true;
										
	
	
}

/*===========================================================================
 Function: getMyVIN
 Parameters: None
 Returns: a memory address (pointer) to where the VIN number lives
 Description: returns the location of the VIN number
 Author: Drew Bell 11/03/17
 Notes: 
===========================================================================*/
char * getMyVIN( void ) {
	
	return VinNumber;
		
}


/*===========================================================================
 Function: checkVIN
 Parameters: None
 Returns: a boolean 
 Description: returns true if the VIN reqested matches the car's
 Author: Drew Bell 11/03/17
 Notes: 

  When	Who	 What/Why
 ------	---	 --------
 11/4/17  MR		Used strncmp rather than strcmp and compare the size 
					of the compared arrays explicitly
===========================================================================*/
bool checkVIN( void ) {
	bool match = false;
	int comparison;
	// look through and store requested VIN
	for(uint8_t i = 0; i < VIN_LENGTH; i++){
		RequestedVin[i] = ReceivedPacket[VIN_BYTE_START + i];
	}
	
		
	if (sizeof(VinNumber)==sizeof(RequestedVin)){
		// see if requested VIN matches the cars VIN
		comparison = strncmp(RequestedVin, VinNumber, sizeof(VinNumber));
	}
	else{
		comparison = 0;
	}
			
			
	// if there is a match, return true. Otherewise, false
	if (comparison == 0){
		printf("\n\rcheckVIN: VIN Match\n\r");
		match = true;
	}
	else{
		printf("\n\rcheckVIN: VIN Not Ours\n\r");
		match = false;
	}
	
	// return the result of the match
	return match;

} //End CheckVIN Function


/*===========================================================================
 Function: saveControlData
 Parameters: uint8_t pointer to received data 
 Returns: Nothing 
 Description: saves away control data received from the Controller
 Author: Drew Bell 11/04/17
 Notes: 

  When	Who	 What/Why
 ------	---	 --------
 11/4/17  DB		Moving these lines out of the main state machhine 

===========================================================================*/
void saveControlData(uint8_t * ReceivedPacket){
        
        // first, save the digital control byte items
        uint8_t xbeeDigitalControlByte = ReceivedPacket[DIGITAL_CTL_BYTE];
        safetyInterlock = (xbeeDigitalControlByte & SAFETY_INTERLOCK_M);
        
        // if interlock is on, set other variables, else clear
        if(safetyInterlock){

            wiperState = (xbeeDigitalControlByte & WIPER_M);
            hornState = (xbeeDigitalControlByte & HORN_M);
            dirSelection = (xbeeDigitalControlByte & DIR_SELECTION_M);
      
            // next save the steering angle
            steeringAngle = CAN2Physical(ReceivedPacket[STEERING_MSB]<<ONE_BYTE_S | ReceivedPacket[STEERING_LSB], GM_STEER_FACTOR);
                    
            // finally save the acceleration command and apply speed control
            float reqAcceleration = CAN2Physical(ReceivedPacket[LONG_ACCEL_0]<<ONE_BYTE_S | ReceivedPacket[LONG_ACCEL_1], GM_ACCEL_FACTOR);
            acceleration = speed_control(reqAcceleration);
        }
        else{   // if interlock is not on, clear the variable
            resetControlVariables();
        }
}

/*===========================================================================
 Function: speed_control
 Parameters: float
 Returns: float  
 Description: currently implements a bang bang controller for limiting speed 
 to a desired maximum value. 
 Author: Drew Bell 03/24/18
 Notes: 
  When	Who	 What/Why
 ------	---	 --------
 03/24/18 db   Created Function
===========================================================================*/
float speed_control( float requested_accel ) {
    
    float upper_accel_limit = (float) POSITIVE_ACCEL_MAX;     // default to full torque range
    float lower_accel_limit = (float) NEGATIVE_ACCEL_MAX;
    
    #ifdef SPEED_LIMIT_ON
        static bool speedLimited = false;
        if(vehicleSpeed >= SPEED_LIM_THRESH_H)   // if the speed is above, always limit speed
        {
            speedLimited = true;
            upper_accel_limit = 0.f;
        }
        // if in middle on the way up, do not limit torque
        else if( vehicleSpeed < SPEED_LIM_THRESH_H && vehicleSpeed >= SPEED_LIM_THRESH_L && !speedLimited)
        {
           // do nothing
        }
        // if in the middle and on the way down, impose limit
        else if( vehicleSpeed < SPEED_LIM_THRESH_H && vehicleSpeed >= SPEED_LIM_THRESH_L && speedLimited)
        {
            upper_accel_limit = 0.f;
        }
        // if below, clear the speedLimited variable but do not limit torque
        else if(vehicleSpeed < SPEED_LIM_THRESH_L) 
        {
            speedLimited = false;
        }
    #endif 

    return clampAccelerationCustom(requested_accel, upper_accel_limit, lower_accel_limit);
}


/*===========================================================================
 Function: GetCANTxDigitalByte
 Parameters: void
 Returns: byte  
 Description: Returns CAN Tx digital data byte to other services
 Author: Gabrielle Vukasin 11/06/17
 Notes: 

  When	Who	 What/Why
 ------	---	 --------
 11/6/17  GV		Created function 
 12/5/17  afb       change name from getDigitalByte() to GetCANTxDigitalByte 

===========================================================================*/
uint8_t GetCANTxDigitalByte( void ) {
    uint8_t CANTxDigitalByte = 0;
    
    CANTxDigitalByte = (safetyInterlock << SAFETY_INTERLOCK_S | ( dirSelection << DIR_SELECTION_S) 
    | (hornState << HORN_S) | (wiperState << WIPER_S));
    
	return CANTxDigitalByte;
}

/*===========================================================================
 Function: GetSteeringAngle
 Parameters: void
 Returns: float steering angle  
 Description: Returns steering angle to other services
 Author: Gabrielle Vukasin 11/06/17
 Notes: 

  When	Who	 What/Why
 ------	---	 --------
 11/6/17  GV		Created function 

===========================================================================*/
float GetSteeringAngle( void ) {
	return steeringAngle;
}

/*===========================================================================
 Function: GetAccel
 Parameters: void
 Returns: float acceleration  
 Description: Returns acceleration to other services
 Author: Gabrielle Vukasin 11/06/17
 Notes: 

  When	Who	 What/Why
 ------	---	 --------
 11/6/17  GV		Created function 

===========================================================================*/
float GetAccel( void ) {
	return acceleration;
}

/*===========================================================================
 Function: updateCarNodeLEDs
 Parameters: void
 Returns: void  
 Description: Prints out the state of several variables in LEDs
 Author: Drew Bell 11/11/17
 Notes: 
    BIT0    AUX2 -- Currently Transmision State LSB
    BIT1    AUX1 -- Currently Transmision State MSB
    BIT2    XBee
    BIT3    CAN
    BIT4    System Init
    BIT5    Interlock
    BIT6    FWD
    BIT7    REV


  When	Who	 What/Why
 ------	---	 --------
 11/11/17 DB		Created function 

===========================================================================*/                
void updateCarNodeLEDs(void){
    uint8_t lightPattern = 0;
    
    lightPattern |= transmissionState;
    
    if(getXBeeConnectionStatus()){
        lightPattern |= 0x01<<2;
    }
    if(getCAN0_pairStatus()){
        lightPattern |= 0x01<<3;
    }
    if(carNodeInitialized){
        lightPattern |= 0x01<<4;
    }
    if(safetyInterlock){
        lightPattern |= 0x01<<5;		 // illuminate the safety Interlock LED
    }
    if(dirSelection == DIR_SEL_FWD && safetyInterlock){
        lightPattern |= 0x01<<6;
    } else if(dirSelection == DIR_SEL_REV){
        lightPattern |= 0x01<<7;
    }
    
    SR_Write(lightPattern);
    
    return;
    
}

/*===========================================================================
saveStatusVarsToCNMain
Parameters: uint8_t * to a 5 byte array
Returns: nothing
Description: save status information received from the car in CarNode Main, making that module
the central repository for variables passed between the CarNode and controller
Author: Drew Bell 11/11/17
Notes: 
    (1) Converts data from transmit bits into physical units for use in Car Node Main
    (2) Current implementation only uses 3 bytes of array for status packet

===========================================================================*/
void saveStatusVarsToCNMain(uint8_t * StatusMsg){
    transmissionState = (StatusMsg[0] >> TRANS_STATE_S) & TRANS_STATE_M;  
    defrostState = (StatusMsg[0] >> DEFROST_S) & DEFROST_M;  
    uint16_t speedBits = (StatusMsg[1] << SPEED_MSB_LEFT_S) | (StatusMsg[2] >> SPEED_LSB_RIGHT_S);      
    vehicleSpeed = CAN2Physical(speedBits, GM_SPEED_FACTOR);  
} 

/*===========================================================================
 Function: getSafetyInterlockState
 Parameters: void
 Returns: bool safetyInterlock  
 Description: Returns safety interlock state 
 Author: Drew Bell 11/30/17
 Notes: 
    True  =  enabled
    False = disabled
===========================================================================*/
bool getSafetyInterlockState( void ){
    return safetyInterlock;
}

/*===========================================================================
 Function: getTransmissionState
 Parameters: void
 Returns: uint8_t transmissionState  
 Description: Returns transmission state to other modules
 Author: Drew Bell 11/11/17
 Notes: 
    Per communication protocol: 
        0x00 = PARK, 0x01 = FWD, 0x02 = REV
===========================================================================*/
uint8_t getTransmissionState( void ) {
	return transmissionState;
}

/*===========================================================================
 Function: setTransmissionState
 Parameters: uint8_t newTransState  
 Returns: void  
 Description: sets transmission state from other modules
 Author: Drew Bell 11/11/17
 Notes: 
    Per communication protocol: 
        0x00 = PARK, 0x01 = FWD, 0x02 = REV
===========================================================================*/
void setTransmissionState( uint8_t newTransState ) {
	transmissionState = newTransState;
}

/*===========================================================================
 Function: getDefrostState
 Parameters: void
 Returns: uint8_t defrostState  
 Description: Returns defrostState to other modules
 Author: Drew Bell 11/11/17
 Notes: 
===========================================================================*/
uint8_t getDefrostState( void ) {
	return defrostState;
}

/*===========================================================================
 Function: setDefrostState
 Parameters: uint8_t newDefrostState
 Returns: void  
 Description: sets defrost state from other modules
 Author: Drew Bell 11/11/17
 Notes: 
    Per communication protocol: 
        0x00 = PARK, 0x01 = FWD, 0x02 = REV
===========================================================================*/
void setDefrostState( uint8_t newDefrostState ) {
	defrostState = newDefrostState;
}


/*===========================================================================
 Function: getVehicleSpeed
 Parameters: void
 Returns: float vehicleSpeed  
 Description: Returns vehicleSpeed to other services
 Author: Drew Bell 11/11/17
 Notes: 
===========================================================================*/
float getVehicleSpeed( void ) {
	return vehicleSpeed;
}


/*===========================================================================
 Function: setVehicleSpeed
 Parameters: float newVehicleSpeed
 Returns: void  
 Description: sets vehicle speed variable from other modules
 Author: Drew Bell 11/11/17
 Notes: 
    Valid speeds are from 0 to 16 km/hr
===========================================================================*/
void setVehicleSpeed( float newVehicleSpeed ) {
	vehicleSpeed = newVehicleSpeed;
}

float clampVehicleSpeed(float newSpeed);

/*===========================================================================
clampVehicleSpeed
Takes float, returns float
Purpose: controls the maximim and minimum allowable speed
Notes: valid input: 0 to 16km/hr
===========================================================================*/
float clampVehicleSpeed(float newSpeed){
    float input = newSpeed;
    if(input > VEHICLE_SPEED_MAX){
        input = VEHICLE_SPEED_MAX;
    }
    else if(input < (float)0.0){
        input = (float)0.0;
    }
    return input;
} 


/*===========================================================================
SETTER FUNCTIONS FOR KEYSTROKE CONTROL
===========================================================================*/



/*===========================================================================
setSafetyInterlock
Takes bool, returns nothing
Purpose: allows external change of safety interlock variable
Notes: 0x00 = System Deactivated, 0x01 = Requests System Activation
===========================================================================*/
void setSafetyInterlock(bool newState){
    safetyInterlock = newState;
}    


/*===========================================================================
setDirSelection
Takes bool, returns nothing
Purpose: allows external change of the direction selection variable 
Notes: 0x00 = FWD, 0x01 = REV
===========================================================================*/
void setDirSelection(bool newState){
    dirSelection = newState;
}    


/*===========================================================================
setHornState
Takes bool, returns nothing
Purpose: allows external change of the horn variable 
Notes: 0x00 = No sound, 0x01 = Horn sound
===========================================================================*/
void setHornState(bool newState){
    hornState = newState;
} 

/*===========================================================================
setWiperState
Takes bool, returns nothing
Purpose: allows external change of the wiper variable 
Notes: 0x00 = deactivated, 0x01 = activate
===========================================================================*/
void setWiperState(bool newState){
    wiperState = newState;
} 

/*===========================================================================
setSteeringAngle
Takes float, returns nothing
Purpose: allows external change of the steering angle
Notes: valid input: 400 as full right, -400 as full left
===========================================================================*/
void setSteeringAngle(float newState){
    steeringAngle = newState;
}    

/*===========================================================================
clampSteering
Takes float, returns float
Purpose: controls the maximim allowable steering angle
Notes: valid input: 400 as full right, -400 as full left
===========================================================================*/
float clampSteering(float newState){
    float input = newState;
    if(input > RIGHT_STEERING_MAX){
        input = RIGHT_STEERING_MAX;
    }
    else if(input < LEFT_STEERING_MAX){
        input = LEFT_STEERING_MAX;
    }
    return input;
} 


/*===========================================================================
setAcceleration
Takes float, returns nothing
Purpose: allows external change of the horn variable 
Notes: 
    (1) valid input: 1000 positive acceleration to -2000 braking
    (2) Positive is propulsion and negative is braking regardless of direction of travel
===========================================================================*/
void setAcceleration(float newState){
    acceleration = newState;
}    

/*===========================================================================
clampAcceleration
Takes float, returns float
Purpose: controls the maximim allowable 
Notes: 
    (1) valid input: 1000 positive acceleration to -2000 braking
===========================================================================*/
float clampAcceleration(float newState){
    float input = newState;
    if(input > POSITIVE_ACCEL_MAX){
        input = POSITIVE_ACCEL_MAX;
    }
    else if(input < NEGATIVE_ACCEL_MAX){
        input = NEGATIVE_ACCEL_MAX;
    }
    return input;
} 

/*===========================================================================
clampAccelerationCustom
Takes float, returns float
Desc: limits speed to the range spread by upper and lower
===========================================================================*/
float clampAccelerationCustom(float requested, float upper, float lower){
    float input = requested;
    if(input > upper){
        input = upper;
    }
    else if(input < lower){
        input = lower;
    }
    return input;
} 


/*===========================================================================
 Function: getXBeeConnectionStatus
 Parameters: void
 Returns: bool XBeeConnectionStatus 
 Description: Returns ConnectionTimerActive to other services
 Author: Drew Bell 11/11/17
 Notes: 
===========================================================================*/
bool getXBeeConnectionStatus( void ) {
	return ConnectionTimerActive;
}

/*===========================================================================
 Function: resetControlVariables
 Parameters: nothing
 Returns: nothing
 Description: clears the 
 Author: Drew Bell 11/11/17
 Notes: 
===========================================================================*/
void resetControlVariables( void ) {
    safetyInterlock = false;
		wiperState = false;
    hornState = false;
    dirSelection = DIR_SEL_FWD;
    steeringAngle = 0.0f;
    acceleration = 0.0f; 
}



/*------------------------------- Footnotes -------------------------------*/
/*------------------------------ End of file ------------------------------*/

