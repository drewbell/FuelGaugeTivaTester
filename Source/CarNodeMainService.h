/*
 ============================================================
 Name:CarNodeMainService.h
 Author: Michal Rittikaidachar
 Version:1
 Date Created: 5/9/17
 Description: Header File for CarNodeMainService.c
 Code adapted from and based off of skeleton files by Ed Carryer
 ============================================================
 */

#ifndef CarNodeMainService_H
#define CarNodeMainService_H

// Event Definitions
#include "ES_Configure.h" /* gets us event definitions */
#include "ES_Types.h"     /* gets bool type for returns */

// typedefs for the states
// State definitions for use with the query function
typedef enum {InitCarNode, WaitingToPair, Paired,} CarNodeState_t ;

// Public Function Prototypes

bool InitCarNodeMainService ( uint8_t Priority );
bool PostCarNodeMainService( ES_Event ThisEvent );
ES_Event RunCarNodeMainService( ES_Event ThisEvent );

void UART_ISR ( void );
uint8_t GetPairedControllerMSB ( void );
uint8_t GetPairedControllerLSB ( void );
uint8_t GetCANTxDigitalByte ( void );
float GetSteeringAngle( void );
float GetAccel( void );
bool getXBeeConnectionStatus( void );

// getters and setter for status data
bool getSafetyInterlockState( void );
void saveStatusVarsToCNMain(uint8_t * StatusMsg);
float getVehicleSpeed( void );
uint8_t getDefrostState( void );
uint8_t getTransmissionState( void );
void setVehicleSpeed( float newVehicleSpeed );
void setDefrostState( uint8_t newDefrostState );
void setTransmissionState( uint8_t newTransState );
float clampVehicleSpeed(float newSpeed);

// Setter and clamp functions for control data
void setSafetyInterlock(bool newState);
void setDirSelection(bool newState);
void setHornState(bool newState);
void setWiperState(bool newState);
void setSteeringAngle(float newState);
void setAcceleration(float newState);
float clampSteering(float newState);
float clampAcceleration(float newState);
void updateCarNodeLEDs(void);

char * getMyVIN( void );
	
uint16_t GetAccelerationMagnitude (void); //TODO
bool GetAccelerationDirection (void); //TODO



#endif

 /* CarNodeMainService_H */

