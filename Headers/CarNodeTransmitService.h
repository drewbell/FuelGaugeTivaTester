/*
 ============================================================
 Name: CarNodeTransmitService.h
 Author: Michal Rittikaidachar
 Version:1
 Date Created: 5/9/17
 Description: Header File for CarNodeTransmitService.c
 Code adapted from and based off of skeleton files by Ed Carryer
 ============================================================
 */

#ifndef CarNodeTransmitService_H
#define CarNodeTransmitService_H

// Event Definitions
#include "ES_Configure.h" /* gets us event definitions */
#include "ES_Types.h"     /* gets bool type for returns */

// typedefs for the states
// State definitions for use with the query function
typedef enum {WaitingToStart, Transmitting, InitTransmit } TransmitState_t ;

// Public Function Prototypes
bool InitCarNodeTransmitService ( uint8_t Priority );
bool PostCarNodeTransmitService( ES_Event ThisEvent );
ES_Event RunCarNodeTransmitService( ES_Event ThisEvent );
void TransmitISR ( void ); 

#endif

 /* CarNodeTransmitService_H */

