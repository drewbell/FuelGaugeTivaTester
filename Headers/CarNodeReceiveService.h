/*
 ============================================================
 Name: CarNodeReceiveService.h
 Author: Michal Rittikaidachar
 Version:1
 Date Created: 5/10/17
 Description: Header File for FarmReceiveService.c
 Code adapted from and based off of skeleton files by Ed Carryer
 ============================================================
 */

#ifndef CarNodeReceiveService_H
#define CarNodeReceiveService_H

// Event Definitions
#include "ES_Configure.h" /* gets us event definitions */
#include "ES_Types.h"     /* gets bool type for returns */

// typedefs for the states
// State definitions for use with the query function
typedef enum {InitReceive, Run,WaitingFor0x7E, ReceivingFrameData } ReceiveState_t ;

// Public Function Prototypes
bool InitCarNodeReceiveService ( uint8_t Priority );
bool PostCarNodeReceiveService( ES_Event ThisEvent );
ES_Event RunCarNodeReceiveService( ES_Event ThisEvent );
void ReceiveISR ( void ); 
uint8_t *GetReceivedPacket (void);

#endif

 /* CarNodeReceiveService_H */

