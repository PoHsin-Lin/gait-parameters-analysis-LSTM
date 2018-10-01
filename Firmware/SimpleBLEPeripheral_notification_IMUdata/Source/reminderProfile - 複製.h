/**************************************************************************************************
	Filename:       reminderProfile.h
	Revised:        $Date:2016-04-16 20:28:57 $

	Description:    This file contains the GATT profile definitions and prototypes..

	Copyright 2013 EPLAB National Tsing Hua University. All rights reserved.
	The information contained herein is confidential property of NTHU. 	The material may be used for a personal and non-commercial use only in connection with 	a legitimate academic research purpose. Any attempt to copy, modify, and distribute any portion of this source code or derivative thereof for commercial, political, or propaganda purposes is strictly prohibited. All other uses require explicit written permission from the authors and copyright holders. This copyright statement must be retained in its entirety and displayed in the copyright statement of derived source code or systems.
**************************************************************************************************/

#ifndef REMINDERPROFILE_H
#define REMINDERPROFILE_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
 * CONSTANTS
 */

// Profile Parameters
#define TRAINNUMBER		0
#define DEPART		1
#define DESTINATION		2
#define VIBRATION		3
#define BROADCAST		4
#define STARTDETECT		5
#define EULER_ANGLE		6
#define MACADDRESS		7

// reminderProfile Profile Service UUID
#define REMINDERPROFILE_SERV_UUID		0xFED0

// reminderProfile UUID
#define TRAINNUMBER_UUID		0xFED1
#define DEPART_UUID		0xFED2
#define DESTINATION_UUID		0xFED3
#define VIBRATION_UUID		0xFED4
#define BROADCAST_UUID		0xFED5
#define STARTDETECT_UUID		0xFED6
#define EULER_ANGLE_UUID		0xFED7
#define MACADDRESS_UUID		0xFED8

// reminderProfile Profile Services bit fields
#define REMINDERPROFILE_SERVICE		0x00000001

// Length of Characteristics
#define TRAINNUMBER_LEN		2
#define DEPART_LEN		1
#define DESTINATION_LEN		1
#define VIBRATION_LEN		1
#define BROADCAST_LEN		1
#define STARTDETECT_LEN		1
#define EULER_ANGLE_LEN		12
#define MACADDRESS_LEN		6

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * Profile Callbacks
 */

// Callback when a characteristic value has changed
typedef NULL_OK void (*reminderProfileChange_t)( uint8 paramID );

typedef struct
{
  reminderProfileChange_t		pfnreminderProfileChange; // Called when characteristic value changes
} reminderProfileCBs_t;



/*********************************************************************
 * API FUNCTIONS 
 */

/*
 * reminderProfile_AddService- Initializes the GATT Profile service by registering
 *          GATT attributes with the GATT server.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 */

extern bStatus_t reminderProfile_AddService( uint32 services );

/*
 * reminderProfile_RegisterAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
extern bStatus_t reminderProfile_RegisterAppCBs( reminderProfileCBs_t *appCallbacks );

/*
 * reminderProfile_SetParameter - Set a GATT Profile parameter.
 *
 *    param - Profile parameter ID
 *    len - length of data to right
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
extern bStatus_t reminderProfile_SetParameter( uint8 param, uint8 len, void *value );

/*
 * reminderProfile_GetParameter - Get a GATT Profile parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 */
extern bStatus_t reminderProfile_GetParameter( uint8 param, void *value );


/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* REMINDERPROFILE_H */	
