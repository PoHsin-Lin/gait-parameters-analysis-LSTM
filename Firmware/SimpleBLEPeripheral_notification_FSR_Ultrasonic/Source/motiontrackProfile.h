/**************************************************************************************************
	Filename:       motiontrackProfile.h
	Revised:        $Date:2015-03-11 23:05:02 $

	Description:    This file contains the GATT profile definitions and prototypes..

	Copyright 2013 EPLAB National Tsing Hua University. All rights reserved.
	The information contained herein is confidential property of NTHU. 	The material may be used for a personal and non-commercial use only in connection with 	a legitimate academic research purpose. Any attempt to copy, modify, and distribute any portion of this source code or derivative thereof for commercial, political, or propaganda purposes is strictly prohibited. All other uses require explicit written permission from the authors and copyright holders. This copyright statement must be retained in its entirety and displayed in the copyright statement of derived source code or systems.
**************************************************************************************************/

#ifndef MOTIONTRACKPROFILE_H
#define MOTIONTRACKPROFILE_H

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
#define ACC_PERIOD		0
#define ACC_X		1
#define ACC_Y		2
#define ACC_Z		3
#define GYRO_PERIOD		4
#define GYRO_X		5
#define GYRO_Y		6
#define GYRO_Z		7
#define COMPASS_PERIOD		8
#define COMPASS_X		9
#define COMPASS_Y		10
#define COMPASS_Z		11

// motiontrackProfile Profile Service UUID
#define MOTIONTRACKPROFILE_SERV_UUID		0xFFA0

// motiontrackProfile UUID
#define ACC_PERIOD_UUID		0xFFA1
#define ACC_X_UUID		0xFFA2
#define ACC_Y_UUID		0xFFA3
#define ACC_Z_UUID		0xFFA4
#define GYRO_PERIOD_UUID		0xFFA5
#define GYRO_X_UUID		0xFFA6
#define GYRO_Y_UUID		0xFFA7
#define GYRO_Z_UUID		0xFFA8
#define COMPASS_PERIOD_UUID		0xFFA9
#define COMPASS_X_UUID		0xFFAA
#define COMPASS_Y_UUID		0xFFAB
#define COMPASS_Z_UUID		0xFFAC

// motiontrackProfile Profile Services bit fields
#define MOTIONTRACKPROFILE_SERVICE		0x00000001

// Length of Characteristics
#define ACC_PERIOD_LEN		1
#define ACC_X_LEN		2
#define ACC_Y_LEN		2
#define ACC_Z_LEN		2
#define GYRO_PERIOD_LEN		1
#define GYRO_X_LEN		2
#define GYRO_Y_LEN		2
#define GYRO_Z_LEN		2
#define COMPASS_PERIOD_LEN	1
#define COMPASS_X_LEN		2
#define COMPASS_Y_LEN		2
#define COMPASS_Z_LEN		2

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
typedef NULL_OK void (*motiontrackProfileChange_t)( uint8 paramID );

typedef struct
{
  motiontrackProfileChange_t		pfnmotiontrackProfileChange; // Called when characteristic value changes
} motiontrackProfileCBs_t;



/*********************************************************************
 * API FUNCTIONS 
 */

/*
 * motiontrackProfile_AddService- Initializes the GATT Profile service by registering
 *          GATT attributes with the GATT server.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 */

extern bStatus_t motiontrackProfile_AddService( uint32 services );

/*
 * motiontrackProfile_RegisterAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
extern bStatus_t motiontrackProfile_RegisterAppCBs( motiontrackProfileCBs_t *appCallbacks );

/*
 * motiontrackProfile_SetParameter - Set a GATT Profile parameter.
 *
 *    param - Profile parameter ID
 *    len - length of data to right
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
extern bStatus_t motiontrackProfile_SetParameter( uint8 param, uint8 len, void *value );

/*
 * motiontrackProfile_GetParameter - Get a GATT Profile parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 */
extern bStatus_t motiontrackProfile_GetParameter( uint8 param, void *value );


/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* MOTIONTRACKPROFILE_H */	
