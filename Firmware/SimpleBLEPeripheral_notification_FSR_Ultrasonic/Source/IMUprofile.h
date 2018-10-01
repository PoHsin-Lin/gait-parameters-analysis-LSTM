/**************************************************************************************************
	Filename:       IMUProfile.h
	Revised:        $Date:2016-03-01 12:59:55 $

	Description:    This file contains the GATT profile definitions and prototypes..

	Copyright 2013 EPLAB National Tsing Hua University. All rights reserved.
	The information contained herein is confidential property of NTHU. 	The material may be used for a personal and non-commercial use only in connection with 	a legitimate academic research purpose. Any attempt to copy, modify, and distribute any portion of this source code or derivative thereof for commercial, political, or propaganda purposes is strictly prohibited. All other uses require explicit written permission from the authors and copyright holders. This copyright statement must be retained in its entirety and displayed in the copyright statement of derived source code or systems.
**************************************************************************************************/

#ifndef IMUPROFILE_H
#define IMUPROFILE_H

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
#define EULER_ANGLE		0

// IMUProfile Profile Service UUID
#define IMUPROFILE_SERV_UUID		0xFFB0

// IMUProfile UUID
#define EULER_ANGLE_UUID		0xFFB1

// IMUProfile Profile Services bit fields
#define IMUPROFILE_SERVICE		0x00000001

// Length of Characteristics
#define EULER_ANGLE_LEN		12

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
typedef NULL_OK void (*IMUProfileChange_t)( uint8 paramID );

typedef struct
{
  IMUProfileChange_t		pfnIMUProfileChange; // Called when characteristic value changes
} IMUProfileCBs_t;



/*********************************************************************
 * API FUNCTIONS 
 */

/*
 * IMUProfile_AddService- Initializes the GATT Profile service by registering
 *          GATT attributes with the GATT server.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 */

extern bStatus_t IMUProfile_AddService( uint32 services );

/*
 * IMUProfile_RegisterAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
extern bStatus_t IMUProfile_RegisterAppCBs( IMUProfileCBs_t *appCallbacks );

/*
 * IMUProfile_SetParameter - Set a GATT Profile parameter.
 *
 *    param - Profile parameter ID
 *    len - length of data to right
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
extern bStatus_t IMUProfile_SetParameter( uint8 param, uint8 len, void *value );

/*
 * IMUProfile_GetParameter - Get a GATT Profile parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 */
extern bStatus_t IMUProfile_GetParameter( uint8 param, void *value );


/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* IMUPROFILE_H */	
