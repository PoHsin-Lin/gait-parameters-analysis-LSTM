/**************************************************************************************************
	Filename:       Calibration_value.h
	Revised:        $Date:2016-11-12 21:15:28 $

	Description:    This file contains the GATT profile definitions and prototypes..

	Copyright 2013 EPLAB National Tsing Hua University. All rights reserved.
	The information contained herein is confidential property of NTHU. 	The material may be used for a personal and non-commercial use only in connection with 	a legitimate academic research purpose. Any attempt to copy, modify, and distribute any portion of this source code or derivative thereof for commercial, political, or propaganda purposes is strictly prohibited. All other uses require explicit written permission from the authors and copyright holders. This copyright statement must be retained in its entirety and displayed in the copyright statement of derived source code or systems.
**************************************************************************************************/

#ifndef CALIBRATION_VALUE_H
#define CALIBRATION_VALUE_H

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
#define ACC_BIAS		0
#define GYO_BIAS		1
#define MAG_BIAS		2
#define MAG_SCALE		3
#define MAG_CALI		4
#define OTHER2		5
#define OTHER3		6
#define OTHER4		7

// Calibration_value Profile Service UUID
#define CALIBRATION_VALUE_SERV_UUID		0x2210

// Calibration_value UUID
#define ACC_BIAS_UUID		0x2211
#define GYO_BIAS_UUID		0x2212
#define MAG_BIAS_UUID		0x2213
#define MAG_SCALE_UUID		0x2214
#define MAG_CALI_UUID		0x2215
#define OTHER2_UUID		0x2216
#define OTHER3_UUID		0x2217
#define OTHER4_UUID		0x2218

// Calibration_value Profile Services bit fields
#define CALIBRATION_VALUE_SERVICE		0x00000001

// Length of Characteristics
#define ACC_BIAS_LEN		12
#define GYO_BIAS_LEN		12
#define MAG_BIAS_LEN		12
#define MAG_SCALE_LEN		12
#define MAG_CALI_LEN		12
#define OTHER2_LEN		12
#define OTHER3_LEN		12
#define OTHER4_LEN		1

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
typedef NULL_OK void (*Calibration_valueChange_t)( uint8 paramID );

typedef struct
{
  Calibration_valueChange_t		pfnCalibration_valueChange; // Called when characteristic value changes
} Calibration_valueCBs_t;



/*********************************************************************
 * API FUNCTIONS 
 */

/*
 * Calibration_value_AddService- Initializes the GATT Profile service by registering
 *          GATT attributes with the GATT server.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 */

extern bStatus_t Calibration_value_AddService( uint32 services );

/*
 * Calibration_value_RegisterAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
extern bStatus_t Calibration_value_RegisterAppCBs( Calibration_valueCBs_t *appCallbacks );

/*
 * Calibration_value_SetParameter - Set a GATT Profile parameter.
 *
 *    param - Profile parameter ID
 *    len - length of data to right
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
extern bStatus_t Calibration_value_SetParameter( uint8 param, uint8 len, void *value );

/*
 * Calibration_value_GetParameter - Get a GATT Profile parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 */
extern bStatus_t Calibration_value_GetParameter( uint8 param, void *value );


/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* CALIBRATION_VALUE_H */	
