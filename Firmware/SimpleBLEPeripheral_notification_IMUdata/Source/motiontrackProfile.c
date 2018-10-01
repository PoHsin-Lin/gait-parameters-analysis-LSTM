/**************************************************************************************************
	Filename:       motiontrackProfile.h
	Revised:        $Date:2015-03-11 23:05:02 $

	Description:    This file contains the GATT profile definitions and prototypes..

	Copyright 2013 EPLAB National Tsing Hua University. All rights reserved.
	The information contained herein is confidential property of NTHU. 	The material may be used for a personal and non-commercial use only in connection with 	a legitimate academic research purpose. Any attempt to copy, modify, and distribute any portion of this source code or derivative thereof for commercial, political, or propaganda purposes is strictly prohibited. All other uses require explicit written permission from the authors and copyright holders. This copyright statement must be retained in its entirety and displayed in the copyright statement of derived source code or systems.
**************************************************************************************************/
/*********************************************************************
 * INCLUDES
 */
#include "bcomdef.h"
#include "OSAL.h"
#include "linkdb.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gattservapp.h"
#include "gapbondmgr.h"

#include "motiontrackProfile.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

#define SERVAPP_NUM_ATTR_SUPPORTED        46

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

// GATT Profile Service UUID: 0xFFA0
CONST uint8 motiontrackProfileServUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(MOTIONTRACKPROFILE_SERV_UUID), HI_UINT16(MOTIONTRACKPROFILE_SERV_UUID)
};

// acc_period UUID: 0xFFA1
CONST uint8 acc_periodUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(ACC_PERIOD_UUID), HI_UINT16(ACC_PERIOD_UUID)
};

// acc_x UUID: 0xFFA2
CONST uint8 acc_xUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(ACC_X_UUID), HI_UINT16(ACC_X_UUID)
};

// acc_y UUID: 0xFFA3
CONST uint8 acc_yUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(ACC_Y_UUID), HI_UINT16(ACC_Y_UUID)
};

// acc_z UUID: 0xFFA4
CONST uint8 acc_zUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(ACC_Z_UUID), HI_UINT16(ACC_Z_UUID)
};

// gyro_period UUID: 0xFFA5
CONST uint8 gyro_periodUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(GYRO_PERIOD_UUID), HI_UINT16(GYRO_PERIOD_UUID)
};

// gyro_x UUID: 0xFFA6
CONST uint8 gyro_xUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(GYRO_X_UUID), HI_UINT16(GYRO_X_UUID)
};

// gyro_y UUID: 0xFFA7
CONST uint8 gyro_yUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(GYRO_Y_UUID), HI_UINT16(GYRO_Y_UUID)
};

// gyro_z UUID: 0xFFA8
CONST uint8 gyro_zUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(GYRO_Z_UUID), HI_UINT16(GYRO_Z_UUID)
};

// compass_period UUID: 0xFFA9
CONST uint8 compass_periodUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(COMPASS_PERIOD_UUID), HI_UINT16(COMPASS_PERIOD_UUID)
};

// compass_x UUID: 0xFFAA
CONST uint8 compass_xUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(COMPASS_X_UUID), HI_UINT16(COMPASS_X_UUID)
};

// compass_y UUID: 0xFFAB
CONST uint8 compass_yUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(COMPASS_Y_UUID), HI_UINT16(COMPASS_Y_UUID)
};

// compass_z UUID: 0xFFAC
CONST uint8 compass_zUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(COMPASS_Z_UUID), HI_UINT16(COMPASS_Z_UUID)
};

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

static motiontrackProfileCBs_t *motiontrackProfile_AppCBs = NULL;

/*********************************************************************
 * Profile Attributes - variables
 */

// Profile Service attribute
static CONST gattAttrType_t motiontrackProfileService = { ATT_BT_UUID_SIZE, motiontrackProfileServUUID };

// Profile acc_period Properties
static uint8 acc_periodProps = GATT_PROP_READ | GATT_PROP_WRITE;

// acc_period Value
static uint8 acc_periodValue = 0;// Profile acc_period User Description
static uint8 acc_periodUserDesp[18] = "Acc config period\0";

// Profile acc_x Properties
static uint8 acc_xProps = GATT_PROP_READ | GATT_PROP_NOTIFY;

// acc_x Value
static uint8 acc_xValue[2] = {0,0};

// Profile acc_x User Description
static uint8 acc_xUserDesp[22] = "accelerometer x value\0";

// motiontrackProfile acc_x Configuration Each client has its own
// instantiation of the Client Characteristic Configuration. Reads of the
// Client Characteristic Configuration only shows the configuration for
// that client and writes only affect the configuration of that client.
static gattCharCfg_t acc_xConfig[GATT_MAX_NUM_CONN];

// Profile acc_y Properties
static uint8 acc_yProps = GATT_PROP_READ | GATT_PROP_NOTIFY;

// acc_y Value
static uint8 acc_yValue[2] = {0,0};

// Profile acc_y User Description
static uint8 acc_yUserDesp[22] = "accelerometer y value\0";

// motiontrackProfile acc_y Configuration Each client has its own
// instantiation of the Client Characteristic Configuration. Reads of the
// Client Characteristic Configuration only shows the configuration for
// that client and writes only affect the configuration of that client.
static gattCharCfg_t acc_yConfig[GATT_MAX_NUM_CONN];

// Profile acc_z Properties
static uint8 acc_zProps = GATT_PROP_READ | GATT_PROP_NOTIFY;

// acc_z Value
static uint8 acc_zValue[2] = {0,0};

// Profile acc_z User Description
static uint8 acc_zUserDesp[22] = "accelerometer z value\0";

// motiontrackProfile acc_z Configuration Each client has its own
// instantiation of the Client Characteristic Configuration. Reads of the
// Client Characteristic Configuration only shows the configuration for
// that client and writes only affect the configuration of that client.
static gattCharCfg_t acc_zConfig[GATT_MAX_NUM_CONN];

// Profile gyro_period Properties
static uint8 gyro_periodProps = GATT_PROP_READ | GATT_PROP_WRITE;

// gyro_period Value
static uint8 gyro_periodValue = 0;// Profile gyro_period User Description
static uint8 gyro_periodUserDesp[19] = "gyro config period\0";

// Profile gyro_x Properties
static uint8 gyro_xProps = GATT_PROP_READ | GATT_PROP_NOTIFY;

// gyro_x Value
static uint8 gyro_xValue[2] = {0,0};

// Profile gyro_x User Description
static uint8 gyro_xUserDesp[13] = "gyro x value\0";

// motiontrackProfile gyro_x Configuration Each client has its own
// instantiation of the Client Characteristic Configuration. Reads of the
// Client Characteristic Configuration only shows the configuration for
// that client and writes only affect the configuration of that client.
static gattCharCfg_t gyro_xConfig[GATT_MAX_NUM_CONN];

// Profile gyro_y Properties
static uint8 gyro_yProps = GATT_PROP_READ | GATT_PROP_NOTIFY;

// gyro_y Value
static uint8 gyro_yValue[2] = {0,0};

// Profile gyro_y User Description
static uint8 gyro_yUserDesp[13] = "gyro y value\0";

// motiontrackProfile gyro_y Configuration Each client has its own
// instantiation of the Client Characteristic Configuration. Reads of the
// Client Characteristic Configuration only shows the configuration for
// that client and writes only affect the configuration of that client.
static gattCharCfg_t gyro_yConfig[GATT_MAX_NUM_CONN];

// Profile gyro_z Properties
static uint8 gyro_zProps = GATT_PROP_READ | GATT_PROP_NOTIFY;

// gyro_z Value
static uint8 gyro_zValue[2] = {0,0};

// Profile gyro_z User Description
static uint8 gyro_zUserDesp[13] = "gyro z value\0";

// motiontrackProfile gyro_z Configuration Each client has its own
// instantiation of the Client Characteristic Configuration. Reads of the
// Client Characteristic Configuration only shows the configuration for
// that client and writes only affect the configuration of that client.
static gattCharCfg_t gyro_zConfig[GATT_MAX_NUM_CONN];

// Profile compass_period Properties
static uint8 compass_periodProps = GATT_PROP_READ | GATT_PROP_WRITE;

// compass_period Value
static uint8 compass_periodValue = 0;// Profile compass_period User Description
static uint8 compass_periodUserDesp[22] = "compass config period\0";

// Profile compass_x Properties
static uint8 compass_xProps = GATT_PROP_READ | GATT_PROP_NOTIFY;

// compass_x Value
static uint8 compass_xValue[2] = {0,0};

// Profile compass_x User Description
static uint8 compass_xUserDesp[16] = "compass x value\0";

// motiontrackProfile compass_x Configuration Each client has its own
// instantiation of the Client Characteristic Configuration. Reads of the
// Client Characteristic Configuration only shows the configuration for
// that client and writes only affect the configuration of that client.
static gattCharCfg_t compass_xConfig[GATT_MAX_NUM_CONN];

// Profile compass_y Properties
static uint8 compass_yProps = GATT_PROP_READ | GATT_PROP_NOTIFY;

// compass_y Value
static uint8 compass_yValue[2] = {0,0};

// Profile compass_y User Description
static uint8 compass_yUserDesp[16] = "compass y value\0";

// motiontrackProfile compass_y Configuration Each client has its own
// instantiation of the Client Characteristic Configuration. Reads of the
// Client Characteristic Configuration only shows the configuration for
// that client and writes only affect the configuration of that client.
static gattCharCfg_t compass_yConfig[GATT_MAX_NUM_CONN];

// Profile compass_z Properties
static uint8 compass_zProps = GATT_PROP_READ | GATT_PROP_NOTIFY;

// compass_z Value
static uint8 compass_zValue[2] = {0,0};

// Profile compass_z User Description
static uint8 compass_zUserDesp[16] = "compass z value\0";

// motiontrackProfile compass_z Configuration Each client has its own
// instantiation of the Client Characteristic Configuration. Reads of the
// Client Characteristic Configuration only shows the configuration for
// that client and writes only affect the configuration of that client.
static gattCharCfg_t compass_zConfig[GATT_MAX_NUM_CONN];

/*********************************************************************
 * Profile Attributes - Table
 */

static gattAttribute_t motiontrackProfileAttrTbl[SERVAPP_NUM_ATTR_SUPPORTED] = 
{
  // Simple Profile Service
  {
    { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
    GATT_PERMIT_READ,                         /* permissions */
    0,                                        /* handle */
    (uint8 *)&motiontrackProfileService           /* pValue */
  },

    // acc_period Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &acc_periodProps
    },

      // acc_period Value
      {
        { ATT_BT_UUID_SIZE, acc_periodUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        &acc_periodValue
      },

      // acc_period User Description
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0,
        acc_periodUserDesp
      },

    // acc_x Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &acc_xProps
    },

      // acc_x Value
      {
        { ATT_BT_UUID_SIZE, acc_xUUID },
        GATT_PERMIT_READ,
        0,
        acc_xValue
      },

      // acc_x configuration
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8 *) acc_xConfig
      },

      // acc_x User Description
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0,
        acc_xUserDesp
      },

    // acc_y Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &acc_yProps
    },

      // acc_y Value
      {
        { ATT_BT_UUID_SIZE, acc_yUUID },
        GATT_PERMIT_READ,
        0,
        acc_yValue
      },

      // acc_y configuration
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8 *) acc_yConfig
      },

      // acc_y User Description
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0,
        acc_yUserDesp
      },

    // acc_z Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &acc_zProps
    },

      // acc_z Value
      {
        { ATT_BT_UUID_SIZE, acc_zUUID },
        GATT_PERMIT_READ,
        0,
        acc_zValue
      },

      // acc_z configuration
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8 *) acc_zConfig
      },

      // acc_z User Description
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0,
        acc_zUserDesp
      },

    // gyro_period Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &gyro_periodProps
    },

      // gyro_period Value
      {
        { ATT_BT_UUID_SIZE, gyro_periodUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        &gyro_periodValue
      },

      // gyro_period User Description
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0,
        gyro_periodUserDesp
      },

    // gyro_x Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &gyro_xProps
    },

      // gyro_x Value
      {
        { ATT_BT_UUID_SIZE, gyro_xUUID },
        GATT_PERMIT_READ,
        0,
        gyro_xValue
      },

      // gyro_x configuration
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8 *) gyro_xConfig
      },

      // gyro_x User Description
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0,
        gyro_xUserDesp
      },

    // gyro_y Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &gyro_yProps
    },

      // gyro_y Value
      {
        { ATT_BT_UUID_SIZE, gyro_yUUID },
        GATT_PERMIT_READ,
        0,
        gyro_yValue
      },

      // gyro_y configuration
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8 *) gyro_yConfig
      },

      // gyro_y User Description
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0,
        gyro_yUserDesp
      },

    // gyro_z Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &gyro_zProps
    },

      // gyro_z Value
      {
        { ATT_BT_UUID_SIZE, gyro_zUUID },
        GATT_PERMIT_READ,
        0,
        gyro_zValue
      },

      // gyro_z configuration
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8 *) gyro_zConfig
      },

      // gyro_z User Description
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0,
        gyro_zUserDesp
      },

    // compass_period Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &compass_periodProps
    },

      // compass_period Value
      {
        { ATT_BT_UUID_SIZE, compass_periodUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        &compass_periodValue
      },

      // compass_period User Description
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0,
        compass_periodUserDesp
      },

    // compass_x Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &compass_xProps
    },

      // compass_x Value
      {
        { ATT_BT_UUID_SIZE, compass_xUUID },
        GATT_PERMIT_READ,
        0,
        compass_xValue
      },

      // compass_x configuration
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8 *) compass_xConfig
      },

      // compass_x User Description
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0,
        compass_xUserDesp
      },

    // compass_y Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &compass_yProps
    },

      // compass_y Value
      {
        { ATT_BT_UUID_SIZE, compass_yUUID },
        GATT_PERMIT_READ,
        0,
        compass_yValue
      },

      // compass_y configuration
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8 *) compass_yConfig
      },

      // compass_y User Description
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0,
        compass_yUserDesp
      },

    // compass_z Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &compass_zProps
    },

      // compass_z Value
      {
        { ATT_BT_UUID_SIZE, compass_zUUID },
        GATT_PERMIT_READ,
        0,
        compass_zValue
      },

      // compass_z configuration
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8 *) compass_zConfig
      },

      // compass_z User Description
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0,
        compass_zUserDesp
      },


};


/*********************************************************************
 * LOCAL FUNCTIONS
 */
static uint8 motiontrackProfile_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr, 
                            uint8 *pValue, uint8 *pLen, uint16 offset, uint8 maxLen );
static bStatus_t motiontrackProfile_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                 uint8 *pValue, uint8 len, uint16 offset );

static void motiontrackProfile_HandleConnStatusCB( uint16 connHandle, uint8 changeType );


/*********************************************************************
 * PROFILE CALLBACKS
 */
// motiontrackProfileService Callbacks
CONST gattServiceCBs_t motiontrackProfileCBs =
{
  motiontrackProfile_ReadAttrCB,  // Read callback function pointer
  motiontrackProfile_WriteAttrCB, // Write callback function pointer
  NULL                       // Authorization callback function pointer
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      motiontrackProfile_AddService
 *
 * @brief   Initializes the motiontrackProfile service by registering
 *          GATT attributes with the GATT server.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 *
 * @return  Success or Failure
 */
bStatus_t motiontrackProfile_AddService( uint32 services )
{
  uint8 status = SUCCESS;

  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, acc_xConfig);
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, acc_yConfig);
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, acc_zConfig);
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, gyro_xConfig);
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, gyro_yConfig);
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, gyro_zConfig);
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, compass_xConfig);
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, compass_yConfig);
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, compass_zConfig);

  // Register with Link DB to receive link status change callback
  VOID linkDB_Register( motiontrackProfile_HandleConnStatusCB );

  if ( services & MOTIONTRACKPROFILE_SERVICE )
  {
    // Register GATT attribute list and CBs with GATT Server App
    status = GATTServApp_RegisterService( motiontrackProfileAttrTbl, 
                                          GATT_NUM_ATTRS( motiontrackProfileAttrTbl ),
                                          &motiontrackProfileCBs );
  }

  return ( status );
}


/*********************************************************************
 * @fn      motiontrackProfile_RegisterAppCBs
 *
 * @brief   Registers the application callback function. Only call
 *          this function once.
 *
 * @param   callbacks - pointer to application callbacks.
 *
 * @return  SUCCESS or bleAlreadyInRequestedMode
 */
bStatus_t motiontrackProfile_RegisterAppCBs( motiontrackProfileCBs_t *appCallbacks )
{
  if ( appCallbacks )
  {
    motiontrackProfile_AppCBs = appCallbacks;

    return ( SUCCESS );
  }
  else
  {
    return ( bleAlreadyInRequestedMode );
  }
}


/*********************************************************************
 * @fn      motiontrackProfile_SetParameter
 *
 * @brief   Set a motiontrackProfile parameter.
 *
 * @param   param - Profile parameter ID
 * @param   len - length of data to right
 * @param   value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t motiontrackProfile_SetParameter( uint8 param, uint8 len, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case ACC_PERIOD:
      if ( len == ACC_PERIOD_LEN )
      {
        VOID osal_memcpy( &acc_periodValue, value, ACC_PERIOD_LEN );
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case ACC_X:
      if ( len == ACC_X_LEN )
      {
        VOID osal_memcpy( acc_xValue, value, ACC_X_LEN );
        // See if Notification has been enabled 
        GATTServApp_ProcessCharCfg( acc_xConfig, acc_xValue, FALSE,
        motiontrackProfileAttrTbl, GATT_NUM_ATTRS( motiontrackProfileAttrTbl ),
        INVALID_TASK_ID );
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case ACC_Y:
      if ( len == ACC_Y_LEN )
      {
        VOID osal_memcpy( acc_yValue, value, ACC_Y_LEN );
        // See if Notification has been enabled 
        GATTServApp_ProcessCharCfg( acc_yConfig, acc_yValue, FALSE,
        motiontrackProfileAttrTbl, GATT_NUM_ATTRS( motiontrackProfileAttrTbl ),
        INVALID_TASK_ID );
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case ACC_Z:
      if ( len == ACC_Z_LEN )
      {
        VOID osal_memcpy( acc_zValue, value, ACC_Z_LEN );
        // See if Notification has been enabled 
        GATTServApp_ProcessCharCfg( acc_zConfig, acc_zValue, FALSE,
        motiontrackProfileAttrTbl, GATT_NUM_ATTRS( motiontrackProfileAttrTbl ),
        INVALID_TASK_ID );
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case GYRO_PERIOD:
      if ( len == GYRO_PERIOD_LEN )
      {
        VOID osal_memcpy( &gyro_periodValue, value, GYRO_PERIOD_LEN );
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case GYRO_X:
      if ( len == GYRO_X_LEN )
      {
        VOID osal_memcpy( gyro_xValue, value, GYRO_X_LEN );
        // See if Notification has been enabled 
        GATTServApp_ProcessCharCfg( gyro_xConfig, gyro_xValue, FALSE,
        motiontrackProfileAttrTbl, GATT_NUM_ATTRS( motiontrackProfileAttrTbl ),
        INVALID_TASK_ID );
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case GYRO_Y:
      if ( len == GYRO_Y_LEN )
      {
        VOID osal_memcpy( gyro_yValue, value, GYRO_Y_LEN );
        // See if Notification has been enabled 
        GATTServApp_ProcessCharCfg( gyro_yConfig, gyro_yValue, FALSE,
        motiontrackProfileAttrTbl, GATT_NUM_ATTRS( motiontrackProfileAttrTbl ),
        INVALID_TASK_ID );
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case GYRO_Z:
      if ( len == GYRO_Z_LEN )
      {
        VOID osal_memcpy( gyro_zValue, value, GYRO_Z_LEN );
        // See if Notification has been enabled 
        GATTServApp_ProcessCharCfg( gyro_zConfig, gyro_zValue, FALSE,
        motiontrackProfileAttrTbl, GATT_NUM_ATTRS( motiontrackProfileAttrTbl ),
        INVALID_TASK_ID );
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case COMPASS_PERIOD:
      if ( len == COMPASS_PERIOD_LEN )
      {
        VOID osal_memcpy( &compass_periodValue, value, COMPASS_PERIOD_LEN );
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case COMPASS_X:
      if ( len == COMPASS_X_LEN )
      {
        VOID osal_memcpy( compass_xValue, value, COMPASS_X_LEN );
        // See if Notification has been enabled 
        GATTServApp_ProcessCharCfg( compass_xConfig, compass_xValue, FALSE,
        motiontrackProfileAttrTbl, GATT_NUM_ATTRS( motiontrackProfileAttrTbl ),
        INVALID_TASK_ID );
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case COMPASS_Y:
      if ( len == COMPASS_Y_LEN )
      {
        VOID osal_memcpy( compass_yValue, value, COMPASS_Y_LEN );
        // See if Notification has been enabled 
        GATTServApp_ProcessCharCfg( compass_yConfig, compass_yValue, FALSE,
        motiontrackProfileAttrTbl, GATT_NUM_ATTRS( motiontrackProfileAttrTbl ),
        INVALID_TASK_ID );
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case COMPASS_Z:
      if ( len == COMPASS_Z_LEN )
      {
        VOID osal_memcpy( compass_zValue, value, COMPASS_Z_LEN );
        // See if Notification has been enabled 
        GATTServApp_ProcessCharCfg( compass_zConfig, compass_zValue, FALSE,
        motiontrackProfileAttrTbl, GATT_NUM_ATTRS( motiontrackProfileAttrTbl ),
        INVALID_TASK_ID );
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    default:
      ret = INVALIDPARAMETER;
      break;
  }

  return ( ret );
}

/*********************************************************************
 * @fn      motiontrackProfile_GetParameter
 *
 * @brief   Get a motiontrackProfile parameter.
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to put.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t motiontrackProfile_GetParameter( uint8 param, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case ACC_PERIOD:
      VOID osal_memcpy( value, &acc_periodValue, ACC_PERIOD_LEN );
      break;

    case ACC_X:
      VOID osal_memcpy( value, acc_xValue, ACC_X_LEN );
      break;

    case ACC_Y:
      VOID osal_memcpy( value, acc_yValue, ACC_Y_LEN );
      break;

    case ACC_Z:
      VOID osal_memcpy( value, acc_zValue, ACC_Z_LEN );
      break;

    case GYRO_PERIOD:
      VOID osal_memcpy( value, &gyro_periodValue, GYRO_PERIOD_LEN );
      break;

    case GYRO_X:
      VOID osal_memcpy( value, gyro_xValue, GYRO_X_LEN );
      break;

    case GYRO_Y:
      VOID osal_memcpy( value, gyro_yValue, GYRO_Y_LEN );
      break;

    case GYRO_Z:
      VOID osal_memcpy( value, gyro_zValue, GYRO_Z_LEN );
      break;

    case COMPASS_PERIOD:
      VOID osal_memcpy( value, &compass_periodValue, COMPASS_PERIOD_LEN );
      break;

    case COMPASS_X:
      VOID osal_memcpy( value, compass_xValue, COMPASS_X_LEN );
      break;

    case COMPASS_Y:
      VOID osal_memcpy( value, compass_yValue, COMPASS_Y_LEN );
      break;

    case COMPASS_Z:
      VOID osal_memcpy( value, compass_zValue, COMPASS_Z_LEN );
      break;

    default:
      ret = INVALIDPARAMETER;
      break;
  }

  return ( ret );
}

/*********************************************************************
 * @fn          motiontrackProfile_ReadAttrCB
 *
 * @brief       Read an attribute.
 *
 * @param       connHandle - connection message was received on
 * @param       pAttr - pointer to attribute
 * @param       pValue - pointer to data to be read
 * @param       pLen - length of data to be read
 * @param       offset - offset of the first octet to be read
 * @param       maxLen - maximum length of data to be read
 *
 * @return      Success or Failure
 */
static uint8 motiontrackProfile_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr, 
                            uint8 *pValue, uint8 *pLen, uint16 offset, uint8 maxLen )
{
  bStatus_t status = SUCCESS;

  // If attribute permissions require authorization to read, return error
  if ( gattPermitAuthorRead( pAttr->permissions ) )
  {
    // Insufficient authorization
    return ( ATT_ERR_INSUFFICIENT_AUTHOR );
  }

  // Make sure it's not a blob operation (no attributes in the profile are long)
  if ( offset > 0 )
  {
    return ( ATT_ERR_ATTR_NOT_LONG );
  }

  if ( pAttr->type.len == ATT_BT_UUID_SIZE )
  {
    // 16-bit UUID
    uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
    switch ( uuid )
    {
      // No need for "GATT_SERVICE_UUID" or "GATT_CLIENT_CHAR_CFG_UUID" cases;
      // gattserverapp handles those reads

      case ACC_PERIOD_UUID:
        *pLen = ACC_PERIOD_LEN;
        VOID osal_memcpy( pValue, pAttr->pValue, ACC_PERIOD_LEN );
        break;

      case ACC_X_UUID:
        *pLen = ACC_X_LEN;
        VOID osal_memcpy( pValue, pAttr->pValue, ACC_X_LEN );
        break;

      case ACC_Y_UUID:
        *pLen = ACC_Y_LEN;
        VOID osal_memcpy( pValue, pAttr->pValue, ACC_Y_LEN );
        break;

      case ACC_Z_UUID:
        *pLen = ACC_Z_LEN;
        VOID osal_memcpy( pValue, pAttr->pValue, ACC_Z_LEN );
        break;

      case GYRO_PERIOD_UUID:
        *pLen = GYRO_PERIOD_LEN;
        VOID osal_memcpy( pValue, pAttr->pValue, GYRO_PERIOD_LEN );
        break;

      case GYRO_X_UUID:
        *pLen = GYRO_X_LEN;
        VOID osal_memcpy( pValue, pAttr->pValue, GYRO_X_LEN );
        break;

      case GYRO_Y_UUID:
        *pLen = GYRO_Y_LEN;
        VOID osal_memcpy( pValue, pAttr->pValue, GYRO_Y_LEN );
        break;

      case GYRO_Z_UUID:
        *pLen = GYRO_Z_LEN;
        VOID osal_memcpy( pValue, pAttr->pValue, GYRO_Z_LEN );
        break;

      case COMPASS_PERIOD_UUID:
        *pLen = COMPASS_PERIOD_LEN;
        VOID osal_memcpy( pValue, pAttr->pValue, COMPASS_PERIOD_LEN );
        break;

      case COMPASS_X_UUID:
        *pLen = COMPASS_X_LEN;
        VOID osal_memcpy( pValue, pAttr->pValue, COMPASS_X_LEN );
        break;

      case COMPASS_Y_UUID:
        *pLen = COMPASS_Y_LEN;
        VOID osal_memcpy( pValue, pAttr->pValue, COMPASS_Y_LEN );
        break;

      case COMPASS_Z_UUID:
        *pLen = COMPASS_Z_LEN;
        VOID osal_memcpy( pValue, pAttr->pValue, COMPASS_Z_LEN );
        break;

      default:
        // Should never get here!
        *pLen = 0;
        status = ATT_ERR_ATTR_NOT_FOUND;
        break;
    }
  }
  else
  {
    // 128-bit UUID
    *pLen = 0;
    status = ATT_ERR_INVALID_HANDLE;
  }

  return ( status );
}

/*********************************************************************
 * @fn      motiontrackProfile_WriteAttrCB
 *
 * @brief   Validate attribute data prior to a write operation
 *
 * @param   connHandle - connection message was received on
 * @param   pAttr - pointer to attribute
 * @param   pValue - pointer to data to be written
 * @param   len - length of data
 * @param   offset - offset of the first octet to be written
 * @param   complete - whether this is the last packet
 * @param   oper - whether to validate and/or write attribute value
 *
 * @return  Success or Failure
 */
static bStatus_t motiontrackProfile_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                 uint8 *pValue, uint8 len, uint16 offset )
{
  bStatus_t status = SUCCESS;
  uint8 notifyApp = 0xFF;

  // If attribute permissions require authorization to write, return error
  if ( gattPermitAuthorWrite( pAttr->permissions ) )
  {
    // Insufficient authorization
    return ( ATT_ERR_INSUFFICIENT_AUTHOR );
  }

  if ( pAttr->type.len == ATT_BT_UUID_SIZE )
  {
    // 16-bit UUID
    uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
    switch ( uuid )
    {
      case ACC_PERIOD_UUID:
        //Validate the value
        // Make sure it's not a blob oper
        if ( offset == 0 )
        {
          if ( len != ACC_PERIOD_LEN )
          {
            status = ATT_ERR_INVALID_VALUE_SIZE;
          }
        }
        else
        {
          status = ATT_ERR_ATTR_NOT_LONG;
        }

        //Write the value
        if ( status == SUCCESS )
        {
          VOID osal_memcpy( pAttr->pValue, pValue, ACC_PERIOD_LEN );
          notifyApp = ACC_PERIOD;
        }
        break;

      case GYRO_PERIOD_UUID:
        //Validate the value
        // Make sure it's not a blob oper
        if ( offset == 0 )
        {
          if ( len != GYRO_PERIOD_LEN )
          {
            status = ATT_ERR_INVALID_VALUE_SIZE;
          }
        }
        else
        {
          status = ATT_ERR_ATTR_NOT_LONG;
        }

        //Write the value
        if ( status == SUCCESS )
        {
          VOID osal_memcpy( pAttr->pValue, pValue, GYRO_PERIOD_LEN );
          notifyApp = GYRO_PERIOD;
        }
        break;

      case COMPASS_PERIOD_UUID:
        //Validate the value
        // Make sure it's not a blob oper
        if ( offset == 0 )
        {
          if ( len != COMPASS_PERIOD_LEN )
          {
            status = ATT_ERR_INVALID_VALUE_SIZE;
          }
        }
        else
        {
          status = ATT_ERR_ATTR_NOT_LONG;
        }

        //Write the value
        if ( status == SUCCESS )
        {
          VOID osal_memcpy( pAttr->pValue, pValue, COMPASS_PERIOD_LEN );
          notifyApp = COMPASS_PERIOD;
        }
        break;


      case GATT_CLIENT_CHAR_CFG_UUID:
        status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
                                                 offset, GATT_CLIENT_CFG_NOTIFY );
        break;

      default:
        status = ATT_ERR_ATTR_NOT_FOUND;
        break;
    }
  }
  else
  {
    // 128-bit UUID
    status = ATT_ERR_INVALID_HANDLE;
  }

  // If a charactersitic value changed then callback function to notify application of change
  if ( (notifyApp != 0xFF ) && motiontrackProfile_AppCBs && motiontrackProfile_AppCBs->pfnmotiontrackProfileChange )
  {
    motiontrackProfile_AppCBs->pfnmotiontrackProfileChange( notifyApp );  
  }

  return ( status );
}

/*********************************************************************
 * @fn          motiontrackProfile_HandleConnStatusCB
 *
 * @brief       motiontrackProfile link status change handler function.
 *
 * @param       connHandle - connection handle
 * @param       changeType - type of change
 *
 * @return      none
 */
static void motiontrackProfile_HandleConnStatusCB( uint16 connHandle, uint8 changeType )
{
  // Make sure this is not loopback connection
  if ( connHandle != LOOPBACK_CONNHANDLE )
  {
    // Reset Client Char Config if connection has dropped
    if ( ( changeType == LINKDB_STATUS_UPDATE_REMOVED )      ||
         ( ( changeType == LINKDB_STATUS_UPDATE_STATEFLAGS ) && 
           ( !linkDB_Up( connHandle ) ) ) )
    {
      GATTServApp_InitCharCfg( connHandle, acc_xConfig );
      GATTServApp_InitCharCfg( connHandle, acc_yConfig );
      GATTServApp_InitCharCfg( connHandle, acc_zConfig );
      GATTServApp_InitCharCfg( connHandle, gyro_xConfig );
      GATTServApp_InitCharCfg( connHandle, gyro_yConfig );
      GATTServApp_InitCharCfg( connHandle, gyro_zConfig );
      GATTServApp_InitCharCfg( connHandle, compass_xConfig );
      GATTServApp_InitCharCfg( connHandle, compass_yConfig );
      GATTServApp_InitCharCfg( connHandle, compass_zConfig );
    }
  }
}



/*********************************************************************
*********************************************************************/
