/**************************************************************************************************
	Filename:       reminderProfile.h
	Revised:        $Date:2016-04-16 20:28:57 $

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

#include "reminderProfile.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

#define SERVAPP_NUM_ATTR_SUPPORTED        27

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

// GATT Profile Service UUID: 0xFED0
CONST uint8 reminderProfileServUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(REMINDERPROFILE_SERV_UUID), HI_UINT16(REMINDERPROFILE_SERV_UUID)
};

// TRAINNUMBER UUID: 0xFED1
CONST uint8 TRAINNUMBERUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(TRAINNUMBER_UUID), HI_UINT16(TRAINNUMBER_UUID)
};

// DEPART UUID: 0xFED2
CONST uint8 DEPARTUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(DEPART_UUID), HI_UINT16(DEPART_UUID)
};

// DESTINATION UUID: 0xFED3
CONST uint8 DESTINATIONUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(DESTINATION_UUID), HI_UINT16(DESTINATION_UUID)
};

// VIBRATION UUID: 0xFED4
CONST uint8 VIBRATIONUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(VIBRATION_UUID), HI_UINT16(VIBRATION_UUID)
};

// BROADCAST UUID: 0xFED5
CONST uint8 BROADCASTUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(BROADCAST_UUID), HI_UINT16(BROADCAST_UUID)
};

// STARTDETECT UUID: 0xFED6
CONST uint8 STARTDETECTUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(STARTDETECT_UUID), HI_UINT16(STARTDETECT_UUID)
};

// EULER_ANGLE UUID: 0xFED7
CONST uint8 EULER_ANGLEUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(EULER_ANGLE_UUID), HI_UINT16(EULER_ANGLE_UUID)
};

// MACADDRESS UUID: 0xFED8
CONST uint8 MACADDRESSUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(MACADDRESS_UUID), HI_UINT16(MACADDRESS_UUID)
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

static reminderProfileCBs_t *reminderProfile_AppCBs = NULL;

/*********************************************************************
 * Profile Attributes - variables
 */

// Profile Service attribute
static CONST gattAttrType_t reminderProfileService = { ATT_BT_UUID_SIZE, reminderProfileServUUID };

// Profile TRAINNUMBER Properties
static uint8 TRAINNUMBERProps = GATT_PROP_READ | GATT_PROP_WRITE;

// TRAINNUMBER Value
static uint8 TRAINNUMBERValue[2] = {0,0};

// Profile TRAINNUMBER User Description
static uint8 TRAINNUMBERUserDesp[17] = "characteristic 1\0";

// Profile DEPART Properties
static uint8 DEPARTProps = GATT_PROP_READ | GATT_PROP_WRITE;

// DEPART Value
static uint8 DEPARTValue = 0;// Profile DEPART User Description
static uint8 DEPARTUserDesp[17] = "characteristic 2\0";

// Profile DESTINATION Properties
static uint8 DESTINATIONProps = GATT_PROP_READ | GATT_PROP_WRITE;

// DESTINATION Value
static uint8 DESTINATIONValue = 0;// Profile DESTINATION User Description
static uint8 DESTINATIONUserDesp[17] = "characteristic 3\0";

// Profile VIBRATION Properties
static uint8 VIBRATIONProps = GATT_PROP_READ | GATT_PROP_WRITE | GATT_PROP_NOTIFY;

// VIBRATION Value
static uint8 VIBRATIONValue = 0;// Profile VIBRATION User Description
static uint8 VIBRATIONUserDesp[17] = "characteristic 4\0";

// reminderProfile VIBRATION Configuration Each client has its own
// instantiation of the Client Characteristic Configuration. Reads of the
// Client Characteristic Configuration only shows the configuration for
// that client and writes only affect the configuration of that client.
static gattCharCfg_t VIBRATIONConfig[GATT_MAX_NUM_CONN];

// Profile BROADCAST Properties
static uint8 BROADCASTProps = GATT_PROP_READ | GATT_PROP_WRITE;

// BROADCAST Value
static uint8 BROADCASTValue[4] = {0,0,0,0};// Profile BROADCAST User Description
static uint8 BROADCASTUserDesp[17] = "characteristic 5\0";

// Profile STARTDETECT Properties
static uint8 STARTDETECTProps = GATT_PROP_READ | GATT_PROP_WRITE;

// STARTDETECT Value
static uint8 STARTDETECTValue = 0;// Profile STARTDETECT User Description
static uint8 STARTDETECTUserDesp[17] = "characteristic 6\0";

// Profile EULER_ANGLE Properties
static uint8 EULER_ANGLEProps = GATT_PROP_READ | GATT_PROP_WRITE | GATT_PROP_NOTIFY;

// EULER_ANGLE Value
static uint8 EULER_ANGLEValue[2] = {0,0};

// Profile EULER_ANGLE User Description
static uint8 EULER_ANGLEUserDesp[17] = "characteristic 7\0";

// reminderProfile EULER_ANGLE Configuration Each client has its own
// instantiation of the Client Characteristic Configuration. Reads of the
// Client Characteristic Configuration only shows the configuration for
// that client and writes only affect the configuration of that client.
static gattCharCfg_t EULER_ANGLEConfig[GATT_MAX_NUM_CONN];

// Profile MACADDRESS Properties
static uint8 MACADDRESSProps = GATT_PROP_READ | GATT_PROP_WRITE;

// MACADDRESS Value
static uint8 MACADDRESSValue[12] = {0,0,0,0,0,0,0,0,0,0,0,0};

// Profile MACADDRESS User Description
static uint8 MACADDRESSUserDesp[17] = "characteristic 8\0";

int successCount;
int failCount = 0;
bStatus_t notify;
char buf[100];


/*********************************************************************
 * Profile Attributes - Table
 */

static gattAttribute_t reminderProfileAttrTbl[SERVAPP_NUM_ATTR_SUPPORTED] = 
{
  // Simple Profile Service
  {
    { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
    GATT_PERMIT_READ,                         /* permissions */
    0,                                        /* handle */
    (uint8 *)&reminderProfileService           /* pValue */
  },

    // TRAINNUMBER Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &TRAINNUMBERProps
    },

      // TRAINNUMBER Value
      {
        { ATT_BT_UUID_SIZE, TRAINNUMBERUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        TRAINNUMBERValue
      },

      // TRAINNUMBER User Description
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0,
        TRAINNUMBERUserDesp
      },

    // DEPART Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &DEPARTProps
    },

      // DEPART Value
      {
        { ATT_BT_UUID_SIZE, DEPARTUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        &DEPARTValue
      },

      // DEPART User Description
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0,
        DEPARTUserDesp
      },

    // DESTINATION Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &DESTINATIONProps
    },

      // DESTINATION Value
      {
        { ATT_BT_UUID_SIZE, DESTINATIONUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        &DESTINATIONValue
      },

      // DESTINATION User Description
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0,
        DESTINATIONUserDesp
      },

    // VIBRATION Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &VIBRATIONProps
    },

      // VIBRATION Value
      {
        { ATT_BT_UUID_SIZE, VIBRATIONUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        &VIBRATIONValue
      },

      // VIBRATION configuration
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8 *) VIBRATIONConfig
      },

      // VIBRATION User Description
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0,
        VIBRATIONUserDesp
      },

    // BROADCAST Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &BROADCASTProps
    },

      // BROADCAST Value
      {
        { ATT_BT_UUID_SIZE, BROADCASTUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        BROADCASTValue
      },

      // BROADCAST User Description
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0,
        BROADCASTUserDesp
      },

    // STARTDETECT Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &STARTDETECTProps
    },

      // STARTDETECT Value
      {
        { ATT_BT_UUID_SIZE, STARTDETECTUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        &STARTDETECTValue
      },

      // STARTDETECT User Description
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0,
        STARTDETECTUserDesp
      },

    // EULER_ANGLE Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &EULER_ANGLEProps
    },

      // EULER_ANGLE Value
      {
        { ATT_BT_UUID_SIZE, EULER_ANGLEUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        EULER_ANGLEValue
      },

      // EULER_ANGLE configuration
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8 *) EULER_ANGLEConfig
      },

      // EULER_ANGLE User Description
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0,
        EULER_ANGLEUserDesp
      },

    // MACADDRESS Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &MACADDRESSProps
    },

      // MACADDRESS Value
      {
        { ATT_BT_UUID_SIZE, MACADDRESSUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        MACADDRESSValue
      },

      // MACADDRESS User Description
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0,
        MACADDRESSUserDesp
      },


};


/*********************************************************************
 * LOCAL FUNCTIONS
 */
static uint8 reminderProfile_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr, 
                            uint8 *pValue, uint8 *pLen, uint16 offset, uint8 maxLen );
static bStatus_t reminderProfile_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                 uint8 *pValue, uint8 len, uint16 offset );

static void reminderProfile_HandleConnStatusCB( uint16 connHandle, uint8 changeType );


/*********************************************************************
 * PROFILE CALLBACKS
 */
// reminderProfileService Callbacks
CONST gattServiceCBs_t reminderProfileCBs =
{
  reminderProfile_ReadAttrCB,  // Read callback function pointer
  reminderProfile_WriteAttrCB, // Write callback function pointer
  NULL                       // Authorization callback function pointer
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      reminderProfile_AddService
 *
 * @brief   Initializes the reminderProfile service by registering
 *          GATT attributes with the GATT server.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 *
 * @return  Success or Failure
 */
bStatus_t reminderProfile_AddService( uint32 services )
{
  uint8 status = SUCCESS;

  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, VIBRATIONConfig);
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, EULER_ANGLEConfig);

  // Register with Link DB to receive link status change callback
  VOID linkDB_Register( reminderProfile_HandleConnStatusCB );

  if ( services & REMINDERPROFILE_SERVICE )
  {
    // Register GATT attribute list and CBs with GATT Server App
    status = GATTServApp_RegisterService( reminderProfileAttrTbl, 
                                          GATT_NUM_ATTRS( reminderProfileAttrTbl ),
                                          &reminderProfileCBs );
  }

  return ( status );
}


/*********************************************************************
 * @fn      reminderProfile_RegisterAppCBs
 *
 * @brief   Registers the application callback function. Only call
 *          this function once.
 *
 * @param   callbacks - pointer to application callbacks.
 *
 * @return  SUCCESS or bleAlreadyInRequestedMode
 */
bStatus_t reminderProfile_RegisterAppCBs( reminderProfileCBs_t *appCallbacks )
{
  if ( appCallbacks )
  {
    reminderProfile_AppCBs = appCallbacks;

    return ( SUCCESS );
  }
  else
  {
    return ( bleAlreadyInRequestedMode );
  }
}


/*********************************************************************
 * @fn      reminderProfile_SetParameter
 *
 * @brief   Set a reminderProfile parameter.
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
bStatus_t reminderProfile_SetParameter( uint8 param, uint8 len, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case TRAINNUMBER:
      if ( len == TRAINNUMBER_LEN )
      {
        VOID osal_memcpy( TRAINNUMBERValue, value, TRAINNUMBER_LEN );
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case DEPART:
      if ( len == DEPART_LEN )
      {
        VOID osal_memcpy( &DEPARTValue, value, DEPART_LEN );
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case DESTINATION:
      if ( len == DESTINATION_LEN )
      {
        VOID osal_memcpy( &DESTINATIONValue, value, DESTINATION_LEN );
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case VIBRATION:
      if ( len == VIBRATION_LEN )
      {
        VOID osal_memcpy( &VIBRATIONValue, value, VIBRATION_LEN );
        // See if Notification has been enabled 
        GATTServApp_ProcessCharCfg( VIBRATIONConfig, &VIBRATIONValue, FALSE,
        reminderProfileAttrTbl, GATT_NUM_ATTRS( reminderProfileAttrTbl ),
        INVALID_TASK_ID );
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case BROADCAST:
      if ( len == BROADCAST_LEN )
      {
        VOID osal_memcpy( &BROADCASTValue, value, BROADCAST_LEN );
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case STARTDETECT:
      if ( len == STARTDETECT_LEN )
      {
        VOID osal_memcpy( &STARTDETECTValue, value, STARTDETECT_LEN );
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case EULER_ANGLE:
      if ( len == EULER_ANGLE_LEN )
      {
        VOID osal_memcpy( EULER_ANGLEValue, value, EULER_ANGLE_LEN );
        // See if Notification has been enabled 
        notify = GATTServApp_ProcessCharCfg( EULER_ANGLEConfig, EULER_ANGLEValue, FALSE,
        reminderProfileAttrTbl, GATT_NUM_ATTRS( reminderProfileAttrTbl ),
        INVALID_TASK_ID);
//        if(notify == 0x00){
//          successCount++;
//        } else{
//          failCount++;
//        }
//        sprintf(buf, "success = %d, fail = %d\r\n", successCount, failCount);
//        uartWriteString(buf);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case MACADDRESS:
      if ( len == MACADDRESS_LEN )
      {
        VOID osal_memcpy( MACADDRESSValue, value, MACADDRESS_LEN );
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
 * @fn      reminderProfile_GetParameter
 *
 * @brief   Get a reminderProfile parameter.
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to put.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t reminderProfile_GetParameter( uint8 param, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case TRAINNUMBER:
      VOID osal_memcpy( value, TRAINNUMBERValue, TRAINNUMBER_LEN );
      break;

    case DEPART:
      VOID osal_memcpy( value, &DEPARTValue, DEPART_LEN );
      break;

    case DESTINATION:
      VOID osal_memcpy( value, &DESTINATIONValue, DESTINATION_LEN );
      break;

    case VIBRATION:
      VOID osal_memcpy( value, &VIBRATIONValue, VIBRATION_LEN );
      break;

    case BROADCAST:
      VOID osal_memcpy( value, &BROADCASTValue, BROADCAST_LEN );
      break;

    case STARTDETECT:
      VOID osal_memcpy( value, &STARTDETECTValue, STARTDETECT_LEN );
      break;

    case EULER_ANGLE:
      VOID osal_memcpy( value, EULER_ANGLEValue, EULER_ANGLE_LEN );
      break;

    case MACADDRESS:
      VOID osal_memcpy( value, MACADDRESSValue, MACADDRESS_LEN );
      break;

    default:
      ret = INVALIDPARAMETER;
      break;
  }

  return ( ret );
}

/*********************************************************************
 * @fn          reminderProfile_ReadAttrCB
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
static uint8 reminderProfile_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr, 
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

      case TRAINNUMBER_UUID:
        *pLen = TRAINNUMBER_LEN;
        VOID osal_memcpy( pValue, pAttr->pValue, TRAINNUMBER_LEN );
        break;

      case DEPART_UUID:
        *pLen = DEPART_LEN;
        VOID osal_memcpy( pValue, pAttr->pValue, DEPART_LEN );
        break;

      case DESTINATION_UUID:
        *pLen = DESTINATION_LEN;
        VOID osal_memcpy( pValue, pAttr->pValue, DESTINATION_LEN );
        break;

      case VIBRATION_UUID:
        *pLen = VIBRATION_LEN;
        VOID osal_memcpy( pValue, pAttr->pValue, VIBRATION_LEN );
        break;

      case BROADCAST_UUID:
        *pLen = BROADCAST_LEN;
        VOID osal_memcpy( pValue, pAttr->pValue, BROADCAST_LEN );
        break;

      case STARTDETECT_UUID:
        *pLen = STARTDETECT_LEN;
        VOID osal_memcpy( pValue, pAttr->pValue, STARTDETECT_LEN );
        break;

      case EULER_ANGLE_UUID:
        *pLen = EULER_ANGLE_LEN;
        VOID osal_memcpy( pValue, pAttr->pValue, EULER_ANGLE_LEN );
        break;

      case MACADDRESS_UUID:
        *pLen = MACADDRESS_LEN;
        VOID osal_memcpy( pValue, pAttr->pValue, MACADDRESS_LEN );
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
 * @fn      reminderProfile_WriteAttrCB
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
static bStatus_t reminderProfile_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
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
      case TRAINNUMBER_UUID:
        //Validate the value
        // Make sure it's not a blob oper
        if ( offset == 0 )
        {
          if ( len != TRAINNUMBER_LEN )
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
          VOID osal_memcpy( pAttr->pValue, pValue, TRAINNUMBER_LEN );
          notifyApp = TRAINNUMBER;
        }
        break;

      case DEPART_UUID:
        //Validate the value
        // Make sure it's not a blob oper
        if ( offset == 0 )
        {
          if ( len != DEPART_LEN )
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
          VOID osal_memcpy( pAttr->pValue, pValue, DEPART_LEN );
          notifyApp = DEPART;
        }
        break;

      case DESTINATION_UUID:
        //Validate the value
        // Make sure it's not a blob oper
        if ( offset == 0 )
        {
          if ( len != DESTINATION_LEN )
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
          VOID osal_memcpy( pAttr->pValue, pValue, DESTINATION_LEN );
          notifyApp = DESTINATION;
        }
        break;

      case VIBRATION_UUID:
        //Validate the value
        // Make sure it's not a blob oper
        if ( offset == 0 )
        {
          if ( len != VIBRATION_LEN )
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
          VOID osal_memcpy( pAttr->pValue, pValue, VIBRATION_LEN );
          notifyApp = VIBRATION;
        }
        break;

      case BROADCAST_UUID:
        //Validate the value
        // Make sure it's not a blob oper
        if ( offset == 0 )
        {
          if ( len != BROADCAST_LEN )
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
          VOID osal_memcpy( pAttr->pValue, pValue, BROADCAST_LEN );
          notifyApp = BROADCAST;
        }
        break;

      case STARTDETECT_UUID:
        //Validate the value
        // Make sure it's not a blob oper
        if ( offset == 0 )
        {
          if ( len != STARTDETECT_LEN )
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
          VOID osal_memcpy( pAttr->pValue, pValue, STARTDETECT_LEN );
          notifyApp = STARTDETECT;
        }
        break;

      case EULER_ANGLE_UUID:
        //Validate the value
        // Make sure it's not a blob oper
        if ( offset == 0 )
        {
          if ( len != EULER_ANGLE_LEN )
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
          VOID osal_memcpy( pAttr->pValue, pValue, EULER_ANGLE_LEN );
          notifyApp = EULER_ANGLE;
        }
        break;

      case MACADDRESS_UUID:
        //Validate the value
        // Make sure it's not a blob oper
        if ( offset == 0 )
        {
          if ( len != MACADDRESS_LEN )
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
          VOID osal_memcpy( pAttr->pValue, pValue, MACADDRESS_LEN );
          notifyApp = MACADDRESS;
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
  if ( (notifyApp != 0xFF ) && reminderProfile_AppCBs && reminderProfile_AppCBs->pfnreminderProfileChange )
  {
    reminderProfile_AppCBs->pfnreminderProfileChange( notifyApp );  
  }

  return ( status );
}

/*********************************************************************
 * @fn          reminderProfile_HandleConnStatusCB
 *
 * @brief       reminderProfile link status change handler function.
 *
 * @param       connHandle - connection handle
 * @param       changeType - type of change
 *
 * @return      none
 */
static void reminderProfile_HandleConnStatusCB( uint16 connHandle, uint8 changeType )
{
  // Make sure this is not loopback connection
  if ( connHandle != LOOPBACK_CONNHANDLE )
  {
    // Reset Client Char Config if connection has dropped
    if ( ( changeType == LINKDB_STATUS_UPDATE_REMOVED )      ||
         ( ( changeType == LINKDB_STATUS_UPDATE_STATEFLAGS ) && 
           ( !linkDB_Up( connHandle ) ) ) )
    {
      GATTServApp_InitCharCfg( connHandle, VIBRATIONConfig );
      GATTServApp_InitCharCfg( connHandle, EULER_ANGLEConfig );
    }
  }
}



/*********************************************************************
*********************************************************************/
