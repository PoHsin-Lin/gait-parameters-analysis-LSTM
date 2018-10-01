/**************************************************************************************************
	Filename:       Calibration_value.h
	Revised:        $Date:2016-11-12 21:15:28 $

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

#include "Calibration_value.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

#define SERVAPP_NUM_ATTR_SUPPORTED        25

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

// GATT Profile Service UUID: 0x2210
CONST uint8 Calibration_valueServUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(CALIBRATION_VALUE_SERV_UUID), HI_UINT16(CALIBRATION_VALUE_SERV_UUID)
};

// Acc_Bias UUID: 0x2211
CONST uint8 Acc_BiasUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(ACC_BIAS_UUID), HI_UINT16(ACC_BIAS_UUID)
};

// Gyo_Bias UUID: 0x2212
CONST uint8 Gyo_BiasUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(GYO_BIAS_UUID), HI_UINT16(GYO_BIAS_UUID)
};

// Mag_Bias UUID: 0x2213
CONST uint8 Mag_BiasUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(MAG_BIAS_UUID), HI_UINT16(MAG_BIAS_UUID)
};

// Mag_Scale UUID: 0x2214
CONST uint8 Mag_ScaleUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(MAG_SCALE_UUID), HI_UINT16(MAG_SCALE_UUID)
};

// Other1 UUID: 0x2215
CONST uint8 Mag_CaliUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(MAG_CALI_UUID), HI_UINT16(MAG_CALI_UUID)
};

// Other2 UUID: 0x2216
CONST uint8 Other2UUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(OTHER2_UUID), HI_UINT16(OTHER2_UUID)
};

// Other3 UUID: 0x2217
CONST uint8 Other3UUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(OTHER3_UUID), HI_UINT16(OTHER3_UUID)
};

// Other4 UUID: 0x2218
CONST uint8 Other4UUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(OTHER4_UUID), HI_UINT16(OTHER4_UUID)
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

static Calibration_valueCBs_t *Calibration_value_AppCBs = NULL;

/*********************************************************************
 * Profile Attributes - variables
 */

// Profile Service attribute
static CONST gattAttrType_t Calibration_valueService = { ATT_BT_UUID_SIZE, Calibration_valueServUUID };

// Profile Acc_Bias Properties
static uint8 Acc_BiasProps = GATT_PROP_READ | GATT_PROP_WRITE;

// Acc_Bias Value
static uint8 Acc_BiasValue[12] = {0,0,0,0,0,0,0,0,0,0,0,0};

// Profile Acc_Bias User Description
static uint8 Acc_BiasUserDesp[17] = "characteristic 1\0";

// Profile Gyo_Bias Properties
static uint8 Gyo_BiasProps = GATT_PROP_READ | GATT_PROP_WRITE;

// Gyo_Bias Value
static uint8 Gyo_BiasValue[12] = {0,0,0,0,0,0,0,0,0,0,0,0};

// Profile Gyo_Bias User Description
static uint8 Gyo_BiasUserDesp[17] = "characteristic 2\0";

// Profile Mag_Bias Properties
static uint8 Mag_BiasProps = GATT_PROP_READ | GATT_PROP_WRITE;

// Mag_Bias Value
static uint8 Mag_BiasValue[12] = {0,0,0,0,0,0,0,0,0,0,0,0};

// Profile Mag_Bias User Description
static uint8 Mag_BiasUserDesp[17] = "characteristic 3\0";

// Profile Mag_Scale Properties
static uint8 Mag_ScaleProps = GATT_PROP_READ | GATT_PROP_WRITE;

// Mag_Scale Value
static uint8 Mag_ScaleValue[12] = {0,0,0,0,0,0,0,0,0,0,0,0};

// Profile Mag_Scale User Description
static uint8 Mag_ScaleUserDesp[17] = "characteristic 4\0";

// Profile Other1 Properties
static uint8 Mag_CaliProps = GATT_PROP_READ | GATT_PROP_WRITE;

// Other1 Value
static uint8 Mag_CaliValue[12] = {0,0,0,0,0,0,0,0,0,0,0,0};

// Profile Other1 User Description
static uint8 Mag_CaliUserDesp[17] = "characteristic 5\0";

// Profile Other2 Properties
static uint8 Other2Props = GATT_PROP_READ | GATT_PROP_WRITE;

// Other2 Value
static uint8 Other2Value[12] = {0,0,0,0,0,0,0,0,0,0,0,0};

// Profile Other2 User Description
static uint8 Other2UserDesp[17] = "characteristic 6\0";

// Profile Other3 Properties
static uint8 Other3Props = GATT_PROP_READ | GATT_PROP_WRITE;

// Other3 Value
static uint8 Other3Value[12] = {0,0,0,0,0,0,0,0,0,0,0,0};

// Profile Other3 User Description
static uint8 Other3UserDesp[17] = "characteristic 7\0";

// Profile Other4 Properties
static uint8 Other4Props = GATT_PROP_READ | GATT_PROP_WRITE;

// Other4 Value
static uint8 Other4Value = 0;// Profile Other4 User Description
static uint8 Other4UserDesp[17] = "characteristic 8\0";

/*********************************************************************
 * Profile Attributes - Table
 */

static gattAttribute_t Calibration_valueAttrTbl[SERVAPP_NUM_ATTR_SUPPORTED] = 
{
  // Simple Profile Service
  {
    { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
    GATT_PERMIT_READ,                         /* permissions */
    0,                                        /* handle */
    (uint8 *)&Calibration_valueService           /* pValue */
  },

    // Acc_Bias Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &Acc_BiasProps
    },

      // Acc_Bias Value
      {
        { ATT_BT_UUID_SIZE, Acc_BiasUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        Acc_BiasValue
      },

      // Acc_Bias User Description
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0,
        Acc_BiasUserDesp
      },

    // Gyo_Bias Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &Gyo_BiasProps
    },

      // Gyo_Bias Value
      {
        { ATT_BT_UUID_SIZE, Gyo_BiasUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        Gyo_BiasValue
      },

      // Gyo_Bias User Description
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0,
        Gyo_BiasUserDesp
      },

    // Mag_Bias Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &Mag_BiasProps
    },

      // Mag_Bias Value
      {
        { ATT_BT_UUID_SIZE, Mag_BiasUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        Mag_BiasValue
      },

      // Mag_Bias User Description
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0,
        Mag_BiasUserDesp
      },

    // Mag_Scale Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &Mag_ScaleProps
    },

      // Mag_Scale Value
      {
        { ATT_BT_UUID_SIZE, Mag_ScaleUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        Mag_ScaleValue
      },

      // Mag_Scale User Description
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0,
        Mag_ScaleUserDesp
      },

    // Other1 Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &Mag_CaliProps
    },

      // Other1 Value
      {
        { ATT_BT_UUID_SIZE, Mag_CaliUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        Mag_CaliValue
      },

      // Other1 User Description
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0,
        Mag_CaliUserDesp
      },

    // Other2 Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &Other2Props
    },

      // Other2 Value
      {
        { ATT_BT_UUID_SIZE, Other2UUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        Other2Value
      },

      // Other2 User Description
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0,
        Other2UserDesp
      },

    // Other3 Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &Other3Props
    },

      // Other3 Value
      {
        { ATT_BT_UUID_SIZE, Other3UUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        Other3Value
      },

      // Other3 User Description
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0,
        Other3UserDesp
      },

    // Other4 Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &Other4Props
    },

      // Other4 Value
      {
        { ATT_BT_UUID_SIZE, Other4UUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        &Other4Value
      },

      // Other4 User Description
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0,
        Other4UserDesp
      },


};


/*********************************************************************
 * LOCAL FUNCTIONS
 */
static uint8 Calibration_value_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr, 
                            uint8 *pValue, uint8 *pLen, uint16 offset, uint8 maxLen );
static bStatus_t Calibration_value_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                 uint8 *pValue, uint8 len, uint16 offset );

static void Calibration_value_HandleConnStatusCB( uint16 connHandle, uint8 changeType );


/*********************************************************************
 * PROFILE CALLBACKS
 */
// Calibration_valueService Callbacks
CONST gattServiceCBs_t Calibration_valueCBs =
{
  Calibration_value_ReadAttrCB,  // Read callback function pointer
  Calibration_value_WriteAttrCB, // Write callback function pointer
  NULL                       // Authorization callback function pointer
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      Calibration_value_AddService
 *
 * @brief   Initializes the Calibration_value service by registering
 *          GATT attributes with the GATT server.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 *
 * @return  Success or Failure
 */
bStatus_t Calibration_value_AddService( uint32 services )
{
  uint8 status = SUCCESS;

  // Initialize Client Characteristic Configuration attributes

  // Register with Link DB to receive link status change callback
  VOID linkDB_Register( Calibration_value_HandleConnStatusCB );

  if ( services & CALIBRATION_VALUE_SERVICE )
  {
    // Register GATT attribute list and CBs with GATT Server App
    status = GATTServApp_RegisterService( Calibration_valueAttrTbl, 
                                          GATT_NUM_ATTRS( Calibration_valueAttrTbl ),
                                          &Calibration_valueCBs );
  }

  return ( status );
}


/*********************************************************************
 * @fn      Calibration_value_RegisterAppCBs
 *
 * @brief   Registers the application callback function. Only call
 *          this function once.
 *
 * @param   callbacks - pointer to application callbacks.
 *
 * @return  SUCCESS or bleAlreadyInRequestedMode
 */
bStatus_t Calibration_value_RegisterAppCBs( Calibration_valueCBs_t *appCallbacks )
{
  if ( appCallbacks )
  {
    Calibration_value_AppCBs = appCallbacks;

    return ( SUCCESS );
  }
  else
  {
    return ( bleAlreadyInRequestedMode );
  }
}


/*********************************************************************
 * @fn      Calibration_value_SetParameter
 *
 * @brief   Set a Calibration_value parameter.
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
bStatus_t Calibration_value_SetParameter( uint8 param, uint8 len, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case ACC_BIAS:
      if ( len == ACC_BIAS_LEN )
      {
        VOID osal_memcpy( Acc_BiasValue, value, ACC_BIAS_LEN );
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case GYO_BIAS:
      if ( len == GYO_BIAS_LEN )
      {
        VOID osal_memcpy( Gyo_BiasValue, value, GYO_BIAS_LEN );
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case MAG_BIAS:
      if ( len == MAG_BIAS_LEN )
      {
        VOID osal_memcpy( Mag_BiasValue, value, MAG_BIAS_LEN );
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case MAG_SCALE:
      if ( len == MAG_SCALE_LEN )
      {
        VOID osal_memcpy( Mag_ScaleValue, value, MAG_SCALE_LEN );
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case MAG_CALI:
      if ( len == MAG_CALI_LEN )
      {
        VOID osal_memcpy( Mag_CaliValue, value, MAG_CALI_LEN );
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case OTHER2:
      if ( len == OTHER2_LEN )
      {
        VOID osal_memcpy( Other2Value, value, OTHER2_LEN );
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case OTHER3:
      if ( len == OTHER3_LEN )
      {
        VOID osal_memcpy( Other3Value, value, OTHER3_LEN );
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case OTHER4:
      if ( len == OTHER4_LEN )
      {
        VOID osal_memcpy( &Other4Value, value, OTHER4_LEN );
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
 * @fn      Calibration_value_GetParameter
 *
 * @brief   Get a Calibration_value parameter.
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to put.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t Calibration_value_GetParameter( uint8 param, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case ACC_BIAS:
      VOID osal_memcpy( value, Acc_BiasValue, ACC_BIAS_LEN );
      break;

    case GYO_BIAS:
      VOID osal_memcpy( value, Gyo_BiasValue, GYO_BIAS_LEN );
      break;

    case MAG_BIAS:
      VOID osal_memcpy( value, Mag_BiasValue, MAG_BIAS_LEN );
      break;

    case MAG_SCALE:
      VOID osal_memcpy( value, Mag_ScaleValue, MAG_SCALE_LEN );
      break;

    case MAG_CALI:
      VOID osal_memcpy( value, Mag_CaliValue, MAG_CALI_LEN );
      break;

    case OTHER2:
      VOID osal_memcpy( value, Other2Value, OTHER2_LEN );
      break;

    case OTHER3:
      VOID osal_memcpy( value, Other3Value, OTHER3_LEN );
      break;

    case OTHER4:
      VOID osal_memcpy( value, &Other4Value, OTHER4_LEN );
      break;

    default:
      ret = INVALIDPARAMETER;
      break;
  }

  return ( ret );
}

/*********************************************************************
 * @fn          Calibration_value_ReadAttrCB
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
static uint8 Calibration_value_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr, 
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

      case ACC_BIAS_UUID:
        *pLen = ACC_BIAS_LEN;
        VOID osal_memcpy( pValue, pAttr->pValue, ACC_BIAS_LEN );
        break;

      case GYO_BIAS_UUID:
        *pLen = GYO_BIAS_LEN;
        VOID osal_memcpy( pValue, pAttr->pValue, GYO_BIAS_LEN );
        break;

      case MAG_BIAS_UUID:
        *pLen = MAG_BIAS_LEN;
        VOID osal_memcpy( pValue, pAttr->pValue, MAG_BIAS_LEN );
        break;

      case MAG_SCALE_UUID:
        *pLen = MAG_SCALE_LEN;
        VOID osal_memcpy( pValue, pAttr->pValue, MAG_SCALE_LEN );
        break;

      case MAG_CALI_UUID:
        *pLen = MAG_CALI_LEN;
        VOID osal_memcpy( pValue, pAttr->pValue, MAG_CALI_LEN );
        break;

      case OTHER2_UUID:
        *pLen = OTHER2_LEN;
        VOID osal_memcpy( pValue, pAttr->pValue, OTHER2_LEN );
        break;

      case OTHER3_UUID:
        *pLen = OTHER3_LEN;
        VOID osal_memcpy( pValue, pAttr->pValue, OTHER3_LEN );
        break;

      case OTHER4_UUID:
        *pLen = OTHER4_LEN;
        VOID osal_memcpy( pValue, pAttr->pValue, OTHER4_LEN );
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
 * @fn      Calibration_value_WriteAttrCB
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
static bStatus_t Calibration_value_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
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
      case ACC_BIAS_UUID:
        //Validate the value
        // Make sure it's not a blob oper
        if ( offset == 0 )
        {
          if ( len != ACC_BIAS_LEN )
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
          VOID osal_memcpy( pAttr->pValue, pValue, ACC_BIAS_LEN );
          notifyApp = ACC_BIAS;
        }
        break;

      case GYO_BIAS_UUID:
        //Validate the value
        // Make sure it's not a blob oper
        if ( offset == 0 )
        {
          if ( len != GYO_BIAS_LEN )
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
          VOID osal_memcpy( pAttr->pValue, pValue, GYO_BIAS_LEN );
          notifyApp = GYO_BIAS;
        }
        break;

      case MAG_BIAS_UUID:
        //Validate the value
        // Make sure it's not a blob oper
        if ( offset == 0 )
        {
          if ( len != MAG_BIAS_LEN )
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
          VOID osal_memcpy( pAttr->pValue, pValue, MAG_BIAS_LEN );
          notifyApp = MAG_BIAS;
        }
        break;

      case MAG_SCALE_UUID:
        //Validate the value
        // Make sure it's not a blob oper
        if ( offset == 0 )
        {
          if ( len != MAG_SCALE_LEN )
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
          VOID osal_memcpy( pAttr->pValue, pValue, MAG_SCALE_LEN );
          notifyApp = MAG_SCALE;
        }
        break;

      case MAG_CALI_UUID:
        //Validate the value
        // Make sure it's not a blob oper
        if ( offset == 0 )
        {
          if ( len != MAG_CALI_LEN )
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
          VOID osal_memcpy( pAttr->pValue, pValue, MAG_CALI_LEN );
          notifyApp = MAG_CALI;
        }
        break;

      case OTHER2_UUID:
        //Validate the value
        // Make sure it's not a blob oper
        if ( offset == 0 )
        {
          if ( len != OTHER2_LEN )
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
          VOID osal_memcpy( pAttr->pValue, pValue, OTHER2_LEN );
          notifyApp = OTHER2;
        }
        break;

      case OTHER3_UUID:
        //Validate the value
        // Make sure it's not a blob oper
        if ( offset == 0 )
        {
          if ( len != OTHER3_LEN )
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
          VOID osal_memcpy( pAttr->pValue, pValue, OTHER3_LEN );
          notifyApp = OTHER3;
        }
        break;

      case OTHER4_UUID:
        //Validate the value
        // Make sure it's not a blob oper
        if ( offset == 0 )
        {
          if ( len != OTHER4_LEN )
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
          VOID osal_memcpy( pAttr->pValue, pValue, OTHER4_LEN );
          notifyApp = OTHER4;
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
  if ( (notifyApp != 0xFF ) && Calibration_value_AppCBs && Calibration_value_AppCBs->pfnCalibration_valueChange )
  {
    Calibration_value_AppCBs->pfnCalibration_valueChange( notifyApp );  
  }

  return ( status );
}

/*********************************************************************
 * @fn          Calibration_value_HandleConnStatusCB
 *
 * @brief       Calibration_value link status change handler function.
 *
 * @param       connHandle - connection handle
 * @param       changeType - type of change
 *
 * @return      none
 */
static void Calibration_value_HandleConnStatusCB( uint16 connHandle, uint8 changeType )
{
  // Make sure this is not loopback connection
  if ( connHandle != LOOPBACK_CONNHANDLE )
  {
    // Reset Client Char Config if connection has dropped
    if ( ( changeType == LINKDB_STATUS_UPDATE_REMOVED )      ||
         ( ( changeType == LINKDB_STATUS_UPDATE_STATEFLAGS ) && 
           ( !linkDB_Up( connHandle ) ) ) )
    {
    }
  }
}



/*********************************************************************
*********************************************************************/
