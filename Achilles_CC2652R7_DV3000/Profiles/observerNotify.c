/*
 * observerNotify.c
 *
 *  Created on: Feb 1, 2023
 *      Author: Dushan
 */

/**********************************************************************************************
 * Filename:       observerNotify.c
 *
 * Description:    This file contains the implementation of the service.
 *
 * Copyright (c) 2015-2019, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *************************************************************************************************/


/*********************************************************************
 * INCLUDES
 */
#include <string.h>

#include <icall.h>
#include <ti/common/cc26xx/uartlog/UartLog.h>  // Comment out if using xdc Log
/* This Header file contains all BLE API and icall structure definition */
#include "icall_ble_api.h"

#include "observerNotify.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
* GLOBAL VARIABLES
*/

// observerNotify Service UUID
CONST uint8_t observerNotifyUUID[ATT_UUID_SIZE] =
{
  TI_BASE_UUID_128(OBSERVERNOTIFY_SERV_UUID)
};

// notifyChrac UUID
CONST uint8_t observerNotify_NotifyChracUUID[ATT_UUID_SIZE] =
{
  TI_BASE_UUID_128(OBSERVERNOTIFY_NOTIFYCHRAC_UUID)
};

/*********************************************************************
 * LOCAL VARIABLES
 */

static observerNotifyCBs_t *pAppCBs = NULL;

/*********************************************************************
* Profile Attributes - variables
*/

// Service declaration
static CONST gattAttrType_t observerNotifyDecl = { ATT_UUID_SIZE, observerNotifyUUID };

// Characteristic "NotifyChrac" Properties (for declaration)
static uint8_t observerNotify_NotifyChracProps = GATT_PROP_READ | GATT_PROP_WRITE | GATT_PROP_NOTIFY;

// Characteristic "NotifyChrac" Value variable
static uint8_t observerNotify_NotifyChracVal[OBSERVERNOTIFY_NOTIFYCHRAC_LEN] = {0};

// Characteristic "NotifyChrac" CCCD
static gattCharCfg_t *observerNotify_NotifyChracConfig;

/*********************************************************************
* Profile Attributes - Table
*/

static gattAttribute_t observerNotifyAttrTbl[] =
{
  // observerNotify Service Declaration
  {
    { ATT_BT_UUID_SIZE, primaryServiceUUID },
    GATT_PERMIT_READ,
    0,
    (uint8_t *)&observerNotifyDecl
  },
    // NotifyChrac Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &observerNotify_NotifyChracProps
    },
      // NotifyChrac Characteristic Value
      {
        { ATT_UUID_SIZE, observerNotify_NotifyChracUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        observerNotify_NotifyChracVal
      },
      // NotifyChrac CCCD
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8 *)&observerNotify_NotifyChracConfig
      },
};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static bStatus_t observerNotify_ReadAttrCB( uint16_t connHandle, gattAttribute_t *pAttr,
                                           uint8_t *pValue, uint16_t *pLen, uint16_t offset,
                                           uint16_t maxLen, uint8_t method );
static bStatus_t observerNotify_WriteAttrCB( uint16_t connHandle, gattAttribute_t *pAttr,
                                            uint8_t *pValue, uint16_t len, uint16_t offset,
                                            uint8_t method );

/*********************************************************************
 * PROFILE CALLBACKS
 */
// Simple Profile Service Callbacks
CONST gattServiceCBs_t observerNotifyCBs =
{
  observerNotify_ReadAttrCB,  // Read callback function pointer
  observerNotify_WriteAttrCB, // Write callback function pointer
  NULL                       // Authorization callback function pointer
};

/*********************************************************************
* PUBLIC FUNCTIONS
*/

/*
 * ObserverNotify_AddService- Initializes the ObserverNotify service by registering
 *          GATT attributes with the GATT server.
 *
 */
extern bStatus_t ObserverNotify_AddService( uint8_t rspTaskId )
{
  uint8_t status;

  // Allocate Client Characteristic Configuration table
  observerNotify_NotifyChracConfig = (gattCharCfg_t *)ICall_malloc( sizeof(gattCharCfg_t) * linkDBNumConns );
  if ( observerNotify_NotifyChracConfig == NULL )
  {
    return ( bleMemAllocError );
  }

  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg( LINKDB_CONNHANDLE_INVALID, observerNotify_NotifyChracConfig );
  // Register GATT attribute list and CBs with GATT Server App
  status = GATTServApp_RegisterService( observerNotifyAttrTbl,
                                        GATT_NUM_ATTRS( observerNotifyAttrTbl ),
                                        GATT_MAX_ENCRYPT_KEY_SIZE,
                                        &observerNotifyCBs );

  return ( status );
}

/*
 * ObserverNotify_RegisterAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
bStatus_t ObserverNotify_RegisterAppCBs( observerNotifyCBs_t *appCallbacks )
{
  if ( appCallbacks )
  {
    pAppCBs = appCallbacks;

    return ( SUCCESS );
  }
  else
  {
    return ( bleAlreadyInRequestedMode );
  }
}

/*
 * ObserverNotify_SetParameter - Set a ObserverNotify parameter.
 *
 *    param - Profile parameter ID
 *    len - length of data to right
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
bStatus_t ObserverNotify_SetParameter( uint8_t param, uint16_t len, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case OBSERVERNOTIFY_NOTIFYCHRAC_ID:
      if ( len == OBSERVERNOTIFY_NOTIFYCHRAC_LEN )
      {
        memcpy(observerNotify_NotifyChracVal, value, len);

        // Try to send notification.

        GATTServApp_ProcessCharCfg( observerNotify_NotifyChracConfig, (uint8_t *)&observerNotify_NotifyChracVal, FALSE,
                                    observerNotifyAttrTbl, GATT_NUM_ATTRS( observerNotifyAttrTbl ),
                                    INVALID_TASK_ID,  observerNotify_ReadAttrCB);
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
  return ret;
}


/*
 * ObserverNotify_GetParameter - Get a ObserverNotify parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
bStatus_t ObserverNotify_GetParameter( uint8_t param, uint16_t *len, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case OBSERVERNOTIFY_NOTIFYCHRAC_ID:
      memcpy(value, observerNotify_NotifyChracVal, OBSERVERNOTIFY_NOTIFYCHRAC_LEN);
      break;

    default:
      ret = INVALIDPARAMETER;
      break;
  }
  return ret;
}


/*********************************************************************
 * @fn          observerNotify_ReadAttrCB
 *
 * @brief       Read an attribute.
 *
 * @param       connHandle - connection message was received on
 * @param       pAttr - pointer to attribute
 * @param       pValue - pointer to data to be read
 * @param       pLen - length of data to be read
 * @param       offset - offset of the first octet to be read
 * @param       maxLen - maximum length of data to be read
 * @param       method - type of read message
 *
 * @return      SUCCESS, blePending or Failure
 */
static bStatus_t observerNotify_ReadAttrCB( uint16_t connHandle, gattAttribute_t *pAttr,
                                       uint8_t *pValue, uint16_t *pLen, uint16_t offset,
                                       uint16_t maxLen, uint8_t method )
{
  bStatus_t status = SUCCESS;

  // See if request is regarding the NotifyChrac Characteristic Value
if ( ! memcmp(pAttr->type.uuid, observerNotify_NotifyChracUUID, pAttr->type.len) )
  {
    if ( offset > OBSERVERNOTIFY_NOTIFYCHRAC_LEN )  // Prevent malicious ATT ReadBlob offsets.
    {
      status = ATT_ERR_INVALID_OFFSET;
    }
    else
    {
      *pLen = MIN(maxLen, OBSERVERNOTIFY_NOTIFYCHRAC_LEN - offset);  // Transmit as much as possible
      memcpy(pValue, pAttr->pValue + offset, *pLen);
    }
  }
  else
  {
    // If we get here, that means you've forgotten to add an if clause for a
    // characteristic value attribute in the attribute table that has READ permissions.
    *pLen = 0;
    status = ATT_ERR_ATTR_NOT_FOUND;
  }

  return status;
}


/*********************************************************************
 * @fn      observerNotify_WriteAttrCB
 *
 * @brief   Validate attribute data prior to a write operation
 *
 * @param   connHandle - connection message was received on
 * @param   pAttr - pointer to attribute
 * @param   pValue - pointer to data to be written
 * @param   len - length of data
 * @param   offset - offset of the first octet to be written
 * @param   method - type of write message
 *
 * @return  SUCCESS, blePending or Failure
 */
static bStatus_t observerNotify_WriteAttrCB( uint16_t connHandle, gattAttribute_t *pAttr,
                                        uint8_t *pValue, uint16_t len, uint16_t offset,
                                        uint8_t method )
{
  bStatus_t status  = SUCCESS;
  uint8_t   paramID = 0xFF;

  // See if request is regarding a Client Characterisic Configuration
  if ( ! memcmp(pAttr->type.uuid, clientCharCfgUUID, pAttr->type.len) )
  {
    // Allow only notifications.
    status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
                                             offset, GATT_CLIENT_CFG_NOTIFY);
  }
  // See if request is regarding the NotifyChrac Characteristic Value
  else if ( ! memcmp(pAttr->type.uuid, observerNotify_NotifyChracUUID, pAttr->type.len) )
  {
    if ( offset + len > OBSERVERNOTIFY_NOTIFYCHRAC_LEN )
    {
      status = ATT_ERR_INVALID_OFFSET;
    }
    else
    {
      // Copy pValue into the variable we point to from the attribute table.
      memcpy(pAttr->pValue + offset, pValue, len);

      // Only notify application if entire expected value is written
      if ( offset + len == OBSERVERNOTIFY_NOTIFYCHRAC_LEN)
        paramID = OBSERVERNOTIFY_NOTIFYCHRAC_ID;
    }
  }
  else
  {
    // If we get here, that means you've forgotten to add an if clause for a
    // characteristic value attribute in the attribute table that has WRITE permissions.
    status = ATT_ERR_ATTR_NOT_FOUND;
  }

  // Let the application know something changed (if it did) by using the
  // callback it registered earlier (if it did).
  if (paramID != 0xFF)
    if ( pAppCBs && pAppCBs->pfnChangeCb )
      pAppCBs->pfnChangeCb(connHandle, paramID, len, pValue); // Call app function from stack task context.

  return status;
}


