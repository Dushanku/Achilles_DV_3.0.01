/*
 * sendNotify.c
 *
 *  Created on: Jun 29, 2023
 *      Author: Dushan
 */

/**********************************************************************************************
 * Filename:       sendNotify.c
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

/* This Header file contains all BLE API and icall structure definition */
#include "icall_ble_api.h"

#include "sendNotify.h"
#include <ti/common/cc26xx/uartlog/UartLog.h>  // Comment out if using xdc Log

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

// sendNotify Service UUID
CONST uint8_t sendNotifyUUID[ATT_UUID_SIZE] =
{
  TI_BASE_UUID_128(SENDNOTIFY_SERV_UUID)
};

// notify_1 UUID
CONST uint8_t sendNotify_Notify_1UUID[ATT_UUID_SIZE] =
{
  TI_BASE_UUID_128(SENDNOTIFY_NOTIFY_1_UUID)
};

/*********************************************************************
 * LOCAL VARIABLES
 */

static sendNotifyCBs_t *pAppCBs = NULL;

/*********************************************************************
* Profile Attributes - variables
*/

// Service declaration
static CONST gattAttrType_t sendNotifyDecl = { ATT_UUID_SIZE, sendNotifyUUID };

// Characteristic "Notify_1" Properties (for declaration)
static uint8_t sendNotify_Notify_1Props = GATT_PROP_READ | GATT_PROP_WRITE | GATT_PROP_NOTIFY;

// Characteristic "Notify_1" Value variable
static uint8_t sendNotify_Notify_1Val[SENDNOTIFY_NOTIFY_1_LEN] = {0};

// Characteristic "Notify_1" CCCD
static gattCharCfg_t *sendNotify_Notify_1Config;

/*********************************************************************
* Profile Attributes - Table
*/

static gattAttribute_t sendNotifyAttrTbl[] =
{
  // sendNotify Service Declaration
  {
    { ATT_BT_UUID_SIZE, primaryServiceUUID },
    GATT_PERMIT_READ,
    0,
    (uint8_t *)&sendNotifyDecl
  },
    // Notify_1 Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &sendNotify_Notify_1Props
    },
      // Notify_1 Characteristic Value
      {
        { ATT_UUID_SIZE, sendNotify_Notify_1UUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        sendNotify_Notify_1Val
      },
      // Notify_1 CCCD
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8 *)&sendNotify_Notify_1Config
      },
};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static bStatus_t sendNotify_ReadAttrCB( uint16_t connHandle, gattAttribute_t *pAttr,
                                           uint8_t *pValue, uint16_t *pLen, uint16_t offset,
                                           uint16_t maxLen, uint8_t method );
static bStatus_t sendNotify_WriteAttrCB( uint16_t connHandle, gattAttribute_t *pAttr,
                                            uint8_t *pValue, uint16_t len, uint16_t offset,
                                            uint8_t method );

/*********************************************************************
 * PROFILE CALLBACKS
 */
// Simple Profile Service Callbacks
CONST gattServiceCBs_t sendNotifyCBs =
{
  sendNotify_ReadAttrCB,  // Read callback function pointer
  sendNotify_WriteAttrCB, // Write callback function pointer
  NULL                       // Authorization callback function pointer
};

/*********************************************************************
* PUBLIC FUNCTIONS
*/

/*
 * SendNotify_AddService- Initializes the SendNotify service by registering
 *          GATT attributes with the GATT server.
 *
 */
extern bStatus_t SendNotify_AddService( uint8_t rspTaskId )
{
  uint8_t status;

  // Allocate Client Characteristic Configuration table
  sendNotify_Notify_1Config = (gattCharCfg_t *)ICall_malloc( sizeof(gattCharCfg_t) * linkDBNumConns );
  if ( sendNotify_Notify_1Config == NULL )
  {
    return ( bleMemAllocError );
  }

  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg( LINKDB_CONNHANDLE_INVALID, sendNotify_Notify_1Config );
  // Register GATT attribute list and CBs with GATT Server App
  status = GATTServApp_RegisterService( sendNotifyAttrTbl,
                                        GATT_NUM_ATTRS( sendNotifyAttrTbl ),
                                        GATT_MAX_ENCRYPT_KEY_SIZE,
                                        &sendNotifyCBs );

  return ( status );
}

/*
 * SendNotify_RegisterAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
bStatus_t SendNotify_RegisterAppCBs( sendNotifyCBs_t *appCallbacks )
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
 * SendNotify_SetParameter - Set a SendNotify parameter.
 *
 *    param - Profile parameter ID
 *    len - length of data to right
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
bStatus_t SendNotify_SetParameter( uint8_t param, uint16_t len, void *value )
{

    Log_info0("Notification sent sendnotify.c ");
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case SENDNOTIFY_NOTIFY_1_ID:
      if ( len == SENDNOTIFY_NOTIFY_1_LEN )
      {
        memcpy(sendNotify_Notify_1Val, value, len);

        // Try to send notification.
        GATTServApp_ProcessCharCfg( sendNotify_Notify_1Config, (uint8_t *)&sendNotify_Notify_1Val, FALSE,
                                    sendNotifyAttrTbl, GATT_NUM_ATTRS( sendNotifyAttrTbl ),
                                    INVALID_TASK_ID,  sendNotify_ReadAttrCB);
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
 * SendNotify_GetParameter - Get a SendNotify parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
bStatus_t SendNotify_GetParameter( uint8_t param, uint16_t *len, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case SENDNOTIFY_NOTIFY_1_ID:
      memcpy(value, sendNotify_Notify_1Val, SENDNOTIFY_NOTIFY_1_LEN);
      break;

    default:
      ret = INVALIDPARAMETER;
      break;
  }
  return ret;
}


/*********************************************************************
 * @fn          sendNotify_ReadAttrCB
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
static bStatus_t sendNotify_ReadAttrCB( uint16_t connHandle, gattAttribute_t *pAttr,
                                       uint8_t *pValue, uint16_t *pLen, uint16_t offset,
                                       uint16_t maxLen, uint8_t method )
{
  bStatus_t status = SUCCESS;

  // See if request is regarding the Notify_1 Characteristic Value
if ( ! memcmp(pAttr->type.uuid, sendNotify_Notify_1UUID, pAttr->type.len) )
  {
    if ( offset > SENDNOTIFY_NOTIFY_1_LEN )  // Prevent malicious ATT ReadBlob offsets.
    {
      status = ATT_ERR_INVALID_OFFSET;
    }
    else
    {
      *pLen = MIN(maxLen, SENDNOTIFY_NOTIFY_1_LEN - offset);  // Transmit as much as possible
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
 * @fn      sendNotify_WriteAttrCB
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
static bStatus_t sendNotify_WriteAttrCB( uint16_t connHandle, gattAttribute_t *pAttr,
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
  // See if request is regarding the Notify_1 Characteristic Value
  else if ( ! memcmp(pAttr->type.uuid, sendNotify_Notify_1UUID, pAttr->type.len) )
  {
    if ( offset + len > SENDNOTIFY_NOTIFY_1_LEN )
    {
      status = ATT_ERR_INVALID_OFFSET;
    }
    else
    {
      // Copy pValue into the variable we point to from the attribute table.
      memcpy(pAttr->pValue + offset, pValue, len);

      // Only notify application if entire expected value is written
      if ( offset + len == SENDNOTIFY_NOTIFY_1_LEN)
        paramID = SENDNOTIFY_NOTIFY_1_ID;
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


