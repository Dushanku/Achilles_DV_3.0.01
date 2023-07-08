/*
 * sendNotify.h
 *
 *  Created on: Jun 29, 2023
 *      Author: Dushan
 */

#ifndef PROFILES_SENDNOTIFY_H_
#define PROFILES_SENDNOTIFY_H_
/**********************************************************************************************
 * Filename:       sendNotify.h
 *
 * Description:    This file contains the sendNotify service definitions and
 *                 prototypes.
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


#ifndef _SENDNOTIFY_H_
#define _SENDNOTIFY_H_

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

// Service UUID
#define SENDNOTIFY_SERV_UUID 0x0001

//  Characteristic defines
#define SENDNOTIFY_NOTIFY_1_ID   0
#define SENDNOTIFY_NOTIFY_1_UUID 0x0002
#define SENDNOTIFY_NOTIFY_1_LEN  2

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
typedef void (*sendNotifyChange_t)(uint16_t connHandle, uint8_t paramID, uint16_t len, uint8_t *pValue);

typedef struct
{
  sendNotifyChange_t        pfnChangeCb;  // Called when characteristic value changes
  sendNotifyChange_t        pfnCfgChangeCb;
} sendNotifyCBs_t;



/*********************************************************************
 * API FUNCTIONS
 */


/*
 * SendNotify_AddService- Initializes the SendNotify service by registering
 *          GATT attributes with the GATT server.
 *
 */
extern bStatus_t SendNotify_AddService( uint8_t rspTaskId);

/*
 * SendNotify_RegisterAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
extern bStatus_t SendNotify_RegisterAppCBs( sendNotifyCBs_t *appCallbacks );

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
extern bStatus_t SendNotify_SetParameter(uint8_t param, uint16_t len, void *value);

/*
 * SendNotify_GetParameter - Get a SendNotify parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
extern bStatus_t SendNotify_GetParameter(uint8_t param, uint16_t *len, void *value);

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* _SENDNOTIFY_H_ */




#endif /* PROFILES_SENDNOTIFY_H_ */
