/*
 * observerNotify.h
 *
 *  Created on: Feb 1, 2023
 *      Author: Dushan
 */

#ifndef PROFILES_OBSERVERNOTIFY_H_
#define PROFILES_OBSERVERNOTIFY_H_

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
#define OBSERVERNOTIFY_SERV_UUID 0x0007

//  Characteristic defines
#define OBSERVERNOTIFY_NOTIFYCHRAC_ID   0
#define OBSERVERNOTIFY_NOTIFYCHRAC_UUID 0x0008
#define OBSERVERNOTIFY_NOTIFYCHRAC_LEN  12

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
typedef void (*observerNotifyChange_t)(uint16_t connHandle, uint8_t paramID, uint16_t len, uint8_t *pValue);

typedef struct
{
  observerNotifyChange_t        pfnChangeCb;  // Called when characteristic value changes
  observerNotifyChange_t        pfnCfgChangeCb;
} observerNotifyCBs_t;



/*********************************************************************
 * API FUNCTIONS
 */


/*
 * ObserverNotify_AddService- Initializes the ObserverNotify service by registering
 *          GATT attributes with the GATT server.
 *
 */
extern bStatus_t ObserverNotify_AddService( uint8_t rspTaskId);

/*
 * ObserverNotify_RegisterAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
extern bStatus_t ObserverNotify_RegisterAppCBs( observerNotifyCBs_t *appCallbacks );

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
extern bStatus_t ObserverNotify_SetParameter(uint8_t param, uint16_t len, void *value);

/*
 * ObserverNotify_GetParameter - Get a ObserverNotify parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
extern bStatus_t ObserverNotify_GetParameter(uint8_t param, uint16_t *len, void *value);

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif
#endif /* PROFILES_OBSERVERNOTIFY_H_ */
