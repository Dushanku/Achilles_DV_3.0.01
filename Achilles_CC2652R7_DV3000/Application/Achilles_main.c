/******************************************************************************

   @file  project_zero.c

   @brief This file contains the Project Zero sample application for use
        with the CC2650 Bluetooth Low Energy Protocol Stack.

   Group: WCS, BTS
   Target Device: cc13xx_cc26xx

 ******************************************************************************
   
 Copyright (c) 2015-2022, Texas Instruments Incorporated
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 *  Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

 *  Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 *  Neither the name of Texas Instruments Incorporated nor the names of
    its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 ******************************************************************************
   
   
 *****************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include <string.h>

#if (!(defined __TI_COMPILER_VERSION__) && !(defined __GNUC__))
#include <intrinsics.h>
#endif

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/drivers/utils/List.h>
#include <stdint.h>
//#include <xdc/runtime/Log.h> // Comment this in to use xdc.runtime.Log
#include <ti/common/cc26xx/uartlog/UartLog.h>  // Comment out if using xdc Log
#include <ti/drivers/ADC.h>
#include <ti/display/AnsiColor.h>
#include <stdbool.h>
#include <ti/drivers/dpl/SemaphoreP.h>
#include <ti/drivers/apps/LED.h>
#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(driverlib/sys_ctrl.h)
#include DeviceFamily_constructPath(inc/hw_prcm.h)
#include DeviceFamily_constructPath(driverlib/sys_ctrl.h)
#include DeviceFamily_constructPath(driverlib/pwr_ctrl.h)
#include <icall.h>
#include <bcomdef.h>
#include <ti/drivers/apps/Button.h>
/* This Header file contains all BLE API and icall structure definition */
#include <icall_ble_api.h>

/* Bluetooth Profiles */
#include <devinfoservice.h>
#include <profiles/project_zero/button_service.h>
//#include <profiles/project_zero/led_service.h>
#include <profiles/project_zero/data_service.h>
#include <profiles/oad/cc26xx/oad.h>
#include <observerNotify.h>
#include "pauseHeater.h"
#include "runHeater.h"
#include "stopHeater.h"
#include <ti/drivers/PWM.h>
#include <ti/drivers/I2C.h>
/* Includes needed for reverting to factory and erasing external flash */
#include <ti/common/cc26xx/oad/oad_image_header.h>
#include <ti/drivers/dpl/HwiP.h>
#include <ti/drivers/NVS.h>
#include <WS2812.h>
#include "scif.h"
#include DeviceFamily_constructPath(driverlib/flash.h)

#ifdef USE_RCOSC
#include <rcosc_calibration.h>
#endif //USE_RCOSC

/* Application specific includes */
#include <ti_drivers_config.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26X2.h>
#include <Achilles_main.h>
#include "ti_ble_config.h"
#include <util.h>
#include <ti/drivers/Watchdog.h>
/*********************************************************************
 * MACROS
 */

// Spin if the expression is not true
#define APP_ASSERT(expr) if(!(expr)) {project_zero_spin();}

#define UTIL_ARRTOHEX_REVERSE     1
#define UTIL_ARRTOHEX_NO_REVERSE  0

/*********************************************************************
 * CONSTANTS
 */
// Task configuration
#define PZ_TASK_PRIORITY                     2

#ifndef PZ_TASK_STACK_SIZE
#define PZ_TASK_STACK_SIZE                   4096
#endif


// Task configuration
#define sbcTaskS_PRIORITY                     1

#ifndef SBS_TASK_STACK_SIZE
#define SBS_TASK_STACK_SIZE                   1024
#endif
//
//#define sbcTaskS_PRIORITY         1
//
//#ifndef SBS_TASK_STACK_SIZE
//
//#if defined __TI_COMPILER_VERSION__
//#define SBS_TASK_STACK_SIZE       800 // multiples of 8 only
//#else  // IAR Compiler Used
//#define SBS_TASK_STACK_SIZE       800 // multiples of 8 only
//#endif // defined __TI_COMPILER_VERSION__

// Internal Events used by OAD profile
#define PZ_OAD_QUEUE_EVT                     OAD_QUEUE_EVT       // Event_Id_01
#define PZ_OAD_COMPLETE_EVT                  OAD_DL_COMPLETE_EVT // Event_Id_02

// Internal Events for RTOS application
#define PZ_ICALL_EVT                         ICALL_MSG_EVENT_ID  // Event_Id_31
#define PZ_APP_MSG_EVT                       Event_Id_30

// Bitwise OR of all RTOS events to pend on
#define PZ_ALL_EVENTS                        (PZ_ICALL_EVT | \
                                              PZ_APP_MSG_EVT | \
                                              PZ_OAD_QUEUE_EVT | \
                                              PZ_OAD_COMPLETE_EVT)

// Types of messages that can be sent to the user application task from other
// tasks or interrupts. Note: Messages from BLE Stack are sent differently.
#define PZ_SERVICE_WRITE_EVT     0  /* A characteristic value has been written     */
#define PZ_SERVICE_CFG_EVT       1  /* A characteristic configuration has changed  */
#define PZ_UPDATE_CHARVAL_EVT    2  /* Request from ourselves to update a value    */
#define PZ_BUTTON_DEBOUNCED_EVT  14  /* A button has been debounced with new value  */
#define PZ_PAIRSTATE_EVT         4  /* The pairing state is updated                */
#define PZ_PASSCODE_EVT          5  /* A pass-code/PIN is requested during pairing */
#define PZ_ADV_EVT               6  /* A subscribed advertisement activity         */
#define PZ_START_ADV_EVT         7  /* Request advertisement start from task ctx   */
#define PZ_SEND_PARAM_UPD_EVT    8  /* Request parameter update req be sent        */
#define PZ_CONN_EVT              9  /* Connection Event End notice                 */
#define PZ_READ_RPA_EVT         10  /* Read RPA event                              */
#define AC_NTFY_CLK_EVT         11  /* Notify Observers                            */
#define PZ_MSG_I2C_TIMER        12
#define PZ_MSG_BATRY_PRECENT    13
#define PZ_MSG_button_sense     3

// Supervision timeout conversion rate to miliseconds
#define CONN_TIMEOUT_MS_CONVERSION            10

// Connection interval conversion rate to miliseconds
#define CONN_INTERVAL_MS_CONVERSION           1.25


#define SAMPLE_TIME 1000      //ADC sample time in ms (when no of bits in sample = 15)
#define CONFIG_REG 0x01
#define ADC_CON 0x15
#define ADC_FUNC_DIS 0x16
#define VBAT_REG1 0x1D
#define VBAT_REG2 0x1E
#define VBUS_REG1 0x1B
#define VBUS_REG2 0x1C
#define IBUS_REG1 0x17
#define IBUS_REG2 0x18
#define CHRG_CURR_REG1 0x19
#define  CHRG_CURR_REG2 0x1A
#define CHARGER_STATUS_REG1 0x0B
#define CHARGER_CTRL1 0x05
#define BQ25887_ADDR 0x6A
#define TIMEOUT_MS 10000
static uint8_t slaveAddress = BQ25887_ADDR;
float charge=0;
float discharge=0;
int batPercentage = 0;

/*********************************************************************
 * TYPEDEFS
 */
// Struct for messages sent to the application task
typedef struct
{
    uint8_t event;
    void    *pData;
} pzMsg_t;

/* Semaphore used to gate for shutdown */
SemaphoreP_Struct semStruct;
SemaphoreP_Handle semHandle;

// Struct for messages about characteristic data
typedef struct
{
    uint16_t svcUUID; // UUID of the service
    uint16_t dataLen; //
    uint8_t paramID; // Index of the characteristic
    uint8_t data[]; // Flexible array member, extended to malloc - sizeof(.)
} pzCharacteristicData_t;

// Struct for message about sending/requesting passcode from peer.
typedef struct
{
    uint16_t connHandle;
    uint8_t uiInputs;
    uint8_t uiOutputs;
    uint32_t numComparison;
} pzPasscodeReq_t;

// Struct for message about a pending parameter update request.
typedef struct
{
    uint16_t connHandle;
} pzSendParamReq_t;

// Struct for message about button state
typedef struct
{
    uint_least8_t gpioId;
    uint8_t state;
} pzButtonState_t;

// Container to store passcode data when passing from gapbondmgr callback
// to app event. See the pfnPairStateCB_t documentation from the gapbondmgr.h
// header file for more information on each parameter.
typedef struct
{
    uint8_t state;
    uint16_t connHandle;
    uint8_t status;
} pzPairStateData_t;

// Container to store passcode data when passing from gapbondmgr callback
// to app event. See the pfnPasscodeCB_t documentation from the gapbondmgr.h
// header file for more information on each parameter.
typedef struct
{
    uint8_t deviceAddr[B_ADDR_LEN];
    uint16_t connHandle;
    uint8_t uiInputs;
    uint8_t uiOutputs;
    uint32_t numComparison;
} pzPasscodeData_t;

// Container to store advertising event data when passing from advertising
// callback to app event. See the respective event in GapAdvScan_Event_IDs
// in gap_advertiser.h for the type that pBuf should be cast to.
typedef struct
{
    uint32_t event;
    void *pBuf;
} pzGapAdvEventData_t;

// List element for parameter update and PHY command status lists
typedef struct
{
    List_Elem elem;
    uint16_t *connHandle;
} pzConnHandleEntry_t;

// Connected device information
typedef struct
{
    uint16_t connHandle;                    // Connection Handle
    Clock_Struct* pUpdateClock;             // pointer to clock struct
    bool phyCngRq;                          // Set to true if PHY change request is in progress
    uint8_t currPhy;                        // The active PHY for a connection
    uint8_t rqPhy;                          // The requested PHY for a connection
    uint8_t phyRqFailCnt;                   // PHY change request fail count
} pzConnRec_t;

// Container to store information from clock expiration using a flexible array
// since data is not always needed
typedef struct
{
  uint8_t event;
  uint8_t data[];
} pzClockEventData_t;



/*********************************************************************
 * GLOBAL VARIABLES
 */
// Task configuration
Task_Struct pzTask, task0;
#if defined __TI_COMPILER_VERSION__
#pragma DATA_ALIGN(appTaskStack, 8)
#elif defined(__GNUC__) || defined(__clang__)
__attribute__ ((aligned (8)))
#else
#pragma data_alignment=8
#endif
uint8_t appTaskStack[PZ_TASK_STACK_SIZE];
Char sbcTaskStack[SBS_TASK_STACK_SIZE];
/*********************************************************************
 * LOCAL VARIABLES
 */


uint8 Status[12];
//Status Reply Constants
uint16 pulsecount = 0;
uint8 chargingCompletionFlag = 0;
uint8 chargingStatus = 0;
uint8 buttonPresscount = 0;
uint8 BLEbuttonPress = 0;
uint8 manulaOnflag = 0;
uint8 buttonPresscountPW = 0;
uint8 pixelNitify = 0;
uint8 buttonPresssState = 0;
uint8 powerOnFlag = 0;
uint8 chargeStaetFlag = 0;
uint8 buttonID = 0;
uint8 Current_Status = 1;
uint8 manualCount = 0;
uint8 Current_Protocol = 0 ;
uint8 Set_Time_01 = 0 ;
uint16 Set_Time_02 = 0 ;
uint16 Remaining_Time_01 = 0;
uint16 Remaining_Time_02 = 0;
float Current_Temperature_01 = 0;
uint8 setPointTemp = 0;
float Current_Temperature_02 = 0;
uint8 Error_Code = 4;
uint8 Error_level = 0;
uint8 Error_Ranking = 0;
uint8 Device_Cali_Status = 0;
int Status_CRC = 0;
int  achiliesTaskID=0;
int heatLevelIndication=0;
uint8 batteryLevelApp = 0;
uint8 batteryLowFLag30 = 0;
uint8 batteryLowFLag15 = 0;
uint8 batteryLowFLag5 = 0;
uint8 errorFLag = 0;
uint8 batteryLevel = 0;
float vBat = 0;
float vBatRead = 0;
//runheater
pzCharacteristicData_t *heater01;
uint8 runheatr[6];
uint8 runheatr_test[6] = {01, 80,1,45,01,01};
int countTick = 0;
int clock_cunt2 = 0;
int timecount = 0;
//Run Heater Constnts
uint8 protclIndx ;
uint8 heatLvl    ;
uint8 time1       ;
uint8 time2       ;
uint8 IRLgt      ;
uint8 CRC        ;
uint8 heatMax    ;
int stopClockTime  ;
int mannual_pro_time;
int clock_cunt = 0;
//Stop heater constants
//uint8_t StopFlag = 0;
int StopVal = 0;
int stopFlag=0;
int pauseFlag=0;
//mannual operation constants
int mannualOperationStop=0;
//Pause Meter Parameters
uint8_t PauseFlag = 0 ;
//status of device
int Current_status=0;
int RunType=0;
//PWM constatnts
uint32   dutyValue1;
uint32   dutyValue2;
uint32   dutyValue3;
int   PWMratio1 = 60;
int constantPWM = 60;

PWM_Handle PWM1 = NULL;;
uint8 bleflag = 0;
//I2C constants
int8       regValue ;
uint8         regValue1;
int8        regValue2;

uint8        txBuffer[2];
uint8        rxBuffer[2];

uint8        txBufferBms[2];
uint8        rxBufferBms[2];

uint8        txBufferTMT117[2];
uint8        rxBufferTMT117[2];

uint16   txBufferTMT117_2[2];
uint16   rxBufferTMT117_2[2];


uint8        txBufferMSP[2];
uint8        rxBufferMSP[2];

#define UNLOCK_REG 0x04
#define CONFIG_REG 0x01
#define EEPROM2_REG 0x06


//define sampling time
uint16_t sample_t = 2000; //sampling time in ms

//#define HeaterCurrent  0

int pre;
uint8 tempzerocount = 0;
uint8 tmp116errorcount = 0;
float Pretemperature = 0;
float temperature=0;
int msp430readValue=0;
int mannualProtocolSelect=0;
int mannulTempSelect=0;
int leastSig,mostSig;
float  batLevADC=0;
float batLev=0;
uint8_t batCount = 0;
float soc0 = 0.00;
I2C_Handle      i2c;
I2C_Params      i2cParams;
I2C_Transaction i2cTransaction;
Watchdog_Handle watchdogHandle;
float dutyCycle = 0;
//ADC_Params params;
//ADC_Handle adcHandle;
//Calibrating Parameters
uint8_t CalibrateIndex ;
uint8_t MIN  ;
uint8_t MAX  ;
uint8_t CalibrateID;
uint8_t CalibrateTemp;
uint8_t I2CStatusTMP116 = 0;
uint8_t I2CStatusBQ25887 = 0;
//Protocol Contants
uint8_t protocol4Count = 1;
int protocol_cunt = 0;
uint16_t heaterAdc = 0;
//Manual Operation Parameters
uint8_t operationIndx ;
uint8_t LEDFlag = 0 ;
uint8_t heatTem = 0  ;
//PID FUnction
float Initial_temp1=0;
float Initial_temp2=0;
float temp1=0;
float temp2=0;
int initialBodyTemp = 0;
int initialWiretemp = 0;
uint16_t initialBP_ADC = 0;
uint8_t batteryLevelPatern=0;
uint16_t heaterOkAdc = 0;

int startCount=0;
int continueFlag=0;
int protocolLock=0;
int protocol_select=0;
//int buttonPannelState=0;
int current_running_state=0;
int pre_pow_state=0;

//fail_safe_fet_on
bool fail_safe_fet_on=true;
bool sleepIndicateflag=true;
// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Event globally used to post local events and pend on system and
// local events.
static ICall_SyncHandle syncEvent;

// Queue object used for app messages
static Queue_Struct appMsgQueue;
static Queue_Handle appMsgQueueHandle;

// Advertising handles
static uint8_t advHandleLegacy;

// Per-handle connection info
static pzConnRec_t connList[MAX_NUM_BLE_CONNS];

// List to store connection handles for set phy command status's
static List_List setPhyCommStatList;

// List to store connection handles for queued param updates
static List_List paramUpdateList;

// Clock objects for debouncing the buttons
static Clock_Struct button0DebounceClock;
static Clock_Struct button1DebounceClock;
static Clock_Handle button0DebounceClockHandle;
static Clock_Handle button1DebounceClockHandle;

// Clock instance for RPA read events.
static Clock_Struct clkRpaRead;
static Clock_Struct Notifyclock;
static Clock_Struct commondelayclock;
//Clock for stop heater
static Clock_Struct stopClock;
static Clock_Struct protocolStartHeatingClock;
static Clock_Struct i2cCommunicateClock;
static Clock_Struct buttonsenseClock;
static Clock_Struct mannualOperationClock;
static Clock_Struct batteryPrecentageClock;
// State of the buttons
static uint8_t button0State = 0;
static uint8_t button1State = 0;

// Variable used to store the number of messages pending once OAD completes
// The application cannot reboot until all pending messages are sent
static uint8_t numPendingMsgs = 0;
static bool oadWaitReboot = false;

// Flag to be stored in NV that tracks whether service changed
// indications needs to be sent out
static uint32_t sendSvcChngdOnNextBoot = FALSE;

// Address mode
static GAP_Addr_Modes_t addrMode = DEFAULT_ADDRESS_MODE;

// Current Random Private Address
static uint8 rpaAddr[B_ADDR_LEN] = {0};

/*********************************************************************
 * LOCAL FUNCTIONS
 */

/* Task functions */
static void AchillesPB_init(void);
static void AchillesPB_taskFxn(UArg a0,
                                UArg a1);

/* Event message processing functions */
static void AchillesPB_processStackEvent(uint32_t stack_event);
static void AchillesPB_processApplicationMessage(pzMsg_t *pMsg);
static uint8_t AchillesPB_processGATTMsg(gattMsgEvent_t *pMsg);
static void AchillesPB_processGapMessage(gapEventHdr_t *pMsg);
static void AchillesPB_processHCIMsg(ICall_HciExtEvt *pMsg);
static void AchillesPB_processPairState(pzPairStateData_t *pPairState);
static void AchillesPB_processPasscode(pzPasscodeReq_t *pReq);
static void AchillesPB_processCmdCompleteEvt(hciEvt_CmdComplete_t *pMsg);
static void AchillesPB_processAdvEvent(pzGapAdvEventData_t *pEventData);

/* Profile value change handlers */
static void AchillesPB_updateCharVal(pzCharacteristicData_t *pCharData);
static void AchillesPB_LedService_ValueChangeHandler(
    pzCharacteristicData_t *pCharData);
static void AchillesPB_ButtonService_CfgChangeHandler(
    pzCharacteristicData_t *pCharData);
static void AchillesPB_DataService_ValueChangeHandler(
    pzCharacteristicData_t *pCharData);
static void AchillesPB_DataService_CfgChangeHandler(
    pzCharacteristicData_t *pCharData);
static void user_observerNotify_ValueChangeHandler(
    pzCharacteristicData_t *pCharData);
static void user_runHeater_ValueChangeHandler(
                            pzCharacteristicData_t *pCharData
                        );
static void user_pauseHeater_ValueChangeHandler(
                            pzCharacteristicData_t *pCharData
                        );
static void user_stopHeater_ValueChangeHandler(
                            pzCharacteristicData_t *pCharData
                        );


/* Stack or profile callback function */
static void AchillesPB_advCallback(uint32_t event,
                                    void *pBuf,
                                    uintptr_t arg);
static void AchillesPB_passcodeCb(uint8_t *pDeviceAddr,
                                   uint16_t connHandle,
                                   uint8_t uiInputs,
                                   uint8_t uiOutputs,
                                   uint32_t numComparison);
static void AchillesPB_pairStateCb(uint16_t connHandle,
                                    uint8_t state,
                                    uint8_t status);

static void AchillesPB_LedService_ValueChangeCB(uint16_t connHandle,
                                                 uint8_t paramID,
                                                 uint16_t len,
                                                 uint8_t *pValue);
static void AchillesPB_DataService_ValueChangeCB(uint16_t connHandle,
                                                  uint8_t paramID,
                                                  uint16_t len,
                                                  uint8_t *pValue);
static void AchillesPB_ButtonService_CfgChangeCB(uint16_t connHandle,
                                                  uint8_t paramID,
                                                  uint16_t len,
                                                  uint8_t *pValue);
static void AchillesPB_DataService_CfgChangeCB(uint16_t connHandle,
                                                uint8_t paramID,
                                                uint16_t len,
                                                uint8_t *pValue);
static void user_observerNotify_ValueChangeCB(uint16_t connHandle,
                                                  uint8_t paramID,
                                                  uint16_t len,
                                                  uint8_t *pValue);
static void user_runHeater_ValueChangeCB(uint16_t connHandle, uint8_t paramID, uint16_t len, uint8_t *pValue);
static void user_pauseHeater_ValueChangeCB(uint16_t connHandle, uint8_t paramID, uint16_t len, uint8_t *pValue);
static void user_stopHeater_ValueChangeCB(uint16_t connHandle, uint8_t paramID, uint16_t len, uint8_t *pValue);


/* Connection handling functions */
static uint8_t AchillesPB_getConnIndex(uint16_t connHandle);
static uint8_t AchillesPB_clearConnListEntry(uint16_t connHandle);
static uint8_t AchillesPB_addConn(uint16_t connHandle);
static uint8_t AchillesPB_removeConn(uint16_t connHandle);
static void AchillesPB_updatePHYStat(uint16_t eventCode,
                                      uint8_t *pMsg);
static void AchillesPB_handleUpdateLinkParamReq(
    gapUpdateLinkParamReqEvent_t *pReq);
static void AchillesPB_sendParamUpdate(uint16_t connHandle);
static void AchillesPB_handleUpdateLinkEvent(gapLinkUpdateEvent_t *pEvt);
#ifdef DEFAULT_SEND_PARAM_UPDATE_REQ
static void AchillesPB_paramUpdClockHandler(UArg arg);
#endif
static void AchillesPB_clockHandler(UArg arg);
static void AchillesPB_processConnEvt(Gap_ConnEventRpt_t *pReport);
static void AchillesPB_connEvtCB(Gap_ConnEventRpt_t *pReport);

/* Button handling functions */
static void buttonDebounceSwiFxn(UArg buttonId);
static void notifyclockSwiFxn(UArg arg);
static void commondelayclockwiFxn(UArg arg);
static void GPIO_Board_keyCallback(uint_least8_t index);
static void AchillesPB_handleButtonPress(pzButtonState_t *pState);
void watchdogCallback(uintptr_t watchdogHandle);
/* Utility functions */
static status_t AchillesPB_enqueueMsg(uint8_t event,
                                   void *pData);
static char * util_arrtohex(uint8_t const *src,
                            uint8_t src_len,
                            uint8_t       *dst,
                            uint8_t dst_len,
                            uint8_t reverse);
static char * util_getLocalNameStr(const uint8_t *advData, uint8_t len);
static void AchillesPB_processOadWriteCB(uint8_t event,
                                          uint16_t arg);
static void AchillesPB_processL2CAPMsg(l2capSignalEvent_t *pMsg);
static void AchillesPB_checkSvcChgndFlag(uint32_t flag);
static void AchillesPB_bootManagerCheck(uint_least8_t revertIo,
                                         uint_least8_t eraseIo);
/*********************************************************************
 * Custom FUNCTIONS
 */

static void notifySetdata();
static void initialCheck();
static void runheater(uint8 runheatr[6]);
static void stopClockSwiFxn(UArg p);
static void setRemainingTime();
static void I2C_read(uint8_t reg, uint8_t regValue);
static float I2CTMT117(uint8 slave);
static void TMP117init(uint8_t state, uint8_t slave);
static int BQ25887();
float SOC_OCV(I2C_Handle i2c, I2C_Transaction i2cTransaction,uint8_t *txBuffer, uint8_t *rxBuffer);
float SOC(uint16_t sample_t, I2C_Handle i2c, I2C_Transaction i2cTransaction,uint8_t *txBuffer, uint8_t *rxBuffer);
float DOD(uint16_t sample_t, I2C_Handle i2c, I2C_Transaction i2cTransaction,uint8_t *txBuffer, uint8_t *rxBuffer);
static void stateCreateTaskFunction(UArg arg0, UArg arg1);
static void achiliesProtocolSelectTask();
static void protocol_1(uint8_t heatTem);
static void protocol_2(uint8_t heatTem);
static void protocol_3(uint8_t heatTem);
static void HeaterPID(uint16_t maxDuty, uint16_t maxHeaterTemp);
static void stateCreateTaskFunction(UArg arg0, UArg arg1);
static void PID();
static void protocolStartClockSwiFxn ();
static void i2cCommClockSwiFxn(UArg buttonId);
static void mannualOperationClockSwiFxn(UArg q);
static void batteryIndication(uint8_t type);
static void achilliesInit(uint8_t type);
static void OPENpwm(float PWMratio1, float PWMratio2);
static void onLEDs(uint8_t pattern, uint8_t arg_u8_red, uint8_t arg_u8_green, uint8_t arg_u8_blue );
static void prsButtondatSWI();
static void prsButtondata();
static void delayFun (int delayTimeInMs);
void scCtrlReadyCallback(void);
void scTaskAlertCallback(void);
void setThermalReg(I2C_Handle i2c, I2C_Transaction i2cTransaction,uint8_t *txBuffer, uint8_t *rxBuffer);
/*********************************************************************
 * EXTERN FUNCTIONS
 */
extern void AssertHandler(uint8_t assertCause,
                          uint8_t assertSubcause);

/*********************************************************************
 * PROFILE CALLBACKS
 */
// GAP Bond Manager Callbacks
static gapBondCBs_t AchillesPB_BondMgrCBs =
{
    AchillesPB_passcodeCb,     // Passcode callback
    AchillesPB_pairStateCb     // Pairing/Bonding state Callback
};

/*
 * Callbacks in the user application for events originating from BLE services.
 */
// LED Service callback handler.
// The type LED_ServiceCBs_t is defined in led_service.h


// Button Service callback handler.
// The type Button_ServiceCBs_t is defined in button_service.h
static ButtonServiceCBs_t AchillesPB_Button_ServiceCBs =
{
    .pfnChangeCb = NULL,  // No writable chars in Button Service, so no change handler.
    .pfnCfgChangeCb = AchillesPB_ButtonService_CfgChangeCB, // Noti/ind configuration callback handler
};

// Data Service callback handler.
// The type Data_ServiceCBs_t is defined in data_service.h
static DataServiceCBs_t AchillesPB_Data_ServiceCBs =
{
    .pfnChangeCb = AchillesPB_DataService_ValueChangeCB,  // Characteristic value change callback handler
    .pfnCfgChangeCb = AchillesPB_DataService_CfgChangeCB, // Noti/ind configuration callback handler
};

// OAD Service callback handler.
// The type oadTargetCBs_t is defined in oad.h
static oadTargetCBs_t AchillesPB_oadCBs =
{
    .pfnOadWrite = AchillesPB_processOadWriteCB // Write Callback.
};

// Service callback function implementation
// ObserverNotify callback handler. The type observerNotifyCBs_t is defined in observerNotify.h
static observerNotifyCBs_t user_observerNotifyCBs =
{
  .pfnChangeCb = user_observerNotify_ValueChangeCB, // Characteristic value change callback handler
  .pfnCfgChangeCb = NULL, // No CCCD change handler implemented
};

static runHeaterCBs_t user_runHeaterCBs =
{
  .pfnChangeCb = user_runHeater_ValueChangeCB,
  .pfnCfgChangeCb = NULL,
};

static pauseHeaterCBs_t user_pauseHeaterCBs =
{
  .pfnChangeCb = user_pauseHeater_ValueChangeCB,
  .pfnCfgChangeCb = NULL,
};

static stopHeaterCBs_t user_stopHeaterCBs =
{
  .pfnChangeCb = user_stopHeater_ValueChangeCB,
  .pfnCfgChangeCb = NULL,
};




/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn     project_zero_spin
 *
 * @brief   Spin forever
 */
static void project_zero_spin(void)
{
  volatile uint8_t x = 0;;

  while(1)
  {
    x++;
  }
}

/*********************************************************************
 * @fn      AchillesPB_createTask
 *
 * @brief   Task creation function for the Project Zero.
 */
void AchillesPB_createTask(void)
{
    Task_Params taskParams;

    // Configure task
    Task_Params_init(&taskParams);
    taskParams.stack = appTaskStack;
    taskParams.stackSize = PZ_TASK_STACK_SIZE;
    taskParams.priority = PZ_TASK_PRIORITY;

    Task_construct(&pzTask, AchillesPB_taskFxn, &taskParams, NULL);
}

/*********************************************************************
 * @fn      AchillesPB_init
 *
 * @brief   Called during initialization and contains application
 *          specific initialization (ie. hardware initialization/setup,
 *          table initialization, power up notification, etc), and
 *          profile initialization/setup.
 */
static void AchillesPB_init(void)
{
    // ******************************************************************
    // NO STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
    // ******************************************************************
    // Register the current thread as an ICall dispatcher application
    // so that the application can send and receive messages.
    ICall_registerApp(&selfEntity, &syncEvent);

#ifdef USE_RCOSC
    // Set device's Sleep Clock Accuracy
#if ( HOST_CONFIG & ( CENTRAL_CFG | PERIPHERAL_CFG ) )
    HCI_EXT_SetSCACmd(500);
#endif // (CENTRAL_CFG | PERIPHERAL_CFG)
    RCOSC_enableCalibration();
#endif // USE_RCOSC

    // Initialize queue for application messages.
    // Note: Used to transfer control to application thread from e.g. interrupts.
    Queue_construct(&appMsgQueue, NULL);
    appMsgQueueHandle = Queue_handle(&appMsgQueue);

    // ******************************************************************
    // Hardware initialization
    // ******************************************************************
    // Setup callback for button gpio
    GPIO_setCallback(CONFIG_GPIO_BTN1, GPIO_Board_keyCallback);
    GPIO_setCallback(CONFIG_GPIO_BTN2, GPIO_Board_keyCallback);
    // Enable interrupt
    GPIO_enableInt(CONFIG_GPIO_BTN1);
    GPIO_enableInt(CONFIG_GPIO_BTN2);
    // Create the debounce clock objects for Button 0 and Button 1
    button0DebounceClockHandle = Util_constructClock(&button0DebounceClock,
                                                     buttonDebounceSwiFxn, 50,
                                                     0,
                                                     0,
                                                     CONFIG_GPIO_BTN1);
    button1DebounceClockHandle = Util_constructClock(&button1DebounceClock,
                                                     buttonDebounceSwiFxn, 50,
                                                     0,
                                                     0,
                                                     CONFIG_GPIO_BTN2);
//    Util_constructClock(&Notifyclock, notifyclockSwiFxn,
//                                  1000, 0, true, AC_NTFY_CLK_EVT);

    //Observer notify clock
    Clock_Params notifyClockprms;
    Clock_Params_init(&notifyClockprms);
    notifyClockprms.period = 1000*(1000/Clock_tickPeriod);
    Clock_construct(&Notifyclock,notifyclockSwiFxn,0,&notifyClockprms);
    //Common delay clock
    Clock_Params commondelayClockprms;
    Clock_Params_init(&commondelayClockprms);
    notifyClockprms.period = 1000*(1000/Clock_tickPeriod);
    Clock_construct(&commondelayclock,commondelayclockwiFxn,0,&commondelayClockprms);

    //Heater Stop Clock
   Clock_Params stopClockprms;
   Clock_Params_init(&stopClockprms);
   //GPIO_init();
   OPENpwm(45,75);
   WS2812_begin();
   //stopClockprms.period = 20000*(1000/Clock_tickPeriod);
   Clock_construct(&stopClock,stopClockSwiFxn,0,&stopClockprms);

   //protocolStartHeatingClock
   Clock_Params protocolStartHeatingClockParams;
          // Insert default params
   Clock_Params_init(&protocolStartHeatingClockParams);
          // Set a period, so it times out periodically without jitter
   protocolStartHeatingClockParams.period = 30000 * (1000/Clock_tickPeriod),
   Clock_construct(&protocolStartHeatingClock,  protocolStartClockSwiFxn,
                   0, // Initial delay before first timeout
                   &protocolStartHeatingClockParams);
   Clock_start(Clock_handle(&protocolStartHeatingClock));

   //i2cCommunicateClock
   Clock_Params i2cCommunicateClockParams;
           // Insert default params
    Clock_Params_init(&i2cCommunicateClockParams);
           // Set a period, so it times out periodically without jitter
    i2cCommunicateClockParams.period = 1400 * (1000/Clock_tickPeriod),
    Clock_construct(&i2cCommunicateClock, i2cCommClockSwiFxn,
                    0, // Initial delay before first timeout
                    &i2cCommunicateClockParams);
    Clock_start(Clock_handle(&i2cCommunicateClock));


    //Button sense clock
      Clock_Params nuttonsense;
              // Insert default params
       Clock_Params_init(&nuttonsense);
              // Set a period, so it times out periodically without jitter
       nuttonsense.period = 100 * (1000/Clock_tickPeriod),
       Clock_construct(&buttonsenseClock, prsButtondatSWI ,
                       0, // Initial delay before first timeout
                       &nuttonsense);
       Clock_start(Clock_handle(&buttonsenseClock));

    // Set the Device Name characteristic in the GAP GATT Service
    // For more information, see the section in the User's Guide:
    // http://software-dl.ti.com/lprf/ble5stack-latest/
    GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName);

    // Configure GAP for param update
    {
        uint16_t paramUpdateDecision = DEFAULT_PARAM_UPDATE_REQ_DECISION;

        // Pass all parameter update requests to the app for it to decide
        GAP_SetParamValue(GAP_PARAM_LINK_UPDATE_DECISION, paramUpdateDecision);
    }

    // Setup the GAP Bond Manager. For more information see the GAP Bond Manager
    // section in the User's Guide
    setBondManagerParameters();

    // ******************************************************************
    // BLE Service initialization
    // ******************************************************************
    GGS_AddService(GAP_SERVICE);               // GAP GATT Service
    GATTServApp_AddService(GATT_ALL_SERVICES); // GATT Service
    DevInfo_AddService();                      // Device Information Service

    // Add services to GATT server and give ID of this task for Indication acks.

    ButtonService_AddService(selfEntity);
    DataService_AddService(selfEntity);
    ObserverNotify_AddService(selfEntity);
    RunHeater_AddService(selfEntity);
    PauseHeater_AddService(selfEntity);
    StopHeater_AddService(selfEntity);



    // Open the OAD module and add the OAD service to the application
    if(OAD_SUCCESS != OAD_open(OAD_DEFAULT_INACTIVITY_TIME))
    {
        Log_error0("OAD failed to open");
    }
    else
    {
        // Resiter the OAD callback with the application
        OAD_register(&AchillesPB_oadCBs);
        Log_info0("Registered OAD Service");
    }

    // Check button state on reset to do ExtFlash erase or revert to factory.
    // We do this after the OAD init because if the external flash is empty
    // it will copy the current image into the factory image slot in external
    // flash.
    AchillesPB_bootManagerCheck(CONFIG_GPIO_BTN1,
                                 CONFIG_GPIO_BTN2);

    // Capture the current OAD version and log it
    static uint8_t versionStr[OAD_SW_VER_LEN + 1];
    OAD_getSWVersion(versionStr, OAD_SW_VER_LEN);

    // Add in Null terminator
    versionStr[OAD_SW_VER_LEN] = 0;

    // Display Image version
    Log_info1("OAD Image v%s", (uintptr_t)versionStr);

    // Register callbacks with the generated services that
    // can generate events (writes received) to the application

    ButtonService_RegisterAppCBs(&AchillesPB_Button_ServiceCBs);
    DataService_RegisterAppCBs(&AchillesPB_Data_ServiceCBs);
    ObserverNotify_RegisterAppCBs(&user_observerNotifyCBs);
    RunHeater_RegisterAppCBs(&user_runHeaterCBs);
    PauseHeater_RegisterAppCBs(&user_pauseHeaterCBs);
    StopHeater_RegisterAppCBs(&user_stopHeaterCBs);

    // Placeholder variable for characteristic intialization
    uint8_t initVal[40] = {0};
    uint8_t initString[] = "This is a pretty long string, isn't it!";



    // Initalization of characteristics in Button_Service that can provide data.
    ButtonService_SetParameter(BS_BUTTON0_ID, BS_BUTTON0_LEN, initVal);
    ButtonService_SetParameter(BS_BUTTON1_ID, BS_BUTTON1_LEN, initVal);

    // Initalization of characteristics in Data_Service that can provide data.
    DataService_SetParameter(DS_STRING_ID, sizeof(initString), initString);
    DataService_SetParameter(DS_STREAM_ID, DS_STREAM_LEN, initVal);

    uint8_t observerNotify_notifyChrac_initVal[OBSERVERNOTIFY_NOTIFYCHRAC_LEN] = {0};
    ObserverNotify_SetParameter(OBSERVERNOTIFY_NOTIFYCHRAC_ID, OBSERVERNOTIFY_NOTIFYCHRAC_LEN, observerNotify_notifyChrac_initVal);

    uint8_t runHeater_runHeater_initVal[RUNHEATER_RUNHEATER_LEN] = {0};
    RunHeater_SetParameter(RUNHEATER_RUNHEATER_ID, RUNHEATER_RUNHEATER_LEN, runHeater_runHeater_initVal);

    uint8_t pauseHeater_pauseHeater_initVal[PAUSEHEATER_PAUSEHEATER_LEN] = {0};
    PauseHeater_SetParameter(PAUSEHEATER_PAUSEHEATER_ID, PAUSEHEATER_PAUSEHEATER_LEN, pauseHeater_pauseHeater_initVal);

    uint8_t stopHeater_stopHeater_initVal[STOPHEATER_STOPHEATER_LEN] = {0};
    StopHeater_SetParameter(STOPHEATER_STOPHEATER_ID, STOPHEATER_STOPHEATER_LEN, stopHeater_stopHeater_initVal);


    // Start Bond Manager and register callback
    VOID GAPBondMgr_Register(&AchillesPB_BondMgrCBs);

    // Register with GAP for HCI/Host messages. This is needed to receive HCI
    // events. For more information, see the HCI section in the User's Guide:
    // http://software-dl.ti.com/lprf/ble5stack-latest/
    GAP_RegisterForMsgs(selfEntity);

    // Register for GATT local events and ATT Responses pending for transmission
    GATT_RegisterForMsgs(selfEntity);

    // Set default values for Data Length Extension
    // Extended Data Length Feature is already enabled by default
    {
      // Set initial values to maximum, RX is set to max. by default(251 octets, 2120us)
      // Some brand smartphone is essentially needing 251/2120, so we set them here.
      #define APP_SUGGESTED_PDU_SIZE 251 //default is 27 octets(TX)
      #define APP_SUGGESTED_TX_TIME 2120 //default is 328us(TX)

      // This API is documented in hci.h
      // See the LE Data Length Extension section in the BLE5-Stack User's Guide for information on using this command:
      // http://software-dl.ti.com/lprf/ble5stack-latest/
      HCI_LE_WriteSuggestedDefaultDataLenCmd(APP_SUGGESTED_PDU_SIZE, APP_SUGGESTED_TX_TIME);
    }

    // Initialize GATT Client, used by GAPBondMgr to look for RPAO characteristic for network privacy
    GATT_InitClient();

    // Initialize Connection List
    AchillesPB_clearConnListEntry(LINKDB_CONNHANDLE_ALL);

    //Initialize GAP layer for Peripheral role and register to receive GAP events
    GAP_DeviceInit(GAP_PROFILE_PERIPHERAL, selfEntity, addrMode, &pRandomAddress);

    // Process the Service changed flag
    AchillesPB_checkSvcChgndFlag(sendSvcChngdOnNextBoot);

    scifOsalInit ();
    scifInit (&scifDriverSetup);
    uint32_t rtcHz = 10;      // 10Hz RTC
    scifStartRtcTicksNow (0x00010000 / rtcHz);
    scifStartTasksNbl(1 << SCIF_ACH_BUTTON_TASK_ID);
    scifOsalRegisterCtrlReadyCallback(scCtrlReadyCallback);
    scifOsalRegisterTaskAlertCallback(scTaskAlertCallback);
    scifClearAlertIntSource();

    GPIO_setConfig(irOutput, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(bukEnable, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_LED_1_GPIO, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_LED_2_GPIO, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_write(bukEnable, 1); //Turn on 5V buck
//    ADC_init();
//    ADC_Params_init(&params);
//    params.isProtected = true;
//    adcHandle = ADC_open(HeaterCurrent, &params);


   Watchdog_Params params;
   uint32_t reloadValue;
   Watchdog_init();
   Watchdog_Params_init(&params);
   params.callbackFxn    = (Watchdog_Callback)watchdogCallback;
   params.debugStallMode = Watchdog_DEBUG_STALL_ON;
   params.resetMode      = Watchdog_RESET_ON;
   watchdogHandle = Watchdog_open(CONFIG_WATCHDOG_0, &params);

   if (watchdogHandle == NULL)
       {
           /* Error opening Watchdog */
       Error_Code = 15;
       errorFLag = 0;
       Current_Status = 5;
          // while (1) {}
       }
   reloadValue = Watchdog_convertMsToTicks(watchdogHandle, TIMEOUT_MS);
       if (reloadValue != 0)
       {
           Watchdog_setReload(watchdogHandle, reloadValue);
       }

}

// SCIF driver callback: Task control interface ready (non-blocking task control operation completed)
void scCtrlReadyCallback(void) {

}

// SCIF driver callback: Sensor Controller task code has generated an alert interrupt
void scTaskAlertCallback(void) {
    // Clear the ALERT interrupt source
    scifClearAlertIntSource();
 // Power_disablePolicy();
    GPIO_write(bukEnable, 1); //Turn On 5V buck
    Clock_start(Clock_handle(&buttonsenseClock));
    // Acknowledge the ALERT event
    scifAckAlertEvents();
}



void watchdogCallback(uintptr_t watchdogHandle)
{
    /*
     * If the Watchdog Non-Maskable Interrupt (NMI) is called,
     * loop until the device resets. Some devices will invoke
     * this callback upon watchdog expiration while others will
     * reset. See the device specific watchdog driver documentation
     * for your device.
     */
    while (1) {}
}

/*********************************************************************
 * @fn      AchillesPB_taskFxn
 *
 * @brief   Application task entry point for the Project Zero.
 *
 * @param   a0, a1 - not used.
 */
static void AchillesPB_taskFxn(UArg a0, UArg a1)
{
    // Initialize application
    PowerCC26X2_ResetReason resetReason = PowerCC26X2_getResetReason();
    if (resetReason == PowerCC26X2_RESET_SHUTDOWN_IO)
     {
         /* Application code must always disable the IO latches when coming out of shutdown */
         PowerCC26X2_releaseLatches();
         //Log_info1("disable the IO latches");
     }

    GPIO_init();
    GPIO_write(bukEnable, 1);
    Task_sleep (100 * (1000 / Clock_tickPeriod));
    AchillesPB_init();
    Clock_start(Clock_handle(&Notifyclock));
    batteryLevelApp = BQ25887();
    //Power_init();

    /* If we are waking up from shutdown, we do something extra. */
    /* Application code must always disable the IO latches when coming out of shutdown */

    int16_t status = Power_getTransitionState();
    Log_info1("status: %d", status);

    WS2812_setPixelColor(0, 0,0, 0);
    WS2812_setPixelColor(1, 0,0, 0);
    WS2812_setPixelColor(2, 0,0, 0);
    WS2812_show();

    for(;; )
    {
        uint32_t events;

        // Waits for an event to be posted associated with the calling thread.
        // Note that an event associated with a thread is posted when a
        // message is queued to the message receive queue of the thread
        events = Event_pend(syncEvent, Event_Id_NONE, PZ_ALL_EVENTS,
                            ICALL_TIMEOUT_FOREVER);

        if(events)
        {
            ICall_EntityID dest;
            ICall_ServiceEnum src;
            ICall_HciExtEvt *pMsg = NULL;

            // Fetch any available messages that might have been sent from the stack
            if(ICall_fetchServiceMsg(&src, &dest,
                                     (void **)&pMsg) == ICALL_ERRNO_SUCCESS)
            {
                uint8_t safeToDealloc = TRUE;

                if((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity))
                {
                    ICall_Stack_Event *pEvt = (ICall_Stack_Event *)pMsg;

                    // Check for BLE stack events first
                    if(pEvt->signature == 0xffff)
                    {
                        // Process stack events
                        AchillesPB_processStackEvent(pEvt->event_flag);
                    }
                    else
                    {
                        switch(pMsg->hdr.event)
                        {
                        case GAP_MSG_EVENT:
                            // Process GAP message
                            AchillesPB_processGapMessage((gapEventHdr_t*) pMsg);
                            break;

                        case GATT_MSG_EVENT:
                            // Process GATT message
                            safeToDealloc =
                                AchillesPB_processGATTMsg(
                                    (gattMsgEvent_t *)pMsg);
                            break;

                        case HCI_GAP_EVENT_EVENT:
                            AchillesPB_processHCIMsg(pMsg);
                            break;

                        case L2CAP_SIGNAL_EVENT:
                            // Process L2CAP free buffer notification
                            AchillesPB_processL2CAPMsg(
                                (l2capSignalEvent_t *)pMsg);
                            break;

                        default:
                            // do nothing
                            break;
                        }
                    }
                }

                if(pMsg && safeToDealloc)
                {
                    ICall_freeMsg(pMsg);
                }
            }

            // Process messages sent from another task or another context.
            while(!Queue_empty(appMsgQueueHandle))
            {
                pzMsg_t *pMsg = (pzMsg_t *)Util_dequeueMsg(appMsgQueueHandle);
                if(pMsg)
                {
                    // Process application-layer message probably sent from ourselves.
                    AchillesPB_processApplicationMessage(pMsg);
                    // Free the received message.
                    ICall_free(pMsg);
                }
            }

            // OAD events
            if(events & PZ_OAD_QUEUE_EVT)
            {
                // Process the OAD Message Queue
                uint8_t status = OAD_processQueue();

                // If the OAD state machine encountered an error, print it
                // Return codes can be found in oad_constants.h
                if(status == OAD_DL_COMPLETE)
                {
                    Log_info0("OAD DL Complete, wait for enable");
                }
                else if(status == OAD_IMG_ID_TIMEOUT)
                {
                    Log_info0("ImgID Timeout, disconnecting");

                    // This may be an attack, terminate the link,
                    // Note HCI_DISCONNECT_REMOTE_USER_TERM seems to most closet reason for
                    // termination at this state
                    MAP_GAP_TerminateLinkReq(
                        OAD_getactiveCxnHandle(),
                        HCI_DISCONNECT_REMOTE_USER_TERM);
                }
                else if(status != OAD_SUCCESS)
                {
                    Log_info1("OAD Error: %d", status);
                }
            }

            if(events & PZ_OAD_COMPLETE_EVT)
            {
                // Register for L2CAP Flow Control Events
                L2CAP_RegisterFlowCtrlTask(selfEntity);
            }
        }
    }

}

/*********************************************************************
 * @fn      AchillesPB_processL2CAPMsg
 *
 * @brief   Process L2CAP messages and events.
 *
 * @param   pMsg - L2CAP signal buffer from stack
 *
 * @return  None
 */
static void AchillesPB_processL2CAPMsg(l2capSignalEvent_t *pMsg)
{
    static bool firstRun = TRUE;

    switch(pMsg->opcode)
    {
      case L2CAP_NUM_CTRL_DATA_PKT_EVT:
      {
          /*
           * We cannot reboot the device immediately after receiving
           * the enable command, we must allow the stack enough time
           * to process and respond to the OAD_EXT_CTRL_ENABLE_IMG
           * command. This command will determine the number of
           * packets currently queued up by the LE controller.
           */
          if(firstRun)
          {
              firstRun = false;

              // We only want to set the numPendingMsgs once
              numPendingMsgs = MAX_NUM_PDU -
                               pMsg->cmd.numCtrlDataPktEvt.numDataPkt;

              // Wait until all PDU have been sent on cxn events
              Gap_RegisterConnEventCb(AchillesPB_connEvtCB,
                                        GAP_CB_REGISTER,
                                        GAP_CB_CONN_EVENT_ALL,
                                        OAD_getactiveCxnHandle());

              /* Set the flag so that the connection event callback will
               * be processed in the context of a pending OAD reboot
               */
              oadWaitReboot = true;
          }

          break;
      }
      default:
          break;
    }
}

/*********************************************************************
 * @fn      AchillesPB_checkSvcChgndFlag
 *
 * @brief   Process an incoming OAD reboot
 *
 * @param   flag - mask of events received
 *
 * @return  none
 */
static void AchillesPB_checkSvcChgndFlag(uint32_t flag)
{
    /*
     * When booting for the first time after an OAD the device must send a service
     * changed indication. This will cause any peers to rediscover services.
     *
     * To prevent sending a service changed IND on every boot, a flag is stored
     * in NV to determine whether or not the service changed IND needs to be
     * sent
     */
    uint8_t status = osal_snv_read(BLE_NVID_CUST_START,
                                   sizeof(flag),
                                   (uint8 *)&flag);
    if(status != SUCCESS)
    {
        /*
         * On first boot the NV item will not have yet been initialzed, and the read
         * will fail. Do a write to set the initial value of the flash in NV
         */
        osal_snv_write(BLE_NVID_CUST_START, sizeof(flag),
                       (uint8 *)&flag);
    }
}

/*********************************************************************
 * @fn      AchillesPB_processStackEvent
 *
 * @brief   Process stack event. The event flags received are user-selected
 *          via previous calls to stack APIs.
 *
 * @param   stack_event - mask of events received
 *
 * @return  none
 */
static void AchillesPB_processStackEvent(uint32_t stack_event)
{
    // Intentionally blank
}

/*********************************************************************
 * @fn      AchillesPB_processGATTMsg
 *
 * @brief   Process GATT messages and events.
 *
 * @param   pMsg - message to process
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t AchillesPB_processGATTMsg(gattMsgEvent_t *pMsg)
{
    if(pMsg->method == ATT_FLOW_CTRL_VIOLATED_EVENT)
    {
        // ATT request-response or indication-confirmation flow control is
        // violated. All subsequent ATT requests or indications will be dropped.
        // The app is informed in case it wants to drop the connection.

        // Display the opcode of the message that caused the violation.
        Log_error1("FC Violated: %d", pMsg->msg.flowCtrlEvt.opcode);
    }
    else if(pMsg->method == ATT_MTU_UPDATED_EVENT)
    {
        // MTU size updated
        OAD_setBlockSize(pMsg->msg.mtuEvt.MTU);
        Log_info1("MTU Size: %d", pMsg->msg.mtuEvt.MTU);
    }

    // Free message payload. Needed only for ATT Protocol messages
    GATT_bm_free(&pMsg->msg, pMsg->method);

    // It's safe to free the incoming message
    return(TRUE);
}

/*********************************************************************
 * @fn      AchillesPB_processApplicationMessage
 *
 * @brief   Handle application messages
 *
 *          These are messages not from the BLE stack, but from the
 *          application itself.
 *
 *          For example, in a Software Interrupt (Swi) it is not possible to
 *          call any BLE APIs, so instead the Swi function must send a message
 *          to the application Task for processing in Task context.
 *
 * @param   pMsg  Pointer to the message of type pzMsg_t.
 */
static void AchillesPB_processApplicationMessage(pzMsg_t *pMsg)
{
    // Cast to pzCharacteristicData_t* here since it's a common message pdu type.
    pzCharacteristicData_t *pCharData = (pzCharacteristicData_t *)pMsg->pData;

    switch(pMsg->event)
    {
      case HCI_BLE_HARDWARE_ERROR_EVENT_CODE:
          AssertHandler(HAL_ASSERT_CAUSE_HARDWARE_ERROR,0);
          break;

      case PZ_SERVICE_WRITE_EVT: /* Message about received value write */
          /* Call different handler per service */
          switch(pCharData->svcUUID)
          {

            case DATA_SERVICE_SERV_UUID:
                AchillesPB_DataService_ValueChangeHandler(pCharData);
                break;
            case OBSERVERNOTIFY_SERV_UUID:
                user_observerNotify_ValueChangeHandler(pCharData);
                break;
            case RUNHEATER_SERV_UUID:
                          user_runHeater_ValueChangeHandler(pCharData);
                          break;

            case PAUSEHEATER_SERV_UUID:
                          user_pauseHeater_ValueChangeHandler(pCharData);
                          break;

            case STOPHEATER_SERV_UUID:
                          user_stopHeater_ValueChangeHandler(pCharData);
                          break;

          }
          break;

      case PZ_SERVICE_CFG_EVT: /* Message about received CCCD write */
          /* Call different handler per service */
          switch(pCharData->svcUUID)
          {
            case BUTTON_SERVICE_SERV_UUID:
                AchillesPB_ButtonService_CfgChangeHandler(pCharData);
                break;
            case DATA_SERVICE_SERV_UUID:
                AchillesPB_DataService_CfgChangeHandler(pCharData);
                break;
          }
          break;

      case PZ_UPDATE_CHARVAL_EVT: /* Message from ourselves to send  */
          AchillesPB_updateCharVal(pCharData);
          break;

      case PZ_BUTTON_DEBOUNCED_EVT: /* Message from swi about gpio change */
      {
          pzButtonState_t *pButtonState = (pzButtonState_t *)pMsg->pData;
          AchillesPB_handleButtonPress(pButtonState);
      }
      break;

      case PZ_ADV_EVT:
          AchillesPB_processAdvEvent((pzGapAdvEventData_t*)(pMsg->pData));
          break;

      case PZ_SEND_PARAM_UPD_EVT:
      {
          // Send connection parameter update
          pzSendParamReq_t* req = (pzSendParamReq_t *)pMsg->pData;
          AchillesPB_sendParamUpdate(req->connHandle);
      }
      break;

      case PZ_START_ADV_EVT:
          if(linkDB_NumActive() < MAX_NUM_BLE_CONNS)
          {
              // Enable advertising if there is room for more connections
              GapAdv_enable(advHandleLegacy, GAP_ADV_ENABLE_OPTIONS_USE_MAX, 0);
          }
          break;

      case PZ_PAIRSTATE_EVT: /* Message about the pairing state */
          AchillesPB_processPairState((pzPairStateData_t*)(pMsg->pData));
          break;

      case PZ_PASSCODE_EVT: /* Message about pairing PIN request */
      {
          pzPasscodeReq_t *pReq = (pzPasscodeReq_t *)pMsg->pData;
          AchillesPB_processPasscode(pReq);
      }
      break;

      case PZ_CONN_EVT:
        AchillesPB_processConnEvt((Gap_ConnEventRpt_t *)(pMsg->pData));
        break;

      case PZ_READ_RPA_EVT:
      {
        uint8_t* pRpaNew;
        // Need static so string persists until printed in idle thread.
        static uint8_t rpaAddrStr[3 * B_ADDR_LEN + 1];

        // Read the current RPA.
        pRpaNew = GAP_GetDevAddress(FALSE);
        if(pRpaNew != NULL)
        {
          if(memcmp(pRpaNew, rpaAddr, B_ADDR_LEN) != 0)
          {
            util_arrtohex(pRpaNew, B_ADDR_LEN, rpaAddrStr, sizeof(rpaAddrStr),
                            UTIL_ARRTOHEX_REVERSE);
            //print RP address
            Log_info1("RP ADDR: " \
                      ANSI_COLOR(FG_GREEN) "%s" ANSI_COLOR(ATTR_RESET),
                      (uintptr_t)rpaAddrStr);

            memcpy(rpaAddr, pRpaNew, B_ADDR_LEN);
          }
        }
        break;
      }

      case AC_NTFY_CLK_EVT:
      {
          notifySetdata();
          break;
      }
      case PZ_MSG_button_sense:
            {
                prsButtondata();
                break;
            }
      case PZ_MSG_I2C_TIMER:
          {
          Initial_temp2= I2CTMT117(0x49)/100;
          if(Initial_temp2 !=0){
                        temp2 = Initial_temp2;
                    }
          Task_sleep (10 * (1000 / Clock_tickPeriod));
          Initial_temp1= I2CTMT117(0x48)/100;
                    Task_sleep (10 * (1000 / Clock_tickPeriod));
                    if(Initial_temp1 !=0){
                        temp1 = Initial_temp1;
                    }
          Current_Temperature_01=temp1;
          Current_Temperature_02=temp2;
          batteryLevelApp = BQ25887();
          Log_info1("Temperature of temp1 is : %d (C)", Current_Temperature_01);
          Log_info1("Temperature of temp2 is : %d (C)", Current_Temperature_02);
          break;
          }
      default:
        break;
    }

    if(pMsg->pData != NULL)
    {
        ICall_free(pMsg->pData);
    }
}

/*********************************************************************
 * @fn      AchillesPB_processGapMessage
 *
 * @brief   Process an incoming GAP event.
 *
 * @param   pMsg - message to process
 */
static void AchillesPB_processGapMessage(gapEventHdr_t *pMsg)
{
    switch(pMsg->opcode)
    {
    case GAP_DEVICE_INIT_DONE_EVENT:
    {
        uint8_t *pRpaNew;
        bStatus_t status = FAILURE;
        // Need static so string persists until printed in idle thread.
        static uint8_t rpaAddrStr[3 * B_ADDR_LEN + 1];

        gapDeviceInitDoneEvent_t *pPkt = (gapDeviceInitDoneEvent_t *)pMsg;

        if(pPkt->hdr.status == SUCCESS)
        {
            // Store the system ID
            uint8_t systemId[DEVINFO_SYSTEM_ID_LEN];

            // use 6 bytes of device address for 8 bytes of system ID value
            systemId[0] = pPkt->devAddr[0];
            systemId[1] = pPkt->devAddr[1];
            systemId[2] = pPkt->devAddr[2];

            // set middle bytes to zero
            systemId[4] = 0x00;
            systemId[3] = 0x00;

            // shift three bytes up
            systemId[7] = pPkt->devAddr[5];
            systemId[6] = pPkt->devAddr[4];
            systemId[5] = pPkt->devAddr[3];

            // Set Device Info Service Parameter
            DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN,
                                 systemId);
            // Display device address
            // Need static so string persists until printed in idle thread.
            static uint8_t addrStr[3 * B_ADDR_LEN + 1];
            util_arrtohex(pPkt->devAddr, B_ADDR_LEN, addrStr, sizeof addrStr,
                          UTIL_ARRTOHEX_REVERSE);
            Log_info1("GAP is started. Our address: " \
                      ANSI_COLOR(FG_GREEN) "%s" ANSI_COLOR(ATTR_RESET),
                      (uintptr_t)addrStr);

            // Setup and start Advertising
            // For more information, see the GAP section in the User's Guide:
            // http://software-dl.ti.com/lprf/ble5stack-latest/

            // Create Advertisement set #1 and assign handle
            status = GapAdv_create(&AchillesPB_advCallback, &advParams1,
                                   &advHandleLegacy);

            APP_ASSERT(status == SUCCESS);

            Log_info1("Name in advData1 array: " \
                      ANSI_COLOR(FG_YELLOW) "%s" ANSI_COLOR(ATTR_RESET),
                      (uintptr_t)util_getLocalNameStr(advData1,
                                                      sizeof(advData1)));

            // Load advertising data for set #1 that is statically allocated by the app
            status = GapAdv_loadByHandle(advHandleLegacy, GAP_ADV_DATA_TYPE_ADV,
                                         sizeof(advData1), advData1);
            APP_ASSERT(status == SUCCESS);

            // Load scan response data for set #1 that is statically allocated by the app
            status =
                GapAdv_loadByHandle(advHandleLegacy, GAP_ADV_DATA_TYPE_SCAN_RSP,
                                    sizeof(scanResData1),
                                    scanResData1);
            APP_ASSERT(status == SUCCESS);

            // Set event mask for set #1
            status = GapAdv_setEventMask(advHandleLegacy,
                                         GAP_ADV_EVT_MASK_START_AFTER_ENABLE |
                                         GAP_ADV_EVT_MASK_END_AFTER_DISABLE |
                                         GAP_ADV_EVT_MASK_SET_TERMINATED);

            // Enable legacy advertising for set #1
            status =
                GapAdv_enable(advHandleLegacy, GAP_ADV_ENABLE_OPTIONS_USE_MAX,
                              0);
            APP_ASSERT(status == SUCCESS);

            if( addrMode > ADDRMODE_RANDOM )
            {
              // Read the current RPA.
              pRpaNew = GAP_GetDevAddress(FALSE);
              Error_Code = 47;
              if(pRpaNew != NULL)
              {
                // Update the current RPA.
                memcpy(rpaAddr, pRpaNew, B_ADDR_LEN);
                //print RP address
                util_arrtohex(pRpaNew, B_ADDR_LEN, rpaAddrStr, sizeof(rpaAddrStr),
                              UTIL_ARRTOHEX_REVERSE);
                Log_info1("RP ADDR: " \
                          ANSI_COLOR(FG_GREEN) "%s" ANSI_COLOR(ATTR_RESET),
                          (uintptr_t)rpaAddrStr);

                // Create one-shot clock for RPA check event.
                Util_constructClock(&clkRpaRead, AchillesPB_clockHandler,
                              READ_RPA_PERIOD, 0, true, PZ_READ_RPA_EVT);
              }
            }
        }
        break;
    }

    case GAP_LINK_ESTABLISHED_EVENT:
    {
        gapEstLinkReqEvent_t *pPkt = (gapEstLinkReqEvent_t *)pMsg;

        // Display the amount of current connections
        Log_info2("Link establish event, status 0x%02x. Num Conns: %d",
                  pPkt->hdr.status,
                  linkDB_NumActive());

        if(pPkt->hdr.status == SUCCESS)
        {
            // Add connection to list
            AchillesPB_addConn(pPkt->connectionHandle);

            // Display the address of this connection
            static uint8_t addrStr[3 * B_ADDR_LEN + 1];
            util_arrtohex(pPkt->devAddr, B_ADDR_LEN, addrStr, sizeof addrStr,
                          UTIL_ARRTOHEX_REVERSE);
            Log_info1("Connected. Peer address: " \
                        ANSI_COLOR(FG_GREEN)"%s"ANSI_COLOR(ATTR_RESET),
                      (uintptr_t)addrStr);
                    onLEDs(1,0,0, 0xFF );
                    bleflag = 1;
                    delayFun (3000);
                    onLEDs(1,0,0, 0 );
            // If we are just connecting after an OAD send SVC changed
            if(sendSvcChngdOnNextBoot == TRUE)
            {
                /* Warning: This requires -DV41_FEATURES=L2CAP_COC_CFG to be
                 * defined in the build_config.opt of the stack project
                 * If L2CAP CoC is not desired comment the following code out
                 */
                GAPBondMgr_ServiceChangeInd(pPkt->connectionHandle, TRUE);

                sendSvcChngdOnNextBoot = FALSE;
            }
        }

        if(linkDB_NumActive() < MAX_NUM_BLE_CONNS)
        {
            // Start advertising since there is room for more connections
            GapAdv_enable(advHandleLegacy, GAP_ADV_ENABLE_OPTIONS_USE_MAX, 0);
        }
    }
    break;

    case GAP_LINK_TERMINATED_EVENT:
    {
        gapTerminateLinkEvent_t *pPkt = (gapTerminateLinkEvent_t *)pMsg;

        // Display the amount of current connections
        Log_info0("Device Disconnected!");
        bleflag = 0;
        Log_info1("Num Conns: %d", linkDB_NumActive());
        // Remove the connection from the list and disable RSSI if needed
        AchillesPB_removeConn(pPkt->connectionHandle);
        if(Current_Status ==3){
            Error_Code = 34;
        }
        // Cancel the OAD if one is going on
        // A disconnect forces the peer to re-identify
        OAD_cancel();
        for(int x =0; x<=6;x++){
                    onLEDs(1,0,0, 0xFF );
                    delayFun (250);
                    onLEDs(1,0,0, 0 );
                    delayFun (250);
                }
        GapAdv_disable(advHandleLegacy);
        delayFun (200);
        GapAdv_enable(advHandleLegacy, GAP_ADV_ENABLE_OPTIONS_USE_MAX, 0);
    }
    break;

    case GAP_UPDATE_LINK_PARAM_REQ_EVENT:
        AchillesPB_handleUpdateLinkParamReq(
            (gapUpdateLinkParamReqEvent_t *)pMsg);
        break;

    case GAP_LINK_PARAM_UPDATE_EVENT:
        AchillesPB_handleUpdateLinkEvent((gapLinkUpdateEvent_t *)pMsg);
        break;

    case GAP_PAIRING_REQ_EVENT:
        // Disable advertising so that the peer device can be added to
        // the resolving list
        GapAdv_disable(advHandleLegacy);
        break;

#if defined ( NOTIFY_PARAM_UPDATE_RJCT )
    case GAP_LINK_PARAM_UPDATE_REJECT_EVENT:
        {
          linkDBInfo_t linkInfo;
          gapLinkUpdateEvent_t *pPkt = (gapLinkUpdateEvent_t *)pMsg;

          // Get the address from the connection handle
          linkDB_GetInfo(pPkt->connectionHandle, &linkInfo);

          // Display the address of the connection update failure
          Log_info2("Peer Device's Update Request Rejected 0x%x: %s", pPkt->opcode,
                     Util_convertBdAddr2Str(linkInfo.addr));

          break;
        }
#endif

    default:
        break;
    }
}

void AchillesPB_processHCIMsg(ICall_HciExtEvt *pEvt)
{
    ICall_Hdr *pMsg = (ICall_Hdr *)pEvt;

    // Process HCI message
    switch(pMsg->status)
    {
    case HCI_COMMAND_COMPLETE_EVENT_CODE:
        // Process HCI Command Complete Events here
        AchillesPB_processCmdCompleteEvt((hciEvt_CmdComplete_t *) pMsg);
        break;

    case HCI_BLE_HARDWARE_ERROR_EVENT_CODE:
        AssertHandler(HAL_ASSERT_CAUSE_HARDWARE_ERROR,0);
        break;

    // HCI Commands Events
    case HCI_COMMAND_STATUS_EVENT_CODE:
    {
        hciEvt_CommandStatus_t *pMyMsg = (hciEvt_CommandStatus_t *)pMsg;
        switch(pMyMsg->cmdOpcode)
        {
        case HCI_LE_SET_PHY:
        {
            if(pMyMsg->cmdStatus == HCI_ERROR_CODE_UNSUPPORTED_REMOTE_FEATURE)
            {
                Log_info0("PHY Change failure, peer does not support this");
            }
            else
            {
                Log_info1("PHY Update Status Event: 0x%x",
                          pMyMsg->cmdStatus);
            }

            AchillesPB_updatePHYStat(HCI_LE_SET_PHY, (uint8_t *)pMsg);
        }
        break;

        default:
            break;
        }
    }
    break;

    // LE Events
    case HCI_LE_EVENT_CODE:
    {
        hciEvt_BLEPhyUpdateComplete_t *pPUC =
            (hciEvt_BLEPhyUpdateComplete_t*) pMsg;

        // A Phy Update Has Completed or Failed
        if(pPUC->BLEEventCode == HCI_BLE_PHY_UPDATE_COMPLETE_EVENT)
        {
            if(pPUC->status != SUCCESS)
            {
                Log_info0("PHY Change failure");
            }
            else
            {
                // Only symmetrical PHY is supported.
                // rxPhy should be equal to txPhy.
                Log_info1("PHY Updated to %s",
                          (uintptr_t)((pPUC->rxPhy == PHY_UPDATE_COMPLETE_EVENT_1M) ? "1M" :
                                      (pPUC->rxPhy == PHY_UPDATE_COMPLETE_EVENT_2M) ? "2M" :
                                      (pPUC->rxPhy == PHY_UPDATE_COMPLETE_EVENT_CODED) ? "CODED" : "Unexpected PHY Value"));
            }

            AchillesPB_updatePHYStat(HCI_BLE_PHY_UPDATE_COMPLETE_EVENT,
                                      (uint8_t *)pMsg);
        }
    }
    break;

    default:
        break;
    }
}

/*********************************************************************
 * @fn      AchillesPB_processAdvEvent
 *
 * @brief   Process advertising event in app context
 *
 * @param   pEventData
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static void AchillesPB_processAdvEvent(pzGapAdvEventData_t *pEventData)
{
    switch(pEventData->event)
    {
    /* Sent on the first advertisement after a GapAdv_enable */
    case GAP_EVT_ADV_START_AFTER_ENABLE:
        Log_info1("Adv Set %d Enabled", *(uint8_t *)(pEventData->pBuf));
        break;

    /* Sent after advertising stops due to a GapAdv_disable */
    case GAP_EVT_ADV_END_AFTER_DISABLE:
        Log_info1("Adv Set %d Disabled", *(uint8_t *)(pEventData->pBuf));
        break;

    /* Sent at the beginning of each advertisement. (Note that this event
     * is not enabled by default, see GapAdv_setEventMask). */
    case GAP_EVT_ADV_START:
        break;

    /* Sent after each advertisement. (Note that this event is not enabled
     * by default, see GapAdv_setEventMask). */
    case GAP_EVT_ADV_END:
        break;

    /* Sent when an advertisement set is terminated due to a
     * connection establishment */
    case GAP_EVT_ADV_SET_TERMINATED:
    {
        GapAdv_setTerm_t *advSetTerm = (GapAdv_setTerm_t *)(pEventData->pBuf);

        Log_info2("Adv Set %d disabled after conn %d",
                  advSetTerm->handle, advSetTerm->connHandle);
    }
    break;

    /* Sent when a scan request is received. (Note that this event
     * is not enabled by default, see GapAdv_setEventMask). */
    case GAP_EVT_SCAN_REQ_RECEIVED:
        break;

    /* Sent when an operation could not complete because of a lack of memory.
       This message is not allocated on the heap and must not be freed */
    case GAP_EVT_INSUFFICIENT_MEMORY:
        break;

    default:
        break;
    }

  // All events have associated memory to free except the insufficient memory
  // event
  if (pEventData->event != GAP_EVT_INSUFFICIENT_MEMORY)
  {
    ICall_free(pEventData->pBuf);
  }
}

/*********************************************************************
 * @fn      AchillesPB_processPairState
 *
 * @brief   Process the new paring state.
 *
 * @param   pPairData - pointer to pair state data container
 */
static void AchillesPB_processPairState(pzPairStateData_t *pPairData)
{
    uint8_t state = pPairData->state;
    uint8_t status = pPairData->status;

    switch(state)
    {
    case GAPBOND_PAIRING_STATE_STARTED:
        Log_info0("Pairing started");
        break;

    case GAPBOND_PAIRING_STATE_COMPLETE:
        if(status == SUCCESS)
        {
            Log_info0("Pairing success");
        }
        else
        {
            Log_info1("Pairing fail: %d", status);
        }
        break;

    case GAPBOND_PAIRING_STATE_ENCRYPTED:
        if(status == SUCCESS)
        {
            Log_info0("Encryption success");
        }
        else
        {
            Log_info1("Encryption failed: %d", status);
        }
        break;

    case GAPBOND_PAIRING_STATE_BOND_SAVED:
        if(status == SUCCESS)
        {
            Log_info0("Bond save success");
        }
        else
        {
            Log_info1("Bond save failed: %d", status);
        }
        break;

    default:
        break;
    }
}

/*********************************************************************
 * @fn      AchillesPB_processPasscode
 *
 * @brief   Process the Passcode request.
 *
 * @param   pReq - pointer to passcode req
 */
static void AchillesPB_processPasscode(pzPasscodeReq_t *pReq)
{
    Log_info2("BondMgr Requested passcode. We are %s passcode %06d",
              (uintptr_t)(pReq->uiInputs ? "Sending" : "Displaying"),
              B_APP_DEFAULT_PASSCODE);

    // Send passcode response.
    GAPBondMgr_PasscodeRsp(pReq->connHandle, SUCCESS, B_APP_DEFAULT_PASSCODE);
}
/*********************************************************************
 * @fn      AchillesPB_processConnEvt
 *
 * @brief   Process connection event.
 *
 * @param pReport pointer to connection event report
 */
static void AchillesPB_processConnEvt(Gap_ConnEventRpt_t *pReport)
{
  /* If we are waiting for an OAD Reboot, process connection events to ensure
   * that we are not waiting to send data before restarting
   */
  if(oadWaitReboot)
  {
      // Wait until all pending messages are sent
      if(numPendingMsgs == 0)
      {
          // Store the flag to indicate that a service changed IND will
          // be sent at the next boot
          sendSvcChngdOnNextBoot = TRUE;

          uint8_t status = osal_snv_write(BLE_NVID_CUST_START,
                                          sizeof(sendSvcChngdOnNextBoot),
                                          (uint8 *)&sendSvcChngdOnNextBoot);
          if(status != SUCCESS)
          {
              Log_error1("SNV WRITE FAIL: %d", status);
          }

          // Reset the system
          SysCtrlSystemReset();
      }
      else
      {
        numPendingMsgs--;
      }
  }
  else
  {
    // Process connection events normally
    Log_info1("Connection event done for connHandle: %d", pReport->handle);

  }
}
/*********************************************************************
 * @fn      AchillesPB_clockHandler
 *
 * @brief   clock handler function
 *
 * @param   arg - argument from the clock initiator
 *
 * @return  none
 */
void AchillesPB_clockHandler(UArg arg)
{
  uint8_t evtId = (uint8_t) (arg & 0xFF);

  switch (evtId)
  {
    case PZ_READ_RPA_EVT:
    {
      // Restart timer
      Util_startClock(&clkRpaRead);
      // Let the application handle the event
      AchillesPB_enqueueMsg(PZ_READ_RPA_EVT, NULL);
      break;
    }

    default:
      break;
  }
}

/*********************************************************************
 * @fn      AchillesPB_connEvtCB
 *
 * @brief   Connection event callback.
 *
 * @param pReport pointer to connection event report
 */
static void AchillesPB_connEvtCB(Gap_ConnEventRpt_t *pReport)
{
  // Enqueue the event for processing in the app context.
  if(AchillesPB_enqueueMsg(PZ_CONN_EVT, pReport) != SUCCESS)
  {
    ICall_free(pReport);
  }
}

/*********************************************************************
 * @fn      AchillesPB_processCmdCompleteEvt
 *
 * @brief   Process an incoming OSAL HCI Command Complete Event.
 *
 * @param   pMsg - message to process
 */
static void AchillesPB_processCmdCompleteEvt(hciEvt_CmdComplete_t *pMsg)
{
    uint8_t status = pMsg->pReturnParam[0];

    //Find which command this command complete is for
    switch(pMsg->cmdOpcode)
    {
    case HCI_READ_RSSI:
    {
        int8 rssi = (int8)pMsg->pReturnParam[3];

        // Display RSSI value, if RSSI is higher than threshold, change to faster PHY
        if(status == SUCCESS)
        {
            uint16_t handle = BUILD_UINT16(pMsg->pReturnParam[1],
                                           pMsg->pReturnParam[2]);

            Log_info2("RSSI:%d, connHandle %d",
                      (uint32_t)(rssi),
                      (uint32_t)handle);
        } // end of if (status == SUCCESS)
        break;
    }

    case HCI_LE_READ_PHY:
    {
        if(status == SUCCESS)
        {
            Log_info2("RXPh: %d, TXPh: %d",
                      pMsg->pReturnParam[3], pMsg->pReturnParam[4]);
        }
        break;
    }

    default:
        break;
    } // end of switch (pMsg->cmdOpcode)
}

/*********************************************************************
 * @fn      AchillesPB_handleUpdateLinkParamReq
 *
 * @brief   Receive and respond to a parameter update request sent by
 *          a peer device
 *
 * @param   pReq - pointer to stack request message
 */
static void AchillesPB_handleUpdateLinkParamReq(
    gapUpdateLinkParamReqEvent_t *pReq)
{
    gapUpdateLinkParamReqReply_t rsp;

    rsp.connectionHandle = pReq->req.connectionHandle;
    rsp.signalIdentifier = pReq->req.signalIdentifier;

    // Only accept connection intervals with slave latency of 0
    // This is just an example of how the application can send a response
    if(pReq->req.connLatency == 0)
    {
        rsp.intervalMin = pReq->req.intervalMin;
        rsp.intervalMax = pReq->req.intervalMax;
        rsp.connLatency = pReq->req.connLatency;
        rsp.connTimeout = pReq->req.connTimeout;
        rsp.accepted = TRUE;
    }
    else
    {
        rsp.accepted = FALSE;
    }

    // Send Reply
    VOID GAP_UpdateLinkParamReqReply(&rsp);
}

/*********************************************************************
 * @fn      AchillesPB_handleUpdateLinkEvent
 *
 * @brief   Receive and parse a parameter update that has occurred.
 *
 * @param   pEvt - pointer to stack event message
 */
static void AchillesPB_handleUpdateLinkEvent(gapLinkUpdateEvent_t *pEvt)
{
    // Get the address from the connection handle
    linkDBInfo_t linkInfo;
    linkDB_GetInfo(pEvt->connectionHandle, &linkInfo);

    static uint8_t addrStr[3 * B_ADDR_LEN + 1];
    util_arrtohex(linkInfo.addr, B_ADDR_LEN, addrStr, sizeof addrStr,
                  UTIL_ARRTOHEX_REVERSE);

    if(pEvt->status == SUCCESS)
    {
        uint8_t ConnIntervalFracture = 25*(pEvt->connInterval % 4);
        // Display the address of the connection update
        Log_info5(
            "Updated params for %s, interval: %d.%d ms, latency: %d, timeout: %d ms",
            (uintptr_t)addrStr,
            (uintptr_t)(pEvt->connInterval*CONN_INTERVAL_MS_CONVERSION),
            ConnIntervalFracture,
            pEvt->connLatency,
            pEvt->connTimeout*CONN_TIMEOUT_MS_CONVERSION);
    }
    else
    {
        // Display the address of the connection update failure
        Log_info2("Update Failed 0x%02x: %s", pEvt->opcode, (uintptr_t)addrStr);
    }

    // Check if there are any queued parameter updates
    pzConnHandleEntry_t *connHandleEntry = (pzConnHandleEntry_t *)List_get(
        &paramUpdateList);
    if(connHandleEntry != NULL)
    {
        // Attempt to send queued update now
        AchillesPB_sendParamUpdate(*(connHandleEntry->connHandle));

        // Free list element
        ICall_free(connHandleEntry->connHandle);
        ICall_free(connHandleEntry);
    }
}

/*********************************************************************
 * @fn      AchillesPB_addConn
 *
 * @brief   Add a device to the connected device list
 *
 * @param   connHandle - connection handle
 *
 * @return  bleMemAllocError if a param update event could not be sent. Else SUCCESS.
 */
static uint8_t AchillesPB_addConn(uint16_t connHandle)
{
    uint8_t i;
    uint8_t status = bleNoResources;

    // Try to find an available entry
    for(i = 0; i < MAX_NUM_BLE_CONNS; i++)
    {
        if(connList[i].connHandle == LINKDB_CONNHANDLE_INVALID)
        {
            // Found available entry to put a new connection info in
            connList[i].connHandle = connHandle;

#ifdef DEFAULT_SEND_PARAM_UPDATE_REQ
            // Create a clock object and start
            connList[i].pUpdateClock
              = (Clock_Struct*) ICall_malloc(sizeof(Clock_Struct));

            if (connList[i].pUpdateClock)
            {
              Util_constructClock(connList[i].pUpdateClock,
                                  AchillesPB_paramUpdClockHandler,
                                  SEND_PARAM_UPDATE_DELAY, 0, true,
                                  (uintptr_t)connHandle);
            }
#endif

            // Set default PHY to 1M
            connList[i].currPhy = HCI_PHY_1_MBPS; // TODO: Is this true, neccessarily?

            break;
        }
    }

    return(status);
}

/*********************************************************************
 * @fn      AchillesPB_getConnIndex
 *
 * @brief   Find index in the connected device list by connHandle
 *
 * @param   connHandle - connection handle
 *
 * @return  the index of the entry that has the given connection handle.
 *          if there is no match, MAX_NUM_BLE_CONNS will be returned.
 */
static uint8_t AchillesPB_getConnIndex(uint16_t connHandle)
{
    uint8_t i;

    for(i = 0; i < MAX_NUM_BLE_CONNS; i++)
    {
        if(connList[i].connHandle == connHandle)
        {
            return(i);
        }
    }

    return(MAX_NUM_BLE_CONNS);
}

/*********************************************************************
 * @fn      AchillesPB_clearConnListEntry
 *
 * @brief   Clear the connection information structure held locally.
 *
 * @param   connHandle - connection handle
 *
 * @return  SUCCESS if connHandle found valid index or bleInvalidRange
 *          if index wasn't found. LINKDB_CONNHANDLE_ALL will always succeed.
 */
static uint8_t AchillesPB_clearConnListEntry(uint16_t connHandle)
{
    uint8_t i;
    // Set to invalid connection index initially
    uint8_t connIndex = MAX_NUM_BLE_CONNS;

    if(connHandle != LINKDB_CONNHANDLE_ALL)
    {
        // Get connection index from handle
        connIndex = AchillesPB_getConnIndex(connHandle);
        if(connIndex >= MAX_NUM_BLE_CONNS)
        {
            return(bleInvalidRange);
        }
    }

    // Clear specific handle or all handles
    for(i = 0; i < MAX_NUM_BLE_CONNS; i++)
    {
        if((connIndex == i) || (connHandle == LINKDB_CONNHANDLE_ALL))
        {
            connList[i].connHandle = LINKDB_CONNHANDLE_INVALID;
            connList[i].currPhy = 0;
            connList[i].phyCngRq = 0;
            connList[i].phyRqFailCnt = 0;
            connList[i].rqPhy = 0;
        }
    }

    return(SUCCESS);
}

/*********************************************************************
 * @fn      AchillesPB_removeConn
 *
 * @brief   Remove a device from the connected device list
 *
 * @param   connHandle - connection handle
 *
 * @return  index of the connected device list entry where the new connection
 *          info is removed from.
 *          if connHandle is not found, MAX_NUM_BLE_CONNS will be returned.
 */
static uint8_t AchillesPB_removeConn(uint16_t connHandle)
{
    uint8_t connIndex = AchillesPB_getConnIndex(connHandle);

    if(connIndex < MAX_NUM_BLE_CONNS)
    {
      Clock_Struct* pUpdateClock = connList[connIndex].pUpdateClock;

      if (pUpdateClock != NULL)
      {
        // Stop and destruct the RTOS clock if it's still alive
        if (Util_isActive(pUpdateClock))
        {
          Util_stopClock(pUpdateClock);
        }

        // Destruct the clock object
        Clock_destruct(pUpdateClock);
        // Free clock struct
        ICall_free(pUpdateClock);
      }
      // Clear Connection List Entry
      AchillesPB_clearConnListEntry(connHandle);
    }

    return connIndex;
}

/*********************************************************************
 * @fn      AchillesPB_sendParamUpdate
 *
 * @brief   Remove a device from the connected device list
 *
 * @param   connHandle - connection handle
 */
static void AchillesPB_sendParamUpdate(uint16_t connHandle)
{
    gapUpdateLinkParamReq_t req;
    uint8_t connIndex;

    req.connectionHandle = connHandle;
#ifdef DEFAULT_SEND_PARAM_UPDATE_REQ
    req.connLatency = DEFAULT_DESIRED_SLAVE_LATENCY;
    req.connTimeout = DEFAULT_DESIRED_CONN_TIMEOUT;
    req.intervalMin = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    req.intervalMax = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
#endif

    connIndex = AchillesPB_getConnIndex(connHandle);
    APP_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

    // Deconstruct the clock object
    Clock_destruct(connList[connIndex].pUpdateClock);
    // Free clock struct
    ICall_free(connList[connIndex].pUpdateClock);
    connList[connIndex].pUpdateClock = NULL;

    // Send parameter update
    bStatus_t status = GAP_UpdateLinkParamReq(&req);

    // If there is an ongoing update, queue this for when the update completes
    if(status == bleAlreadyInRequestedMode)
    {
        pzConnHandleEntry_t *connHandleEntry =
            ICall_malloc(sizeof(pzConnHandleEntry_t));
        if(connHandleEntry)
        {
            connHandleEntry->connHandle = ICall_malloc(sizeof(uint16_t));

            if(connHandleEntry->connHandle)
            {
                *(connHandleEntry->connHandle) = connHandle;

                List_put(&paramUpdateList, (List_Elem *)&connHandleEntry);
            }
        }
    }
}

/*********************************************************************
 * @fn      AchillesPB_updatePHYStat
 *
 * @brief   Update the auto phy update state machine
 *
 * @param   eventCode - HCI LE Event code
 *          pMsg - message to process
 */
static void AchillesPB_updatePHYStat(uint16_t eventCode, uint8_t *pMsg)
{
    uint8_t connIndex;
    pzConnHandleEntry_t *connHandleEntry;

    switch(eventCode)
    {
    case HCI_LE_SET_PHY:
    {
        // Get connection handle from list
        connHandleEntry = (pzConnHandleEntry_t *)List_get(&setPhyCommStatList);

        if(connHandleEntry)
        {
            // Get index from connection handle
            connIndex = AchillesPB_getConnIndex(*(connHandleEntry->connHandle));
            APP_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

            ICall_free(connHandleEntry->connHandle);
            ICall_free(connHandleEntry);

            hciEvt_CommandStatus_t *pMyMsg = (hciEvt_CommandStatus_t *)pMsg;

            if(pMyMsg->cmdStatus == HCI_ERROR_CODE_UNSUPPORTED_REMOTE_FEATURE)
            {
                // Update the phy change request status for active RSSI tracking connection
                connList[connIndex].phyCngRq = FALSE;
                connList[connIndex].phyRqFailCnt++;
            }
        }
        break;
    }

    // LE Event - a Phy update has completed or failed
    case HCI_BLE_PHY_UPDATE_COMPLETE_EVENT:
    {
        hciEvt_BLEPhyUpdateComplete_t *pPUC =
            (hciEvt_BLEPhyUpdateComplete_t*) pMsg;

        if(pPUC)
        {
            // Get index from connection handle
            uint8_t index = AchillesPB_getConnIndex(pPUC->connHandle);
            APP_ASSERT(index < MAX_NUM_BLE_CONNS);

            // Update the phychange request status for active RSSI tracking connection
            connList[index].phyCngRq = FALSE;

            if(pPUC->status == SUCCESS)
            {
                connList[index].currPhy = pPUC->rxPhy;
            }
            if(pPUC->rxPhy != connList[index].rqPhy)
            {
                connList[index].phyRqFailCnt++;
            }
            else
            {
                // Reset the request phy counter and requested phy
                connList[index].phyRqFailCnt = 0;
                connList[index].rqPhy = 0;
            }
        }

        break;
    }

    default:
        break;
    } // end of switch (eventCode)
}

/*********************************************************************
 * @brief   Handle a debounced button press or release in Task context.
 *          Invoked by the taskFxn based on a message received from a callback.
 *
 * @see     buttonDebounceSwiFxn
 * @see     GPIO_Board_keyCallback
 *
 * @param   pState  pointer to pzButtonState_t message sent from debounce Swi.
 *
 * @return  None.
 */
static void AchillesPB_handleButtonPress(pzButtonState_t *pState)
{
    Log_info2("%s %s",
              (uintptr_t)(pState->gpioId ==
                          CONFIG_GPIO_BTN1 ? "Button 0" : "Button 1"),
              (uintptr_t)(pState->state ?
                          ANSI_COLOR(FG_GREEN)"pressed"ANSI_COLOR(ATTR_RESET) :
                          ANSI_COLOR(FG_YELLOW)"released"ANSI_COLOR(ATTR_RESET)
                         ));

    // Update the service with the new value.
    // Will automatically send notification/indication if enabled.
    switch(pState->gpioId)
    {
    case CONFIG_GPIO_BTN1:
        ButtonService_SetParameter(BS_BUTTON0_ID,
                                   sizeof(pState->state),
                                   &pState->state);

        break;
    case CONFIG_GPIO_BTN2:
        ButtonService_SetParameter(BS_BUTTON1_ID,
                                   sizeof(pState->state),
                                   &pState->state);
        break;
    }
}

/*********************************************************************
 * @brief   Handle a write request sent from a peer device.
 *
 *          Invoked by the Task based on a message received from a callback.
 *
 *          When we get here, the request has already been accepted by the
 *          service and is valid from a BLE protocol perspective as well as
 *          having the correct length as defined in the service implementation.
 *
 * @param   pCharData  pointer to malloc'd char write data
 *
 * @return  None.
 */

/*********************************************************************
 * @brief   Handle a CCCD (configuration change) write received from a peer
 *          device. This tells us whether the peer device wants us to send
 *          Notifications or Indications.
 *
 * @param   pCharData  pointer to malloc'd char write data
 *
 * @return  None.
 */
void AchillesPB_ButtonService_CfgChangeHandler(
    pzCharacteristicData_t *pCharData)
{
    // Cast received data to uint16, as that's the format for CCCD writes.
    uint16_t configValue = *(uint16_t *)pCharData->data;
    char *configValString;

    // Determine what to tell the user
    switch(configValue)
    {
    case GATT_CFG_NO_OPERATION:
        configValString = "Noti/Ind disabled";
        break;
    case GATT_CLIENT_CFG_NOTIFY:
        configValString = "Notifications enabled";
        break;
    case GATT_CLIENT_CFG_INDICATE:
        configValString = "Indications enabled";
        break;
    default:
        configValString = "Unsupported operation";
    }

    switch(pCharData->paramID)
    {
    case BS_BUTTON0_ID:
        Log_info3("CCCD Change msg: %s %s: %s",
                  (uintptr_t)"Button Service",
                  (uintptr_t)"BUTTON0",
                  (uintptr_t)configValString);
        // -------------------------
        // Do something useful with configValue here. It tells you whether someone
        // wants to know the state of this characteristic.
        // ...
        break;

    case BS_BUTTON1_ID:
        Log_info3("CCCD Change msg: %s %s: %s",
                  (uintptr_t)"Button Service",
                  (uintptr_t)"BUTTON1",
                  (uintptr_t)configValString);
        // -------------------------
        // Do something useful with configValue here. It tells you whether someone
        // wants to know the state of this characteristic.
        // ...
        break;
    }
}

/*********************************************************************
 * @brief   Handle a write request sent from a peer device.
 *
 *          Invoked by the Task based on a message received from a callback.
 *
 *          When we get here, the request has already been accepted by the
 *          service and is valid from a BLE protocol perspective as well as
 *          having the correct length as defined in the service implementation.
 *
 * @param   pCharData  pointer to malloc'd char write data
 *
 * @return  None.
 */
void AchillesPB_DataService_ValueChangeHandler(
    pzCharacteristicData_t *pCharData)
{
    // Value to hold the received string for printing via Log, as Log printouts
    // happen in the Idle task, and so need to refer to a global/static variable.
    static uint8_t received_string[DS_STRING_LEN] = {0};

    switch(pCharData->paramID)
    {
    case DS_STRING_ID:
        // Do something useful with pCharData->data here
        // -------------------------
        // Copy received data to holder array, ensuring NULL termination.
        memset(received_string, 0, DS_STRING_LEN);
        memcpy(received_string, pCharData->data,
               MIN(pCharData->dataLen, DS_STRING_LEN - 1));
        // Needed to copy before log statement, as the holder array remains after
        // the pCharData message has been freed and reused for something else.
        Log_info3("Value Change msg: %s %s: %s",
                  (uintptr_t)"Data Service",
                  (uintptr_t)"String",
                  (uintptr_t)received_string);
        break;

    case DS_STREAM_ID:
        Log_info3("Value Change msg: Data Service Stream: %02x:%02x:%02x...",
                  pCharData->data[0],
                  pCharData->data[1],
                  pCharData->data[2]);
        // -------------------------
        // Do something useful with pCharData->data here
        break;

    default:
        return;
    }
}

/*********************************************************************
 * @brief   Handle a CCCD (configuration change) write received from a peer
 *          device. This tells us whether the peer device wants us to send
 *          Notifications or Indications.
 *
 * @param   pCharData  pointer to malloc'd char write data
 *
 * @return  None.
 */
void AchillesPB_DataService_CfgChangeHandler(pzCharacteristicData_t *pCharData)
{
    // Cast received data to uint16, as that's the format for CCCD writes.
    uint16_t configValue = *(uint16_t *)pCharData->data;
    char *configValString;

    // Determine what to tell the user
    switch(configValue)
    {
    case GATT_CFG_NO_OPERATION:
        configValString = "Noti/Ind disabled";
        break;
    case GATT_CLIENT_CFG_NOTIFY:
        configValString = "Notifications enabled";
        break;
    case GATT_CLIENT_CFG_INDICATE:
        configValString = "Indications enabled";
        break;
    default:
        configValString = "Unsupported operation";
    }

    switch(pCharData->paramID)
    {
    case DS_STREAM_ID:
        Log_info3("CCCD Change msg: %s %s: %s",
                  (uintptr_t)"Data Service",
                  (uintptr_t)"Stream",
                  (uintptr_t)configValString);
        // -------------------------
        // Do something useful with configValue here. It tells you whether someone
        // wants to know the state of this characteristic.
        // ...
        break;
    }
}

/*********************************************************************
 * @brief  Convenience function for updating characteristic data via pzCharacteristicData_t
 *         structured message.
 *
 * @note   Must run in Task context in case BLE Stack APIs are invoked.
 *
 * @param  *pCharData  Pointer to struct with value to update.
 */
static void AchillesPB_updateCharVal(pzCharacteristicData_t *pCharData)
{
    switch(pCharData->svcUUID)
    {

    case BUTTON_SERVICE_SERV_UUID:
        ButtonService_SetParameter(pCharData->paramID, pCharData->dataLen,
                                   pCharData->data);
        break;
    }
}

/******************************************************************************
 *****************************************************************************
 *
 *  Handlers of direct system callbacks.
 *
 *  Typically enqueue the information or request as a message for the
 *  application Task for handling.
 *
 ****************************************************************************
 *****************************************************************************/

/*
 *  Callbacks from the Stack Task context (GAP or Service changes)
 *****************************************************************************/

/*********************************************************************
 * @fn      AchillesPB_advCallback
 *
 * @brief   GapAdv module callback
 *
 * @param   pMsg - message to process
 *          pBuf - data potentially accompanying event
 *          arg - not used
 */
static void AchillesPB_advCallback(uint32_t event, void *pBuf, uintptr_t arg)
{
    pzGapAdvEventData_t *eventData = ICall_malloc(sizeof(pzGapAdvEventData_t));

    if(eventData != NULL)
    {
        eventData->event = event;
        eventData->pBuf = pBuf;

        if(AchillesPB_enqueueMsg(PZ_ADV_EVT, eventData) != SUCCESS)
        {
          ICall_free(eventData);
        }
    }
}

/*********************************************************************
 * @fn      AchillesPB_pairStateCb
 *
 * @brief   Pairing state callback.
 *
 * @param   connHandle - connection handle
 *          state - pair state
 *          status - pair status
 */
static void AchillesPB_pairStateCb(uint16_t connHandle, uint8_t state,
                                    uint8_t status)
{
    pzPairStateData_t *pairState =
        (pzPairStateData_t *)ICall_malloc(sizeof(pzPairStateData_t));

    if(pairState != NULL)
    {
        pairState->state = state;
        pairState->connHandle = connHandle;
        pairState->status = status;

        if(AchillesPB_enqueueMsg(PZ_PAIRSTATE_EVT, pairState) != SUCCESS)
        {
          ICall_free(pairState);
        }
    }
}

/*********************************************************************
 * @fn      AchillesPB_passcodeCb
 *
 * @brief   Passcode callback.
 *
 * @param   pDeviceAddr - not used
 *          connHandle - connection handle
 *          uiInpuits - if TRUE, the local device should accept a passcode input
 *          uiOutputs - if TRUE, the local device should display the passcode
 *          numComparison - the code that should be displayed for numeric
 *          comparison pairing. If this is zero, then passcode pairing is occurring.
 */
static void AchillesPB_passcodeCb(uint8_t *pDeviceAddr,
                                   uint16_t connHandle,
                                   uint8_t uiInputs,
                                   uint8_t uiOutputs,
                                   uint32_t numComparison)
{
    pzPasscodeReq_t *req =
        (pzPasscodeReq_t *)ICall_malloc(sizeof(pzPasscodeReq_t));
    if(req != NULL)
    {
        req->connHandle = connHandle;
        req->uiInputs = uiInputs;
        req->uiOutputs = uiOutputs;
        req->numComparison = numComparison;

        if(AchillesPB_enqueueMsg(PZ_PASSCODE_EVT, req) != SUCCESS)
        {
          ICall_free(req);
        }
    }
    ;
}



/*********************************************************************
 * @fn      AchillesPB_DataService_ValueChangeCB
 *
 * @brief   Callback for characteristic change when a peer writes to us
 *
 * @param   connHandle - connection handle
 *          paramID - the parameter ID maps to the characteristic written to
 *          len - length of the data written
 *          pValue - pointer to the data written
 */
static void AchillesPB_DataService_ValueChangeCB(uint16_t connHandle,
                                                  uint8_t paramID, uint16_t len,
                                                  uint8_t *pValue)
{
    // See the service header file to compare paramID with characteristic.
    Log_info1("(CB) Data Svc Characteristic value change: paramID(%d). "
              "Sending msg to app.", paramID);

    pzCharacteristicData_t *pValChange =
        ICall_malloc(sizeof(pzCharacteristicData_t) + len);

    if(pValChange != NULL)
    {
        pValChange->svcUUID = DATA_SERVICE_SERV_UUID;
        pValChange->paramID = paramID;
        memcpy(pValChange->data, pValue, len);
        pValChange->dataLen = len;

        if(AchillesPB_enqueueMsg(PZ_SERVICE_WRITE_EVT, pValChange) != SUCCESS)
        {
          ICall_free(pValChange);
        }
    }
}

/*********************************************************************
 * @fn      AchillesPB_ButtonService_CfgChangeCB
 *
 * @brief   Callback for when a peer enables or disables the CCCD attribute,
 *          indicating they are interested in notifications or indications.
 *
 * @param   connHandle - connection handle
 *          paramID - the parameter ID maps to the characteristic written to
 *          len - length of the data written
 *          pValue - pointer to the data written
 */
static void AchillesPB_ButtonService_CfgChangeCB(uint16_t connHandle,
                                                  uint8_t paramID, uint16_t len,
                                                  uint8_t *pValue)
{
    Log_info1("(CB) Button Svc Char config change paramID(%d). "
              "Sending msg to app.", paramID);

    pzCharacteristicData_t *pValChange =
        ICall_malloc(sizeof(pzCharacteristicData_t) + len);

    if(pValChange != NULL)
    {
        pValChange->svcUUID = BUTTON_SERVICE_SERV_UUID;
        pValChange->paramID = paramID;
        memcpy(pValChange->data, pValue, len);
        pValChange->dataLen = len;

        if(AchillesPB_enqueueMsg(PZ_SERVICE_CFG_EVT, pValChange) != SUCCESS)
        {
          ICall_free(pValChange);
        }
    }
}

/*********************************************************************
 * @fn      AchillesPB_DataService_CfgChangeCB
 *
 * @brief   Callback for when a peer enables or disables the CCCD attribute,
 *          indicating they are interested in notifications or indications.
 *
 * @param   connHandle - connection handle
 *          paramID - the parameter ID maps to the characteristic written to
 *          len - length of the data written
 *          pValue - pointer to the data written
 */
static void AchillesPB_DataService_CfgChangeCB(uint16_t connHandle,
                                                uint8_t paramID, uint16_t len,
                                                uint8_t *pValue)
{
    Log_info1("(CB) Data Svc Char config change paramID(%d). "
              "Sending msg to app.", paramID);

    pzCharacteristicData_t *pValChange =
        ICall_malloc(sizeof(pzCharacteristicData_t) + len);

    if(pValChange != NULL)
    {
        pValChange->svcUUID = DATA_SERVICE_SERV_UUID;
        pValChange->paramID = paramID;
        memcpy(pValChange->data, pValue, len);
        pValChange->dataLen = len;

        if(AchillesPB_enqueueMsg(PZ_SERVICE_CFG_EVT, pValChange) != SUCCESS)
        {
          ICall_free(pValChange);
        }
    }
}

/*********************************************************************
 * @fn      AchillesPB_processOadWriteCB
 *
 * @brief   Process an OAD event
 *
 * @param   connHandle - the connection Handle this request is from.
 * @param   bim_var    - bim_var to set before resetting.
 *
 * @return  None.
 */
static void AchillesPB_processOadWriteCB(uint8_t event, uint16_t arg)
{
    Event_post(syncEvent, event);
}

/*
 *  Callbacks from Swi-context
 *****************************************************************************/

#ifdef DEFAULT_SEND_PARAM_UPDATE_REQ
/*********************************************************************
 * @fn      AchillesPB_paramUpdClockHandler
 *
 * @brief   Handler function for clock timeouts.
 *
 * @param   arg - app message pointer
 */
static void AchillesPB_paramUpdClockHandler(UArg arg)
{
    pzSendParamReq_t *req =
        (pzSendParamReq_t *)ICall_malloc(sizeof(pzSendParamReq_t));
    if(req)
    {
        req->connHandle = (uint16_t)arg;
        if(AchillesPB_enqueueMsg(PZ_SEND_PARAM_UPD_EVT, req) != SUCCESS)
        {
          ICall_free(req);
        }
    }
}
#endif

/*********************************************************************
 * @fn     buttonDebounceSwiFxn
 *
 * @brief  Callback from Clock module on timeout
 *
 *         Determines new state after debouncing
 *
 * @param  buttonId    The gpio being debounced
 */
static void buttonDebounceSwiFxn(UArg buttonId)
{
    // Used to send message to app
    pzButtonState_t buttonMsg = { .gpioId = buttonId };
    uint8_t sendMsg = FALSE;

    // Get current value of the button gpio after the clock timeout
    uint8_t buttonGpioVal = GPIO_read(buttonId);

    // Set interrupt direction to opposite of debounced state
    // If button is now released (button is active low, so release is high)

    if(buttonGpioVal)
    {
        // Enable negative edge interrupts to wait for press
        GPIO_setConfig(buttonId, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING | GPIO_CFG_INT_ENABLE);
    }
    else
    {
        // Enable positive edge interrupts to wait for relesae
        GPIO_setConfig(buttonId, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_RISING | GPIO_CFG_INT_ENABLE);
    }

    switch(buttonId)
    {
    case CONFIG_GPIO_BTN1:
        // If button is now released (buttonGpioVal is active low, so release is 1)
        // and button state was pressed (buttonstate is active high so press is 1)
        if(buttonGpioVal && button0State)
        {
            // Button was released
            buttonMsg.state = button0State = 0;
            sendMsg = TRUE;
        }
        else if(!buttonGpioVal && !button0State)
        {
            // Button was pressed
            buttonMsg.state = button0State = 1;
            sendMsg = TRUE;
        }
        break;

    case CONFIG_GPIO_BTN2:
        // If button is now released (buttonGpioVal is active low, so release is 1)
        // and button state was pressed (buttonstate is active high so press is 1)
        if(buttonGpioVal && button1State)
        {
            // Button was released
            buttonMsg.state = button1State = 0;
            sendMsg = TRUE;
        }
        else if(!buttonGpioVal && !button1State)
        {
            // Button was pressed
            buttonMsg.state = button1State = 1;
            sendMsg = TRUE;
        }
        break;
    }

    if(sendMsg == TRUE)
    {
        pzButtonState_t *pButtonState = ICall_malloc(sizeof(pzButtonState_t));
        if(pButtonState != NULL)
        {
            *pButtonState = buttonMsg;
            if(AchillesPB_enqueueMsg(PZ_BUTTON_DEBOUNCED_EVT, pButtonState) != SUCCESS)
            {
              ICall_free(pButtonState);
            }
        }
    }
}

/*********************************************************************
 * @fn      GPIO_Board_keyCallback
 *
 * @brief   Interrupt handler for Keys for GPIO++ module
 *
 * @param   none
 *
 * @return  none
 */
static void GPIO_Board_keyCallback(uint_least8_t index)
{
    Log_info1("Button interrupt: %s",
              (uintptr_t)((index == CONFIG_GPIO_BTN1) ? "Button 0" : "Button 1"));

    // Disable interrupt on that gpio for now. Re-enabled after debounce.
    GPIO_setConfig(index, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING | GPIO_CFG_INT_DISABLE);
    Clock_start(Clock_handle(&Notifyclock));
    // Start debounce timer
    switch(index)
    {
    case CONFIG_GPIO_BTN1:
        Util_startClock((Clock_Struct *)button0DebounceClockHandle);
        break;
    case CONFIG_GPIO_BTN2:
        Util_startClock((Clock_Struct *)button1DebounceClockHandle);
        break;
    }
}

/*********************************************************************
 * @fn      AchillesPB_notifySetdata
 *
 * @brief   Callback for observer notify 1Hz Frequency
 *          Manage charging and Button pannel cotifications / Indications
 *
 * @param
 * len - 12
 *
 */

static void notifySetdata(){

    if(Current_Status ==3 || Current_Status == 9){
       // powerOnFlag
        pulsecount = 0;
    }
    else{
        pulsecount +=1;
    }

    if(pulsecount > 1800){
        powerOnFlag = 0;
        buttonPresscountPW = 0;
        manualCount = 0;
        achilliesInit(2);
        buttonPresscount =0;
        stopFlag = 1;
        mannualOperationStop = 1;
        GapAdv_disable(advHandleLegacy);
        uint32_t bleState = 0;
        HWREG(0x40004000) = bleState;
        Log_info0("BLE DISabled");
        countTick = 0;

        sleepIndicateflag = false; //Sleep pulse

        GPIO_write(Heartbeat, 0);
        Task_sleep (10 * (1000 / Clock_tickPeriod));
        GPIO_write(Heartbeat, 1);
        Task_sleep (10 * (1000 / Clock_tickPeriod));
        GPIO_write(Heartbeat, 0);
        Task_sleep (10 * (1000 / Clock_tickPeriod));
        GPIO_write(Heartbeat, 1);

        GPIO_write(bukEnable, 0); //Turn OFF 5V buck
        /* Enable Power Policies */
        Power_enablePolicy();
        /* Configure DIO for wake up from shutdown */
        GPIO_setConfig(CONFIG_GPIO_WAKEUP, GPIO_CFG_IN_PD | GPIO_CFG_SHUTDOWN_WAKE_HIGH);
        /* Go to shutdown */
        Power_shutdown( 0, 0);
    }

    //batteryLevelApp =96;
    chargingStatus = GPIO_read(CONFIG_GPIO_BTN1);
    heaterAdc = scifTaskData.achButton.output.pAdcValue[1];
    Log_info1("Current Status : %d", Current_Status);
    Log_info1("batteryLevelApp : %d", batteryLevelApp);
//    uint32_t constraints = Power_getConstraintMask();
//    Log_info1("constraints : %d", constraints);e
    if(chargingStatus == 0){//device is charging
        //batCount = 0;
        Current_Status = 8;
        pulsecount = 0;
        powerOnFlag = 0;
        chargeStaetFlag = 1;
        Clock_start(Clock_handle(&i2cCommunicateClock));
        GPIO_write(bukEnable, 1); //Turn On 5V buck
        Task_sleep (100 * (1000 / Clock_tickPeriod));
        buttonPresscount =0;
        PWM_stop(PWM1);
        GPIO_write(irOutput, 0);
        stopFlag = 1;
        Clock_stop(Clock_handle(&stopClock));
        countTick = 0;
        if(batteryLevelApp < 30){

            WS2812_setPixelColor(0, 0, 0, 0);
            WS2812_setPixelColor(1, 0, 0, 0);
            WS2812_setPixelColor(2, 0xFF, 0xC0, 0);
            WS2812_show();
            Task_sleep (100 * (1000 / Clock_tickPeriod));
            onLEDs(1,0,0, 0 );
           }
        else if(batteryLevelApp < 60 && batteryLevelApp > 30 ){

            WS2812_setPixelColor(0, 0, 0, 0);
            WS2812_setPixelColor(1, 0xFF, 0xC0, 0);
            WS2812_setPixelColor(2, 0xFF, 0xC0, 0);

            WS2812_show();
            Task_sleep (100 * (1000 / Clock_tickPeriod));
            WS2812_setPixelColor(1, 0, 0, 0);
            WS2812_show();
        }

        else if(batteryLevelApp < 95 && batteryLevelApp > 60 ){
            WS2812_setPixelColor(0, 0xFF, 0xC0, 0);
            WS2812_setPixelColor(1, 0xFF, 0xC0, 0);
            WS2812_setPixelColor(2, 0xFF, 0xC0, 0);
            WS2812_show();
            Task_sleep (100 * (1000 / Clock_tickPeriod));
            WS2812_setPixelColor(0, 0, 0, 0);
            WS2812_show();
        }

        else if(batteryLevelApp > 95){//sahan add this.when unplug the charging, it continuosly blink the bulb
            WS2812_setPixelColor(0, 0x3B,0xFF, 0x09);
            WS2812_setPixelColor(1, 0x3B,0xFF, 0x09);
            WS2812_setPixelColor(2, 0x3B,0xFF, 0x09);
            WS2812_show();
            Task_sleep (100 * (1000 / Clock_tickPeriod));
            WS2812_setPixelColor(0, 0,0, 0);
            WS2812_setPixelColor(1, 0,0, 0);
            WS2812_setPixelColor(2, 0,0, 0);
            WS2812_show();
            pulsecount = 0;
        }
        //setThermalReg( i2c,  i2cTransaction, txBuffer,  rxBuffer);
    }
    else if(chargingCompletionFlag == 1){
            WS2812_setPixelColor(0, 0x3B,0xFF, 0x09);
            WS2812_setPixelColor(1, 0x3B,0xFF, 0x09);
            WS2812_setPixelColor(2, 0x3B,0xFF, 0x09);
            WS2812_show();
            Task_sleep (100 * (1000 / Clock_tickPeriod));
            WS2812_setPixelColor(0, 0,0, 0);
            WS2812_setPixelColor(1, 0,0, 0);
            WS2812_setPixelColor(2, 0,0, 0);
            WS2812_show();
            chargingCompletionFlag = 0;
    }
    else if(chargeStaetFlag == 1 && chargingStatus == 1 ){
        onLEDs(1,0,0, 0 );
        chargeStaetFlag = 0;
        Current_Status = 1;
        //Clock_stop(Clock_handle(&i2cCommunicateClock));
//        Clock_stop(Clock_handle(&Notifyclock));
//        Clock_stop(Clock_handle(&buttonsenseClock));
//
//        GPIO_write(bukEnable, 0); //Turn OFF 5V buck
//
//        /* Enable Power Policies */
//        Power_enablePolicy();
//        /* Configure DIO for wake up from shutdown */
//        GPIO_setConfig(CONFIG_GPIO_WAKEUP, GPIO_CFG_IN_PD | GPIO_CFG_SHUTDOWN_WAKE_HIGH);
//        /* Go to shutdown */
//        Power_shutdown( 0, 0);
    }
    Status[0] = Current_Status;
    Status[1] = Current_Protocol;
    Status[2] = Set_Time_01 ;
    Status[3] = Set_Time_02 ;
    Status[4] = Remaining_Time_01 ;
    Status[5] = Remaining_Time_02 ;
    Status[6] = Current_Temperature_02 ;
    Status[7] = Current_Temperature_01 ;
    Status[8] = Error_Code;
    Status[9] = Device_Cali_Status ;
    Status[10] = batteryLevelApp;
    Status[11] = Status_CRC;
    ObserverNotify_SetParameter( OBSERVERNOTIFY_NOTIFYCHRAC_ID, sizeof(Status), &Status );

//    if(pulsecount <= 10 && bleflag == 0 && Current_Status < 8 && powerOnFlag == 1 && chargingStatus != 0){
//           onLEDs(1,0,0, 0xFF );
//           Task_sleep (100 * (1000 / Clock_tickPeriod));
//           onLEDs(1,0,0, 0 );
//            //pulsecount = 0;
//       }


//    if(batteryLevelApp <= 30 && batteryLowFLag30 !=1 && chargingStatus != 0 &&powerOnFlag !=0 ){
//        batteryLowFLag30 = 1;
//        for(int x =0; x<=4;x++){
//            WS2812_setPixelColor(0, 0xFF, 0xC0, 0);
//            WS2812_setPixelColor(1, 0xFF, 0xC0, 0);
//            WS2812_setPixelColor(2, 0xFF, 0xC0, 0);
//            WS2812_show();
//            Task_sleep (100 * (1000 / Clock_tickPeriod));
//            WS2812_setPixelColor(0, 0,0, 0);
//            WS2812_setPixelColor(1, 0,0, 0);
//            WS2812_setPixelColor(2, 0,0, 0);
//            WS2812_show();
//            }
//        }
//    else if(batteryLevelApp <= 15 && batteryLowFLag15 !=1 && chargingStatus != 0 &&powerOnFlag !=0 ){
//        batteryLowFLag15 = 1;
//        for(int x =0; x<=4;x++){
//
//                        WS2812_setPixelColor(0, 0xFF, 0xC0, 0);
//                        WS2812_setPixelColor(1, 0xFF, 0xC0, 0);
//                        WS2812_setPixelColor(2, 0xFF, 0xC0, 0);
//                        WS2812_show();
//                        Task_sleep (100 * (1000 / Clock_tickPeriod));
//                        WS2812_setPixelColor(0, 0,0, 0);
//                        WS2812_setPixelColor(1, 0,0, 0);
//                        WS2812_setPixelColor(2, 0,0, 0);
//                        WS2812_show();
//
//            }
//        }
//    else if(batteryLevelApp <= 5 && batteryLowFLag5 !=1 && chargingStatus != 0 &&powerOnFlag !=0 ){
//            batteryLowFLag5 = 1;
//            for(int x = 0; x<=4;x++){
//
//                            WS2812_setPixelColor(0, 0xFF, 0xC0, 0);
//                            WS2812_setPixelColor(1, 0xFF, 0xC0, 0);
//                            WS2812_setPixelColor(2, 0xFF, 0xC0, 0);
//                            WS2812_show();
//                            Task_sleep (100 * (1000 / Clock_tickPeriod));
//                            WS2812_setPixelColor(0, 0,0, 0);
//                            WS2812_setPixelColor(1, 0,0, 0);
//                            WS2812_setPixelColor(2, 0,0, 0);
//                            WS2812_show();
//
//            }
//        }
    if(Error_Code !=4 && Error_level == 3 && errorFLag != 1 && (Current_Status ==3 || Current_Status == 9)){
        Current_Status = 5;
        mannualOperationStop = 1;
        errorFLag = 1;
        stopFlag =1;
        Clock_stop(Clock_handle(&stopClock));
        countTick = 0;
                for(int x =0; x<=3;x++){
                    onLEDs(1,0xFF,0xFF, 0xFF );
                    Task_sleep (250 * (1000 / Clock_tickPeriod));
                    onLEDs(1,0,0, 0 );
                    Task_sleep (250 * (1000 / Clock_tickPeriod));
                }
            }
    Watchdog_clear(watchdogHandle);
    //GPIO_toggle(Heartbeat);
    if(fail_safe_fet_on && sleepIndicateflag){
               GPIO_toggle(Heartbeat);
    }
}

/* Profile value change handlers */
void user_runHeater_ValueChangeHandler(pzCharacteristicData_t *pData)
{
  switch (pData->paramID)
  {
        case RUNHEATER_RUNHEATER_ID:
        Log_info0("Value Change msg for runHeater :: runHeater received");
        heater01 = pData;
                for (int i = 0; i < 6; i++)
                    {
                        runheatr[i] = heater01->data[i];
                        Log_info1("runheater %u",runheatr[i]);
                    }
                Current_Status = 3;
                switch(runheatr[1]){
                        case 35:
                            WS2812_setPixelColor(0, 0, 0, 0);
                            WS2812_setPixelColor(1, 0, 0, 0);
                            WS2812_setPixelColor(2, 0xFF, 0, 0);

                            WS2812_show();
                            break;
                        case 40:
                            WS2812_setPixelColor(0, 0, 0, 0);
                            WS2812_setPixelColor(1, 0xFF, 0, 0);
                            WS2812_setPixelColor(2, 0xFF, 0, 0);
                            WS2812_show();
                            break;
                        case 45:
                            WS2812_setPixelColor(0, 0xFF, 0, 0);
                            WS2812_setPixelColor(1, 0xFF, 0, 0);
                            WS2812_setPixelColor(2, 0xFF, 0, 0);
                            WS2812_show();
                            break;
                        }
                runheater(runheatr);
        break;
      }

}
/* Profile value change handlers */
void user_pauseHeater_ValueChangeHandler(pzCharacteristicData_t *pData)
{
  switch (pData->paramID)
  {
        case PAUSEHEATER_PAUSEHEATER_ID:
        Log_info0("Value Change msg for pauseHeater :: pauseHeater received");
        int PauseVal = pData->data[1];
        if (PauseVal==1){
            PWM_stop(PWM1);
            GPIO_write(irOutput, 0);
            pauseFlag = 1;
            stopFlag=1;
            Log_info0("Heater Paused ");
            AchillesPB_enqueueMsg(AC_NTFY_CLK_EVT, NULL);
            Clock_stop(Clock_handle(&stopClock));
            Clock_stop(Clock_handle(&protocolStartHeatingClock));
          //  I2CWriteMSP430(11);
            Current_Status = 6 ;
            countTick = 0;
          }

        break;
      }
}

/*********************************************************************
 * @fn      runheater
 *
 * @brief   Create Treatement run tasks, set treatment parameters timers, heat levels etc....
 *
 * @param   runheatr[6] : Treatement data receved through BLE or Button pannel
 *
 * @return  none
 */

static void runheater(uint8 runheatr[6]){
    LEDFlag=0;//for switch on IR pannel
    protclIndx = runheatr[0];
    heatLvl    = runheatr[1];
    time1      = runheatr[2];
    time2      = runheatr[3];
    IRLgt      = runheatr[4];
    RunType    = runheatr[5];
    Log_info1("protocol_index: %d",protclIndx);
    Log_info1("protocol_heat: %d",heatLvl);
    Log_info1("protocol_time1: %d",time1);
    Log_info1("protocol_time2: %d",time2);
    Log_info1("protocol_ir: %d",IRLgt);
    Log_info1("protocol_run type: %d",RunType);
    Set_Time_01 = time1 ;
    Set_Time_02 = time2 ;
    Remaining_Time_01 = time1 ;
    Remaining_Time_02 = time2 ;

    heatMax = 100 ;
    heatTem = heatLvl;
    PWMratio1 = heatLvl;

    if(heatLvl<=20){
        heatLevelIndication=5;
    }else if(heatLvl<=30){
        heatLevelIndication=6;
    }else{
        heatLevelIndication=7;
    }

    clock_cunt2=0;//stop clock condition
    stopClockTime  =   (((Set_Time_01 * 255) + Set_Time_02) * 1000);
    Log_info1("protocol_run time: %d",stopClockTime);
    Clock_setPeriod (Clock_handle (&stopClock),stopClockTime *(1000/Clock_tickPeriod));
    Log_info1("Stop Clock Time Is: %u",stopClockTime);
    Log_info0("clock set period");
    Clock_start(Clock_handle(&stopClock));
    Clock_start(Clock_handle(&i2cCommunicateClock));
    if(RunType==3){
        Current_status=2;
    }else if(RunType==2){
        Current_status=3;
        AchillesPB_enqueueMsg(AC_NTFY_CLK_EVT, NULL);
    }
    PWM_start(PWM1);
    GPIO_write(irOutput, 1);
    LEDFlag = 1;
    if(IRLgt==1){
        LEDFlag = 1;
    }
    if(LEDFlag == 1){
        GPIO_write(irOutput, 1);
    }
    stopFlag=0;
    pauseFlag=0;


    achiliesTaskID=3;//mobile app control the protocol
    ACHILLIES_taskFunction();

}
/* Task construct  : Supportive tasks */
void ACHILLIES_taskFunction (void){
    Log_info0("task Function Create");
    Task_Params taskParams1;
    Task_Params_init (&taskParams1);
    taskParams1.stack = sbcTaskStack;
    taskParams1.stackSize = SBS_TASK_STACK_SIZE;
    taskParams1.priority = sbcTaskS_PRIORITY;
    Task_construct (&task0, stateCreateTaskFunction, &taskParams1, NULL);
}

/* TaskFunction : Supportive tasks */
static void stateCreateTaskFunction(UArg arg0, UArg arg1){
    if( achiliesTaskID==3){
        Task_sleep (100 * (1000 / Clock_tickPeriod));
        Log_info0("State Create");
        dutyValue1 = (uint32_t) (((uint64_t) PWM_DUTY_FRACTION_MAX * 0) / 100);
        PWM_setDuty(PWM1, dutyValue1);  // set duty cycle
        PWM_start(PWM1);
        achiliesProtocolSelectTask();
    }

}

/*********************************************************************
 * @fn      achiliesProtocolSelectTask
 *
 * @brief   select protocols selected by the user and execute
 *
 * @param   none
 *
 * @return  none
 */
static void achiliesProtocolSelectTask(){
    Log_info0("Protocol Select");
    if (protclIndx == 1){
      //  I2CWriteMSP430(8);
        Task_sleep (100 * (1000 / Clock_tickPeriod));
        protocol_1(heatTem );

    }
    else if (protclIndx == 2){
        //I2CWriteMSP430(9);
        Task_sleep (100 * (1000 / Clock_tickPeriod));
        protocol_2(heatTem);
    }else if (protclIndx == 3){
        //I2CWriteMSP430(10);
        Task_sleep (100 * (1000 / Clock_tickPeriod));
         protocol_3(heatTem);
    }else{
        Log_info0("Invalid Input");
    }
    stopFlag=0;
    mannualOperationStop=0;
    continueFlag=0;
    PWM_stop(PWM1);
    GPIO_write(irOutput, 0);
    Log_info0("IR off Protocol select end exe");
    onLEDs(1,0,0, 0 );
    manualCount = 0;
    Task_exit();
}

/*********************************************************************
 * @fn      protocol_1
 *
 * @brief   Protocol 01 executes here
 *
 * @param   heatTem
 *
 * @return  none
 */

void
protocol_1(uint8_t heatTem){
    Log_info0("Protocol 1 starts");
    Log_info1("heatTem: %d",heatTem);
    Log_info1("temp1: %d",temp1);
    stopFlag=0;
    pauseFlag=0;
    Current_Protocol = 1;
    //Current_Status = 3;
    PWMratio1 = 60;
    constantPWM = 60;
    dutyValue1 = (uint32_t) (((uint64_t) PWM_DUTY_FRACTION_MAX * 70) / 100);
    PWM_setDuty(PWM1, dutyValue1);  // set duty cycle
    PWM_start(PWM1);

//    Log_info0("Initial Heating start");
//    while(temp1<heatTem){
//        Task_sleep (100 * (1000 / Clock_tickPeriod));
//        //Log_info0("Initial Heating.....");
//        if(pauseFlag==1 || stopFlag==1 || temp2>=heatTem + 5 || temp1>=heatTem){
//            break;
//        }
//    }
//    Log_info0("Initial Heating end");
   if(pauseFlag!=1 || stopFlag!=1){
       startCount=0;
       Task_sleep (10 * (1000 / Clock_tickPeriod));
    //   I2CWriteMSP430(heatLevelIndication);
       Task_sleep (10 * (1000 / Clock_tickPeriod));
       setPointTemp = heatTem;
       HeaterPID(70, (setPointTemp+5));
       //PID();

   }else{
       Log_info1("stop Flag :%d",stopFlag);
       Log_info1("pause Flag :%d",pauseFlag);
   }
   Clock_stop(Clock_handle(&stopClock));
    Log_info0("Protocol 1 finished");
    stopFlag=0;
    Task_sleep (10 * (1000 / Clock_tickPeriod));
    onLEDs(1,0,0, 0 );
}

/*********************************************************************
 * @fn      protocol_2
 *
 * @brief   Protocol 02 executes here
 *
 * @param   heatTem
 *
 * @return  none
 */
void
protocol_2(uint8_t heatTem){
    Log_info0("Protocol 2 starts");
    stopFlag=0;
      pauseFlag=0;
      Current_Protocol = 1;
      //Current_Status = 3;
      Clock_start(Clock_handle(&protocolStartHeatingClock));
      PWMratio1 = 50;
      constantPWM = 50;
      dutyValue1 = (uint32_t) (((uint64_t) PWM_DUTY_FRACTION_MAX * 70) / 100);
      PWM_setDuty(PWM1, dutyValue1);  // set duty cycle

//      while(temp1<heatTem){
//          Task_sleep (10 * (1000 / Clock_tickPeriod));
//          if(pauseFlag==1 || stopFlag==1 || temp2>=heatTem + 5 || temp1>=heatTem){
//              break;
//          }
//      }
      Log_info0("start clock stopped");
      Clock_stop(Clock_handle(&protocolStartHeatingClock));
     if(pauseFlag!=1 || stopFlag!=1){
         startCount=0;
         Task_sleep (10 * (1000 / Clock_tickPeriod));
   //      I2CWriteMSP430(heatLevelIndication);
         Log_info0("Heat Level Indicated");
         Task_sleep (10 * (1000 / Clock_tickPeriod));
         setPointTemp = heatTem;
         HeaterPID(60, (setPointTemp+5));
         //PID();
     }else{
         Log_info1("stop Flag :%d",stopFlag);
         Log_info1("pause Flag :%d",pauseFlag);
     }
     Clock_stop(Clock_handle(&stopClock));

      Log_info0("Protocol 2 finished");
      stopFlag=0;
      Task_sleep (10 * (1000 / Clock_tickPeriod));
      Clock_start(Clock_handle(&i2cCommunicateClock));
      Task_sleep (10 * (1000 / Clock_tickPeriod));
      onLEDs(1,0,0, 0 );
}


/*********************************************************************
 * @fn      protocol_3
 *
 * @brief   Protocol 03 executes here
 *
 * @param   heatTem
 *
 * @return  none
 */
void protocol_3(uint8_t heatTem){
    Log_info0("Protocol 3 starts");
    stopFlag=0;
      pauseFlag=0;
     Current_Protocol = 1;
      //Current_Status = 3 ;
      Clock_start(Clock_handle(&protocolStartHeatingClock));
      PWMratio1 = 40;
      constantPWM = 40;
      dutyValue1 = (uint32_t) (((uint64_t) PWM_DUTY_FRACTION_MAX * 70) / 100);
      PWM_setDuty(PWM1, dutyValue1);  // set duty cycle
//      while(startCount<10){
//          Task_sleep (10 * (1000 / Clock_tickPeriod));
//          if(pauseFlag==1 || stopFlag==1){
//              break;
//          }
//
//      }
//      while(temp1<heatTem){
//          Task_sleep (10 * (1000 / Clock_tickPeriod));
//          if(pauseFlag==1 || stopFlag==1 || temp2>=heatTem + 5 || temp1>=heatTem){
//              break;
//          }
//      }
      Log_info0("start clock stopped");
      Clock_stop(Clock_handle(&protocolStartHeatingClock));
     if(pauseFlag!=1 || stopFlag!=1){
         startCount=0;
         Task_sleep (10 * (1000 / Clock_tickPeriod));
         Log_info0("Heat Level Indicated");
         Task_sleep (10 * (1000 / Clock_tickPeriod));
         setPointTemp = heatTem;
         HeaterPID(50, (setPointTemp+5));
         //PID();
     }else{
         Log_info1("stop Flag :%d",stopFlag);
         Log_info1("pause Flag :%d",pauseFlag);
     }
     Clock_stop(Clock_handle(&stopClock));
      Log_info0("Protocol 3 finished");
      stopFlag=0;
      Clock_start(Clock_handle(&i2cCommunicateClock));
      Task_sleep (10 * (1000 / Clock_tickPeriod));
      onLEDs(1,0,0, 0 );
}

/*********************************************************************
 * @fn      PID - Updated
 *
 * @brief   PID controller for the heater
 *
 * @param   Max PWM Duty ratio
 *
 * @return  none
 */
static void HeaterPID(uint16_t maxDuty, uint16_t maxHeaterTemp){
    Log_info0("PID control Starts");
//    Current_Temperature_01 = temp1;//body temp
//    Current_Temperature_02 = temp2;//wire temp

    float PropoError = 0 ;
    float IntegrateError = 0 ;
    float DerivativeError = 0 ;
    float PreviousError  = 0 ;
    float Error = 0;
    float maxError = 15;


    float Kp = 7;
    float Ki = 0.0015;
    float Kd = 2;

    Log_info1("Stop Flag :%d",stopFlag);
    Task_sleep (10 * (1000 / Clock_tickPeriod));
    while(stopFlag!=1 || mannualOperationStop!=1){
        Task_sleep (10 * (1000 / Clock_tickPeriod));
        PropoError = setPointTemp - temp1;
        IntegrateError = IntegrateError + PropoError ;
        PreviousError = PropoError ;
        DerivativeError = PropoError - PreviousError ;

        Error = Kp*PropoError + Ki*IntegrateError + Kd*DerivativeError;
       // Log_info1("Error :%d",Error*1000);
       // Log_info1("dutyCycle :%d",dutyCycle);
        if (temp2>maxHeaterTemp){
            PWM_setDuty(PWM1, 0);
        }
        else if (Error>=0){

            dutyCycle = (Error*100)/maxError;

            if (dutyCycle >= maxDuty){
                dutyCycle = maxDuty;
            }

            //dutyValue = (uint32) (((uint64) PWM_DUTY_FRACTION_MAX * dutyCycle) / 100);
            uint64_t dutyValue = (uint32_t) (((uint64_t) PWM_DUTY_FRACTION_MAX * dutyCycle) / 100);
            PWM_setDuty(PWM1, dutyValue);
        }

        else{
            PWM_setDuty(PWM1, 0);
            dutyCycle = 0;
        }
        if(stopFlag==1 || mannualOperationStop==1){
             PWM_setDuty(PWM1, 0);
             GPIO_write(irOutput, 0);
             return;
                     }
        Task_sleep (100 * (1000 / Clock_tickPeriod)); //have to reduce at end
    }
  Log_info0("while loop exits");
}



/*********************************************************************
 * @fn      PID
 *
 * @brief   PID controller for the heater
 *
 * @param   none
 *
 * @return  none
 */
static void PID(){
  Log_info0("PID control Starts");
  int count=0;
  int pre_temp=0;
  Current_Temperature_01 = temp1;//body temp
  Current_Temperature_02 = temp2;//wire temp

  float PropoError = 0 ;
  float IntegrateError = 0 ;
  float DerivativeError = 0 ;
  float PreviousError  = 0 ;
  float Error = 0;


  float Kp = 0.5 ;
  float Ki = 0 ;
  float Kd = 0 ;
  Log_info1("Stop Flag :%d",stopFlag);
  Task_sleep (10 * (1000 / Clock_tickPeriod));
  while(stopFlag!=1 || mannualOperationStop!=1){
      Task_sleep (100 * (1000 / Clock_tickPeriod));
      Current_Temperature_01 = temp1;//body temp
      Current_Temperature_02 = temp2;//wire temp

      if(temp1>setPointTemp){
          PWM_setDuty(PWM1, 0);
          //GPIO_write(irOutput, 0);
          if(pre_temp>setPointTemp && temp1>setPointTemp){
              count+=1;
          }else{
              count=0;
          }
          if(count>10){
              PWM_setDuty(PWM1, 0);
              //GPIO_write(irOutput, 0);
         }

      }

      pre_temp=temp1;
      PropoError = setPointTemp - temp2 ;
      IntegrateError = IntegrateError + PropoError ;
      DerivativeError = PropoError - PreviousError ;
      PreviousError = PropoError ;

     Error = Kp*PropoError + Ki*IntegrateError + Kd*DerivativeError ;
     if(temp2>=setPointTemp+5){
         PWM_setDuty(PWM1, 0);
         //GPIO_write(irOutput, 0);
         while(1){
             Log_info0("Temperature is High");
             Task_sleep (100 * (1000 / Clock_tickPeriod));
             if(temp2<setPointTemp+3){
                 break;
             }
             if(temp1>setPointTemp){
                 PWM_setDuty(PWM1, 0);
                 //GPIO_write(irOutput, 0);
             }
             if(stopFlag==1 || mannualOperationStop==1){
                 PWM_setDuty(PWM1, 0);
                 GPIO_write(irOutput, 0);
                 Log_info0("IR off PID 1");
                 return;
             }
         }
     }
         pre_temp=temp1;
         PropoError = setPointTemp - temp2 ;
         IntegrateError = IntegrateError + PropoError ;
         DerivativeError = PropoError - PreviousError ;
         PreviousError = PropoError ;
         Error = Kp*PropoError + Ki*IntegrateError + Kd*DerivativeError ;
         if(Error <= 0){
             while(Error <= 0){
                 Task_sleep (100 * (1000 / Clock_tickPeriod));
                 Current_Temperature_01 = temp1;//wire temp
                 Task_sleep (100 * (1000 / Clock_tickPeriod));
                 Current_Temperature_02 = temp2;//body temp
                 Task_sleep (100 * (1000 / Clock_tickPeriod));
                 if(temp1>setPointTemp){
                     PWM_setDuty(PWM1, 0);
                     //GPIO_write(irOutput, 0);
                     if(pre_temp>setPointTemp && temp1>setPointTemp){
                         count+=1;
                     }else{
                         count=0;
                     }
                     if(count>10){
                         PWM_setDuty(PWM1, 0);
                         //GPIO_write(irOutput, 0);
                     }
                 }
                 if(temp2>=setPointTemp+5){
                     PWM_setDuty(PWM1, 0);
                     //GPIO_write(irOutput, 0);
                     while(1){
                         Log_info0("Temperature is High");
                         Task_sleep (100 * (1000 / Clock_tickPeriod));
                         if(temp2<setPointTemp+3){
                            // GPIO_write(irOutput, 0);
                             PWM_setDuty(PWM1, 0);
                             break;
                         }
                         if(temp1>setPointTemp){
                             PWM_setDuty(PWM1, 0);
                             //GPIO_write(irOutput, 0);
                         }
                         if(stopFlag==1 || mannualOperationStop==1){
                             PWM_setDuty(PWM1, 0);
                             GPIO_write(irOutput, 0);
                             Log_info0("IR off PID 2");
                             return;
                          }
                       }
                   }
                   pre_temp=temp1;
                   PropoError = setPointTemp - temp2 ;
                   IntegrateError = IntegrateError + PropoError ;
                   DerivativeError = PropoError - PreviousError ;
                   PreviousError = PropoError ;
                   Error = Kp*PropoError + Ki*IntegrateError + Kd*DerivativeError ;
                   if(Error>0){
                       PWM_setDuty(PWM1, 0);
                      // GPIO_write(irOutput, 0);
                       break;
                   }
                   PWMratio1 = constantPWM - Error ;
                   Log_info1("pwm ratio :%d",PWMratio1);
                   dutyValue1 = (uint32_t) (((uint64_t) PWM_DUTY_FRACTION_MAX * PWMratio1) / 100);
                   PWM_setDuty(PWM1, dutyValue1);  // set duty cycle
                   if(stopFlag==1 || mannualOperationStop==1){
                       PWM_setDuty(PWM1, 0);
                       GPIO_write(irOutput, 0);
                       Log_info0("IR off PID 3");
                       return;
                   }
                }
             }
             if (Error > 0){
                 while(Error>0){
                     Task_sleep (100 * (1000 / Clock_tickPeriod));
                     Current_Temperature_01 = temp1;//wire temp
                     Task_sleep (100 * (1000 / Clock_tickPeriod));
                     Current_Temperature_02 = temp2;//body temp
                     Task_sleep (100 * (1000 / Clock_tickPeriod));
                     if(temp2>setPointTemp){
                         PWM_setDuty(PWM1, 0);
                         //GPIO_write(irOutput, 0);
                         if(pre_temp>setPointTemp && temp1>setPointTemp){
                             count+=1;
                         }else{
                             count=0;
                         }
                         if(count>10){
                             PWM_setDuty(PWM1, 0);
                             //GPIO_write(irOutput, 0);
                         }
                     }
                     if(temp2>=setPointTemp+5){
                         PWM_setDuty(PWM1, 0);
                         //GPIO_write(irOutput, 0);
                         while(1){
                             Log_info0("Temperature is High");
                             Task_sleep (100 * (1000 / Clock_tickPeriod));
                             if(temp2<setPointTemp+3){
                                 PWM_setDuty(PWM1, 0);
                                 //GPIO_write(irOutput, 0);
                                 break;
                             }
                             if(temp1>setPointTemp){
                                 PWM_setDuty(PWM1, 0);
                                 //GPIO_write(irOutput, 0);
                              }
                              if(stopFlag==1 || mannualOperationStop==1){
                                  PWM_setDuty(PWM1, 0);
                                  GPIO_write(irOutput, 0);
                                  Log_info0("IR off PID 4");
                                  return;
                              }
                         }
                     }
                     pre_temp=temp1;
                     PropoError = setPointTemp - temp2 ;
                     IntegrateError = IntegrateError + PropoError ;
                     DerivativeError = PropoError - PreviousError ;
                     PreviousError = PropoError ;
                     Error = Kp*PropoError + Ki*IntegrateError + Kd*DerivativeError ;
                     if(Error<0){
                         break;
                     }
                     PWMratio1 = constantPWM + Error ;
                     Log_info1("pwm ratio :%d",PWMratio1);
                     dutyValue1 = (uint32_t) (((uint64_t) PWM_DUTY_FRACTION_MAX * PWMratio1) / 100);
                     PWM_setDuty(PWM1, dutyValue1);  // set duty cycle
                     if(stopFlag==1 || mannualOperationStop==1){
                         PWM_setDuty(PWM1, 0);
                         GPIO_write(irOutput, 0);
                         Log_info0("IR off PID 5");
                         return;
                     }
                 }
             }
  }
  Log_info0("while loop exits");
}


/*********************************************************************
 * @fn      i2cCommClockSwiFxn
 *
 * @brief   I2C Clock sfotware interrupt function
 *
 * @param   none
 *
 * @return  none
 */
static void i2cCommClockSwiFxn (UArg buttonId){

    AchillesPB_enqueueMsg(PZ_MSG_I2C_TIMER, NULL);
}

/*********************************************************************
 * @fn      prsButtondatSWI
 *
 * @brief   Push button  Clock sfotware interrupt function
 *
 * @param   none
 *
 * @return  none
 */
static void prsButtondatSWI (){

    AchillesPB_enqueueMsg(PZ_MSG_button_sense, NULL);
}

/*********************************************************************
 * @fn      protocolStartClockSwiFxn
 *
 * @brief   Treatement Clock sfotware interrupt function : Calculate remaining time, timeout treatement
 *
 * @param   none
 *
 * @return  none
 */

static void protocolStartClockSwiFxn (){
    startCount+=1;
   // batteryLevelApp = 65;
    if(batteryLevelApp < 30){
        batteryLevel = 1;
       }
    else if(batteryLevelApp < 60 && batteryLevelApp > 30 ){
        batteryLevel = 2;
    }

    else if(batteryLevelApp < 90 && batteryLevelApp > 60 ){
        batteryLevel = 3;
    }

    else if(batteryLevelApp > 90){
        batteryLevel = 4;
    }
    Log_info0("while loop exits");

}

/*********************************************************************
 * @fn      prsButtondata
 *
 * @brief   Push button Input handler, Power On, Power Off, LED Indication
 *
 * @param   none
 *
 * @return  none
 */

static void prsButtondata(){
//    uint16_t xcurrentADC = scifTaskData.achButton.output.pAdcValue[0];
    buttonID = scifTaskData.achButton.state.buttonState;
//    uint16_t xchargingStatus = GPIO_read(CONFIG_GPIO_WAKEUP);
//    Log_info1("Button : %d", buttonID);
//    Log_info1("Button ADC : %d", xcurrentADC);
//    Log_info1("chargingStatus : %d", xchargingStatus);
    if(buttonID ==1){
        //buttonPresscount += 1;
        buttonPresscountPW += 1;
        buttonPresssState = 0;
    }
    else if(buttonID ==2){
            buttonPresscount += 1;
            //buttonPresscountPW += 1;
            buttonPresssState = 0;
        }

    else if(buttonID ==3){
        BLEbuttonPress += 1;
        }
    else{
        BLEbuttonPress = 0;
        buttonPresscount = 0;
        buttonPresscountPW = 0;
        manulaOnflag = 0;
        buttonPresssState += 1;;
    }

    if(BLEbuttonPress ==2 && powerOnFlag !=0){
        bStatus_t result3 = GAP_TerminateLinkReq(0x0000, HCI_DISCONNECT_REMOTE_USER_TERM);
        Log_info0("Disconnected from all connected devices");
        delayFun (100);
        GapAdv_disable(advHandleLegacy);
        Log_info0("BLE DISabled");
        delayFun (200);
        GapAdv_enable(advHandleLegacy, GAP_ADV_ENABLE_OPTIONS_USE_MAX, 0);
        pixelNitify =1;
        Log_info0("BLE Re-enabled");
        for(int x =0; x<=6;x++){
            onLEDs(1,0,0, 0xFF );
            delayFun (250);
            onLEDs(1,0,0, 0 );
            delayFun (250);
        }
        pixelNitify = 0;

        }
    if(buttonPresscountPW ==20 && powerOnFlag ==0 && chargingStatus != 0){
        //power on
        Log_info0("Device Powered ON");
        fail_safe_fet_on=true;
        achilliesInit(1);
        delayFun (500);
        batCount = 0;
        batteryLevelApp = BQ25887();
        batteryIndication(0);
        delayFun (1000);
        onLEDs(1,0,0, 0 );
        delayFun (500);
        Clock_start(Clock_handle(&i2cCommunicateClock));
        Clock_start(Clock_handle(&Notifyclock));
        powerOnFlag = 1;
        buttonPresscount = 0;
        buttonPresscountPW = 0;
        stopFlag = 0;
        mannualOperationStop = 0;
        dutyValue1 = (uint32_t) (((uint64_t) PWM_DUTY_FRACTION_MAX * 99) / 100);   // make here PWM value equal to 0
        PWM_setDuty(PWM1, dutyValue1);  // set duty cycle
        for(int x =0; x<=6;x++){
                    onLEDs(1,0,0, 0xFF );
                    delayFun (250);
                    onLEDs(1,0,0, 0 );
                    delayFun (250);
                }
//        TMP117init(2 , 0x48);
//        delayFun (100);
//        TMP117init(2 , 0x49);
        pulsecount = 0;
        Error_Code = 4;
        Error_level = 0;
        errorFLag = 1;
        Current_Status = 2;
        initialCheck();
    }
    if(buttonPresscountPW ==1 && powerOnFlag !=0 && chargingStatus != 0){
        batteryIndication(0);
        delayFun (1000);
        onLEDs(1,0,0, 0 );
    }

    else if(buttonPresscountPW ==30 && powerOnFlag !=0){
        Log_info0("Device Powered OFF");
        powerOnFlag = 0;
        buttonPresscountPW = 0;
        manualCount = 0;
        batteryLowFLag30 = 0;
        batteryLowFLag15 = 0;
        batteryLowFLag5 = 0;
        achilliesInit(2);
        Clock_stop(Clock_handle(&stopClock));
        //Clock_stop(Clock_handle(&i2cCommunicateClock));
//        Clock_stop(Clock_handle(&Notifyclock));
        buttonPresscount =0;
        PWM_stop(PWM1);
        GPIO_write(irOutput, 0);
        stopFlag = 1;
        mannualOperationStop = 1;
        GapAdv_disable(advHandleLegacy);
        uint32_t bleState = 0;
        HWREG(0x40004000) = bleState;
        Log_info0("BLE DISabled");
        countTick = 0;
//        TMP117init(1 , 0x48);
//        TMP117init(1 , 0x49);
//        Clock_stop(Clock_handle(&buttonsenseClock));
        sleepIndicateflag = false; //Sleep pulse

        GPIO_write(Heartbeat, 0);
        Task_sleep (10 * (1000 / Clock_tickPeriod));
        GPIO_write(Heartbeat, 1);
        Task_sleep (10 * (1000 / Clock_tickPeriod));
        GPIO_write(Heartbeat, 0);
        Task_sleep (10 * (1000 / Clock_tickPeriod));
        GPIO_write(Heartbeat, 1);

        GPIO_write(bukEnable, 0); //Turn On 5V buck
        /* Enable Power Policies */
        Power_enablePolicy();
        /* Configure DIO for wake up from shutdown */
        GPIO_setConfig(CONFIG_GPIO_WAKEUP, GPIO_CFG_IN_PD | GPIO_CFG_SHUTDOWN_WAKE_HIGH);
        /* Go to shutdown */
        Power_shutdown( 0, 0);
    }

    else if(buttonPresscount == 1 && powerOnFlag !=0 && Current_Status != 3 && manulaOnflag ==0 && chargingStatus != 0){
        //Manual Run LOW
        //pulsecount = 0;
        manulaOnflag = 1;
        buttonPresscount = 0;
        if(Current_Status !=9){
        PWM_stop(PWM1);
        GPIO_write(irOutput, 0);
        stopFlag = 1;
        Clock_stop(Clock_handle(&stopClock));
        countTick = 0;
        mannualOperationStop = 1;
        }
        switch(manualCount){
        case 0:
            manualCount = 1;
            Log_info0("Device MANUAL RUN L");
            mannualOperationStop = 0;
            //WS2812_show();
            if(Current_Status !=9){
            runheatr_test[0] = 01;
            runheatr_test[1] = 35;
            runheatr_test[2] = 4;
            runheatr_test[3] = 176;
            runheatr_test[4] = 01;
            runheatr_test[5] = 03;
            GPIO_write(irOutput, 1);
            runheater(runheatr_test);
            Current_Status = 9;
            setPointTemp = 35;
            }
            else{
                setPointTemp = 35;
                PWMratio1 = 35;
            }
            break;
        case 1:
            manualCount = 2;
            Log_info0("Device MANUAL RUN M");
            //WS2812_show();
            GPIO_write(irOutput, 1);
            setPointTemp = 40;
            PWMratio1 = 40;
            break;
        case 2:
            manualCount = 0;
            Log_info0("Device MANUAL RUN H");
            //WS2812_show();
            GPIO_write(irOutput, 1);
            setPointTemp = 42;
            PWMratio1 = 42;
            break;
        }


    }
    if(Current_Status ==9 && pixelNitify !=1){
    switch(manualCount){
    //manual control heat level display color set
            case 0:
                WS2812_setPixelColor(2, 0xFF, 0, 0);
                WS2812_setPixelColor(1, 0xFF, 0, 0);
                WS2812_setPixelColor(0, 0xFF, 0, 0);
                WS2812_show();
                break;
            case 1:
                WS2812_setPixelColor(2, 0xFF, 0, 0);
                WS2812_setPixelColor(1, 0, 0, 0);
                WS2812_setPixelColor(0, 0, 0, 0);
                WS2812_show();
                break;
            case 2:
                WS2812_setPixelColor(2, 0xFF, 0, 0);
                WS2812_setPixelColor(1, 0xFF, 0, 0);
                WS2812_setPixelColor(0, 0, 0, 0);
                WS2812_show();
                break;

            }
    }
}

static void
delayFun (int delayTimeInMs){

        int delaycount = 0;

          while (delaycount <= delayTimeInMs)
        {
              Task_sleep (1 * (1000 / Clock_tickPeriod));
          delaycount = delaycount + 1;

        }

}
/*********************************************************************
* @fn      setThermalReg
*
* @brief   Activate BMS Thrmal Regulation
*
* @param   I2C_Handle, i2c, I2C_Transaction, txBuffer
*
* @return  none
*/
void setThermalReg(I2C_Handle i2c, I2C_Transaction i2cTransaction,uint8_t *txBuffer, uint8_t *rxBuffer){
    uint8_t CHRG_CTRL2 = 0x06;
    uint8_t slaveAddress = 0x6a;
    i2cTransaction.slaveAddress = slaveAddress;
    i2cTransaction.writeCount = 2;
    i2cTransaction.readCount = 1;
    txBuffer[0] = CHRG_CTRL2;
    txBuffer[1] = 0x4E;    //value to change thermal regulation threshold to 60c

        I2C_init();
    if (I2C_transfer(i2c, &i2cTransaction))
    {

       // Display_printf(display, 0, 0, "CHRG_CTRL2 data = %0x",rxBuffer[0]);
        Log_info1("CHRG_CTRL2 data = %0x",rxBuffer[0]);
    }
    else
    {
        //i2cErrorHandler(&i2cTransaction, display);
        Error_Code = 39;
    }
    I2C_close(i2c);

}

/*
* @fn      batteryIndication
*
* @brief   Battery level LED Indication
*
* @param   type(Indication type)
*
* @return  none
*/
static void batteryIndication(uint8_t type){
    batteryLevelApp = BQ25887();
        if(batteryLevelApp < 30){
                batteryLevel = 1;
               }
            else if(batteryLevelApp < 60 && batteryLevelApp > 30 ){
                batteryLevel = 2;
            }

            else if(batteryLevelApp < 90 && batteryLevelApp > 60 ){
                batteryLevel = 3;
            }

            else if(batteryLevelApp > 90){
                batteryLevel = 4;
            }
        switch(batteryLevel){

            case 1:
                WS2812_setPixelColor(0, 0, 0, 0);
                WS2812_setPixelColor(1, 0, 0, 0);
                WS2812_setPixelColor(2,  0xFF, 0xC0, 0);
                WS2812_show();
                break;
            case 2:
                WS2812_setPixelColor(0, 0, 0, 0);
                WS2812_setPixelColor(1, 0xFF, 0xC0, 0);
                WS2812_setPixelColor(2, 0xFF, 0xC0, 0);


                WS2812_show();
                break;
            case 3:
                WS2812_setPixelColor(0,  0xFF, 0xC0, 0);
                WS2812_setPixelColor(1,  0xFF, 0xC0, 0);
                WS2812_setPixelColor(2,  0xFF, 0xC0, 0);
                WS2812_show();
                break;
            case 4:
                WS2812_setPixelColor(0, 0xFF, 0xC0, 0);
                WS2812_setPixelColor(1, 0xFF, 0xC0, 0);
                WS2812_setPixelColor(2, 0xFF, 0xC0, 0);
                WS2812_show();
                break;
            }

}

/*********************************************************************
* @fn      achilliesInit
*
* @brief   Achilles turn on / Off sequence
*
* @param   type(on or off)
*
* @return  none
*/

static void achilliesInit(uint8_t type){
    switch (type){
    /*
     * 1 : Turn ON
     * 2 : Turn OFF
     *
     */
    case 1:
        onLEDs(2,0x3B,0xFF, 0x09 );
        GPIO_write(CONFIG_LED_1_GPIO, 0);
        GPIO_write(CONFIG_LED_2_GPIO, 0);
        Task_sleep (250 * (1000 / Clock_tickPeriod));
        GPIO_write(CONFIG_LED_1_GPIO, 1);
        GPIO_write(CONFIG_LED_2_GPIO, 1);
        onLEDs(1,0,0, 0 );
        break;
    case 2:
        onLEDs(1,0x3B,0xFF, 0x09 );
        Task_sleep (500 * (1000 / Clock_tickPeriod));
        Current_Status = 1;
        onLEDs(3,0,0, 0 );
        break;

    }


}


/*********************************************************************
* @fn      onLEDs
*
* @brief   LED Pattern common functions
*
* @param   pattern, Color pattern in Hex
*
* @return  none
*/
static void onLEDs( uint8_t pattern,uint8_t arg_u8_red, uint8_t arg_u8_green, uint8_t arg_u8_blue ){
if(pattern == 1){
       WS2812_setPixelColor(0, arg_u8_red, arg_u8_green, arg_u8_blue);
       WS2812_setPixelColor(1, arg_u8_red, arg_u8_green, arg_u8_blue);
       WS2812_setPixelColor(2, arg_u8_red, arg_u8_green, arg_u8_blue);
       WS2812_show();
    }
//power on seq
else if(pattern ==2){
       Task_sleep (500 * (1000 / Clock_tickPeriod));
       WS2812_setPixelColor(2, arg_u8_red, arg_u8_green, arg_u8_blue);
       WS2812_setPixelColor(1, 0, 0, 0);
       WS2812_setPixelColor(0, 0, 0, 0);
       WS2812_show();
       Task_sleep (500 * (1000 / Clock_tickPeriod));
       WS2812_setPixelColor(2, arg_u8_red, arg_u8_green, arg_u8_blue);
       WS2812_setPixelColor(1, arg_u8_red, arg_u8_green, arg_u8_blue);
       WS2812_setPixelColor(0, 0, 0, 0);
       WS2812_show();
       Task_sleep (500 * (1000 / Clock_tickPeriod));
       WS2812_setPixelColor(2, arg_u8_red, arg_u8_green, arg_u8_blue);
       WS2812_setPixelColor(1, arg_u8_red, arg_u8_green, arg_u8_blue);
       WS2812_setPixelColor(0, arg_u8_red, arg_u8_green, arg_u8_blue);
       WS2812_show();

        }
//power off seq
else if(pattern ==3){

       Task_sleep (250 * (1000 / Clock_tickPeriod));
       WS2812_setPixelColor(0, arg_u8_red, arg_u8_green, arg_u8_blue);
       WS2812_show();
       Task_sleep (250 * (1000 / Clock_tickPeriod));
       WS2812_setPixelColor(1, arg_u8_red, arg_u8_green, arg_u8_blue);
       WS2812_show();
       Task_sleep (250 * (1000 / Clock_tickPeriod));
       WS2812_setPixelColor(2, arg_u8_red, arg_u8_green, arg_u8_blue);
       WS2812_show();

        }

}
/*********************************************************************
* @fn      stopClockSwiFxn
*
* @brief   Stop cmd software interrupt, reset all requred flags
*
* @param   Stop type
*
* @return  none
*/
static void
stopClockSwiFxn(UArg p){
   // stopFlag=1;
    Log_info0("stop Clock timed out");
    clock_cunt2  = clock_cunt2 + 1;
    if (clock_cunt2 == 2){
        clock_cunt2=0;
        stopFlag=1;
        GPIO_write(irOutput, 0);
        Log_info0("IR off clock");
        Log_info1("Clock count2 number %d",clock_cunt2);
        Log_info0("Heater output stop due to timeout");
        Current_Status = 2;
        countTick = 0;
        Remaining_Time_01=0;
        Remaining_Time_02=0;
        manualCount = 1;
    }
}

/*********************************************************************
* @fn      setRemainingTime
*
* @brief   Calculate remaining session time
*
* @param   none
*
* @return  none
*/
void
setRemainingTime ()
{
   timecount = 0;
   if(Current_Status == 3){
        timecount = 1;
   }
   else{

       timecount = 0;
   }
  if(stopFlag ==0 && timecount ==1 ){

       int remTimeSec = ((stopClockTime/1000) - countTick);
       countTick = countTick + 1;
           int rem1 = remTimeSec / 255;
           int rem2 = remTimeSec % 255;
           Remaining_Time_01 = rem1;
           Remaining_Time_02 = rem2;
           timecount = 0;
   }
}

/*********************************************************************
* @fn      I2C_read
*
* @brief   to be removed
*
* @param   none
*
* @return  none
*/

static void
I2C_read(uint8_t reg, uint8_t regValue){


}

/*********************************************************************
* @fn      I2CTMT117
*
* @brief   Read TMP116 sensors and calculate Temp
*
* @param   Address
*
* @return  temperature
*/
static float
I2CTMT117(uint8_t slave){

    I2C_init();
    /* Create I2C for usage */
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;
    i2c               = I2C_open(CONFIG_I2C_0, &i2cParams);
    /* Common I2C transaction setup */
    i2cTransaction.writeBuf   = txBufferTMT117;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf    = rxBufferTMT117;
    i2cTransaction.readCount  = 0;

    i2cTransaction.slaveAddress = slave;
    txBufferTMT117[0] =00;
    txBufferTMT117[1] =00;

    i2cTransaction.readCount  = 2;

    if (I2C_transfer(i2c, &i2cTransaction)) {

             temperature = (rxBufferTMT117[0] << 8) | (rxBufferTMT117[1]);
             temperature *= 0.78125;
            // Log_info0("I2C transaction Succeed");
             if (slave == 0x48){
                 //Log_info1("Temperature of temp1 is : %d (C)", temperature);
                 I2C_close(i2c);
                 I2CStatusTMP116 = 4;
                 tmp116errorcount = 0;
                // return temperature;
             }
             if (slave == 0x49){
                // Log_info1("Temperature of temp2 is : %d (C)", temperature);
                 I2CStatusTMP116 = 4;
                 tmp116errorcount = 0;
                 I2C_close(i2c);
                // return temperature;
             }

             return temperature;
    }
    else {
       // Log_info0("No I2C transaction happen");

        I2C_close(i2c);
        tmp116errorcount +=1;
        if(tmp116errorcount > 5 && slave == 0x48 && (Current_Status ==3 || Current_Status == 9)){
            I2CStatusTMP116 = 2;
           // Log_error0("I2c communication failed  - Skin Temp sensor");
            Error_Code = 35;
            Error_level = 3;
            errorFLag = 0;
            tmp116errorcount = 0;
        }
        else if(tmp116errorcount > 5 && slave == 0x49 && (Current_Status ==3 || Current_Status == 9)){
            I2CStatusTMP116 = 3;
            Log_error0("I2c communication failed  - Wire Temp sensor");
            Error_Code = 36;
            Error_level = 3;
            errorFLag = 0;
            tmp116errorcount = 0;
        }
      //  temperature = Pretemperature;
        return temperature;
            }

    if(temperature == 0 ){

           //  temperature = Pretemperature;
             tempzerocount +=1;
         }
         else{
             tempzerocount = 0;
         }

         if(tempzerocount == 5 && slave == 0x48){

             Error_Code = 27;
             tempzerocount = 0;

         }

         else if(tempzerocount == 5 && slave == 0x49){

             Error_Code = 28;
             tempzerocount = 0;

         }
   // Pretemperature = temperature;
    I2C_close(i2c);
    return temperature;

}


/*********************************************************************
* @fn      TMP117init
*
* @brief   Initialize Temp sensors and update relevent registers
*
* @param   State : Shutdown, Continuos convertion etc.... Slave : Address
*
* @return  None
*/

static void
TMP117init(uint8_t state, uint8_t slave){
    //State 1 = shut down
    //State 2 = CC mode
    uint8_t txBuffer2[4] ;
    uint8_t rxBuffer2[4] ;
    I2C_Handle i2c;
    I2C_Params i2cParams;
    I2C_Transaction i2cTransaction;
    I2C_init();
    I2C_Params_init(&i2cParams);
        i2cParams.bitRate = I2C_400kHz;
        i2c               = I2C_open(CONFIG_I2C_0, &i2cParams);
        i2cTransaction.slaveAddress = slave;
        i2cTransaction.writeBuf   = txBuffer2;
        i2cTransaction.readBuf    = rxBuffer2;
        i2cTransaction.readCount  = 2;
        i2cTransaction.writeCount = 3;
        txBuffer2[0] = UNLOCK_REG;
        txBuffer2[1] = 0x80;
        txBuffer2[2] = 0x00;
           if (I2C_transfer(i2c, &i2cTransaction))
           {

               //isplay_printf(display, 0, 0, "unlock reg %0x %0x", rxBuffer2[0],rxBuffer2[1]);
           }
           else
           {
               //i2cErrorHandler(&i2cTransaction, display);
           }


           txBuffer2[0] = CONFIG_REG;
           if (state == 1) {
               txBuffer2[1] = 0x06;  //setting value to shutdown mode
               txBuffer2[2] = 0x20;
               i2cTransaction.writeCount = 3;
               if (I2C_transfer(i2c, &i2cTransaction))
               {
                   Log_info0("TMP116 is shutting down.....");
                   //Display_printf(display, 0, 0, "config reg %0x %0x", rxBuffer2[0],rxBuffer2[1]);
               }
               else
               {
                   Error_Code = 42;
                   //i2cErrorHandler(&i2cTransaction, display);
               }

           }

           else if (state == 2){
               txBuffer2[1] = 0x01;  //setting value to continous conversion mode
               txBuffer2[2] = 0x20;
               i2cTransaction.writeCount = 3;
               if (I2C_transfer(i2c, &i2cTransaction))
               {
                   I2CStatusTMP116 = 4;
                   Log_info0("TMP116 is continuous conversion mode");
                   //Display_printf(display, 0, 0, "config reg %0x %0x", rxBuffer2[0],rxBuffer2[1]);
               }
               else
               {
                   I2CStatusTMP116 = 1;
                   //i2cErrorHandler(&i2cTransaction, display);
               }

           }
           I2C_close(i2c);
}

/*********************************************************************
* @fn      BQ25887
*
* @brief   Communicate with BMS and calculate battery percentage
*
* @param   None
*
* @return  None
*/
static int
BQ25887(){
    batCount += 1;
    uint16_t sample_t = 2000;
    uint8_t txBuffer[2];
    uint8_t rxBuffer[2];
    uint16_t sample;
    uint8_t data;
    int8_t i;
    size_t txsize;
    uint8_t readData;
    uint16_t bat_capacity = 2400;
    uint16_t batPercentage=0;
    uint16_t lastbatPercentage=0;
    I2C_Handle i2c;
    I2C_Params i2cParams;
    I2C_Transaction i2cTransaction;
    I2C_init();

    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;
    i2c               = I2C_open(CONFIG_I2C_0, &i2cParams);

    /* Common I2C transaction setup */
        i2cTransaction.writeBuf   = txBuffer;
        i2cTransaction.writeCount = 1;
        i2cTransaction.readBuf    = rxBuffer;
        i2cTransaction.readCount  = 0;

        i2cTransaction.slaveAddress = slaveAddress;
        txBuffer[0]                 = CHARGER_STATUS_REG1;

        /* Take 20 samples and print them out onto the console */
            i2cTransaction.readCount = 1;
            if (I2C_transfer(i2c, &i2cTransaction))
            {
                /*
                 * Extract degrees C from the received data;
                 * see TMP sensor datasheet
                 */

                data = rxBuffer[0];
                //Display_printf(display, 0, 0, "data in CHARGER_STATUS_REG1 %d",data);
                uint8_t shifted = data << 5;
                if( 0b00000000 == ( shifted ^ 0b00000000)){
                    Log_info0("Not Charging");
                    chargingCompletionFlag = 2;
                }
                else if ( 0b00000000 == (shifted ^ 0b00100000)){
                    Log_info0("Trickle Charge");
                }
                else if ( 0b00000000 == (shifted ^ 0b01000000)){
                    Log_info0("Pre-Charge");
                }
                else if ( 0b00000000 == (shifted ^ 0b01100000)){
                    Log_info0("Fast_Charge(CC mode)");
                }
                else if ( 0b00000000 == (shifted ^ 0b10000000)){
                    Log_info0("Fast_Charge(CV mode)");
                }
                else if ( 0b00000000 == (shifted ^ 0b10100000)){
                    Log_info0("Taper Charge(CV mode)");
                }
                else if ( 0b00000000 == (shifted^ 0b11000000)){
                    Log_info0("Charge termination done");
                    chargingCompletionFlag = 1;
                }
                else if ( 0b00000000 == (shifted ^ 0b11100000)){
                    Log_info0("Reserved");
                }
                else{
                    Log_info0("status unidentified");
                }
            }
            else
            {
                Error_Code = 34;
             //   Log_info0("status reading error#######################3");
            }

    /*Enabling ADC*/
        i2cTransaction.slaveAddress = slaveAddress;
        i2cTransaction.writeCount = 2;
        i2cTransaction.readCount = 1;
        txBuffer[0] = ADC_CON;
        txBuffer[1] = 0x80;    //value to enable ADC

        i2cTransaction.readCount = 1;
        if (I2C_transfer(i2c, &i2cTransaction))
        {

            data = rxBuffer[0];
            I2CStatusBQ25887 = 4;
            //Display_printf(display, 0, 0, "ADC_CON data %d",data);
        }
        else
        {
            I2C_close(i2c);
            I2CStatusBQ25887 = 1;
            return 0;
          //  i2cErrorHandler(&i2cTransaction, display);
        }

        //Calculating battery percentage
        if(batCount ==1){
            soc0 = SOC_OCV(i2c, i2cTransaction, txBuffer, rxBuffer); //calculate SOC using SOC-OCV profile(while battery is at idle)
            Log_info1("soc0 __________________________________________-: %d (%)", soc0);
        }

       //Display_printf(display, 0, 0, "soc0 =  %f",soc0);
       //  Log_info1("soc0 : %d ", soc0);

       SOC(sample_t, i2c, i2cTransaction, txBuffer, rxBuffer);  //calculate the running value of charge due to charge current
      // Display_printf(display, 0, 0, "charge gained =  %f",charge);
      // Log_info1("charge gained : %d ", charge);

       DOD(sample_t, i2c, i2cTransaction, txBuffer, rxBuffer );
       //Display_printf(display, 0, 0, "charge drained =  %f",discharge);  //calculate running value of charge drained due to heater current
       // Log_info1("charge drained : %d ", discharge);
       //float battery_level = soc0 + (charge/bat_capacity)*100 - (discharge/bat_capacity)*100; //charge and discharge should set to zero after this.
       // Display_printf(display, 0, 0, "battery level =  %f",battery_level);
       lastbatPercentage = batPercentage;
       batPercentage = soc0 + (charge/bat_capacity)*100 - (discharge/bat_capacity)*100; //charge and discharge should set to zero after this.
       batPercentage = ((batPercentage + 5) /10)*10;  //Calculate real bat percentage.
       Log_info1("bat Percentage : %d (%)", batPercentage);
       I2C_close(i2c);
       charge=0;
       discharge=0;
       if(batPercentage ==0){
           batPercentage = lastbatPercentage;
       }
       return batPercentage;
}

/*********************************************************************
* @fn      SOC_OCV
*
* @brief   Function to calculate SOC using SOC-OCV profile(while battery is at idle)
*
* @param   I2C Parameters
*
* @return  soc0
*/
float SOC_OCV(I2C_Handle i2c, I2C_Transaction i2cTransaction,uint8_t *txBuffer, uint8_t *rxBuffer)
{
    vBat = 0;
    uint8_t vBatValue1;
    uint8_t vBatValue2;
    i2cTransaction.slaveAddress = slaveAddress;
    txBuffer[0] = VBAT_REG1;

    i2cTransaction.readCount = 1;
    if (I2C_transfer(i2c, &i2cTransaction))
    {

        vBatValue1 = rxBuffer[0];
       // Display_printf(display, 0, 0, "VBAT_REG1 data %d",vBatValue1);
       // Log_info1("VBAT_REG1 data : %d (V)", vBatValue1);
    }
    else
    {
        Error_Code = 46;
        I2C_close(i2c);
        return 0;
    }

    //usleep(10000);


    i2cTransaction.slaveAddress = slaveAddress;
    txBuffer[0] = VBAT_REG2;

    i2cTransaction.readCount = 1;
    if (I2C_transfer(i2c, &i2cTransaction))
    {

        vBatValue2 = rxBuffer[0];
       // Log_info1("VBAT_REG2 data : %d (V)", vBatValue2);
    }
    else
    {
        I2C_close(i2c);
        return 0;
       // i2cErrorHandler(&i2cTransaction, display);
    }

    //calculate battery voltage
    //consider voltage reading in VBAT_REG1
    for (int i = 6; i >= 0; i--) {
        if (vBatValue1 & (1 << i)) {
            int n=1;
            for (int j=0; j<(8+i); j++){
                n *= 2;
            }
            vBat += n;
        }
    }
    //consider voltage reading in VBAT_REG2
    for (int i = 7; i >= 0; i--) {
        if (vBatValue2 & (1 << i)) {
            int n=1;
            for (int j=0; j<i; j++){
                n *= 2;
            }
            vBat += n;
        }
    }


    if (chargingStatus == 0){
    vBat = (vBat - 230)/1000;
    }
    else{
    vBat = vBat/1000;
    }
    //Display_printf(display, 0, 0, "battery voltage =  %f",vBat);

    //calculate SOC using SOC-OCV profile
    vBatRead = 1.007*vBat + 0.02840;
    Log_info1("battery voltag __________________________________________________: %d (V)", vBatRead*1000);
    float soc0;
    if (vBatRead<6.5){
        soc0=0;
        //Display_printf(display, 0, 0, "Battery drained to zero");
        // Error code 45
    }
    else if (vBatRead<6.9){
        soc0=13.28*vBatRead - 88.6;
    }
    else if (vBatRead<7.012){
        soc0=62.5*vBatRead - 431.1;
    }
    else if (vBatRead<7.19){
        soc0=74.5*vBatRead - 516.1;
    }
    else if (vBatRead<7.352){
        soc0=172*vBatRead - 1225;
    }
    else if (vBatRead<7.478){
        soc0=114.75*vBatRead - 800.9;
    }
    else if (vBatRead<7.934){
        soc0=55.95*vBatRead - 359.9;
    }
    else if (vBatRead<8.078){
        soc0=52.4*vBatRead - 332;
    }
    else if (vBatRead<8.264){
        soc0=(45.305*vBatRead) - 274.7;
    }
    else{
        soc0=100;
    }


    //Display_printf(display, 0, 0, "battery level =  %f",soc0);
    return soc0;
}


/*********************************************************************
* @fn      SOC
*
* @brief   Function calculates the running value of charge due to charge current
*
* @param   I2C Parameters
*
* @return  charge
*/
float SOC(uint16_t sample_t, I2C_Handle i2c, I2C_Transaction i2cTransaction,uint8_t *txBuffer, uint8_t *rxBuffer){
    float chrgCurr=0;
    uint8_t chargeCurr1;
    uint8_t chargeCurr2;
    i2cTransaction.slaveAddress = slaveAddress;
      txBuffer[0] = CHRG_CURR_REG1;

      i2cTransaction.readCount = 1;
      if (I2C_transfer(i2c, &i2cTransaction))
      {

          chargeCurr1 = rxBuffer[0];
          //Display_printf(display, 0, 0, "charge current reg1 data %d",chargeCurr1);
       //   Log_info1("charge current reg1 data : %d ", chargeCurr1);
      }
      else
      {
          I2C_close(i2c);
          return 0;
        //  i2cErrorHandler(&i2cTransaction, display);
      }


      //usleep(10000);


      i2cTransaction.slaveAddress = slaveAddress;
      txBuffer[0] = CHRG_CURR_REG2;

      i2cTransaction.readCount = 1;
      if (I2C_transfer(i2c, &i2cTransaction))
      {

          chargeCurr2 = rxBuffer[0];
        //  Log_info1("charge current reg2 data : %d ", chargeCurr2);
      }
      else
      {
          I2C_close(i2c);
          return 0;
         // i2cErrorHandler(&i2cTransaction, display);
      }

      //usleep(10000);
      //consider charge current reading in CHRG_CURR_REG1
      for (int i = 6; i >= 0; i--) {
          if (chargeCurr1 & (1 << i)) {
              int n=1;
              for (int j=0; j<(8+i); j++){
                  n *= 2;
              }
              chrgCurr += n;
          }
      }
      //consider charge current reading in CHRG_CURR_REG2
      for (int i = 7; i >= 0; i--) {
          if (chargeCurr2 & (1 << i)) {
              int n=1;
              for (int j=0; j<i; j++){
                  n *= 2;
              }
              chrgCurr += n;
          }
      }


      // Thermal Regulation
         uint8_t CHRG_CTRL2 = 0x06;
         uint8_t slaveAddress = 0x6a;
         i2cTransaction.slaveAddress = slaveAddress;
         i2cTransaction.writeCount = 2;
         i2cTransaction.readCount = 1;
         txBuffer[0] = CHRG_CTRL2;
         txBuffer[1] = 0x4E;    //value to change thermal regulation threshold to 60c

         if (I2C_transfer(i2c, &i2cTransaction))
         {

            // Display_printf(display, 0, 0, "CHRG_CTRL2 data = %0x",rxBuffer[0]);
            // Log_info1("CHRG_CTRL2 data = %0x",rxBuffer[0]);
         }
         else
         {
             //i2cErrorHandler(&i2cTransaction, display);
             Error_Code = 39;
             I2C_close(i2c);
         }


     // Display_printf(display, 0, 0, "charge current %f", chrgCurr);
    Log_info1("charge current : %d (A)", chrgCurr);
    float charge_gained = chrgCurr * sample_t /3600000;  //charge gained in mAh
    charge += charge_gained;
    return charge;
}

/*********************************************************************
* @fn      SOC
*
* @brief   Function calculates running value of charge drained due to heater current
*
* @param   I2C Parameters
*
* @return discharge
*/

float DOD(uint16_t sample_t, I2C_Handle i2c, I2C_Transaction i2cTransaction,uint8_t *txBuffer, uint8_t *rxBuffer)
{
    float busCurr = 0;
    uint16_t currentADC = 0;
    if(Current_Status == 3 || Current_Status == 9 ){
        currentADC = scifTaskData.achButton.output.pAdcValue[1];
       // Log_info1("currentRead_______________________________________________________ : %d ", currentADC);

    }
    else{
        currentADC = 0;
    }
    float heaterCurrentReading = currentADC * 4.949;
    Log_info1("heaterCurrentReading_______________________________________________________ : %d ", heaterCurrentReading);
    busCurr += heaterCurrentReading;
    uint8_t IBUS1;
    uint8_t IBUS2;
    i2cTransaction.slaveAddress = slaveAddress;
          txBuffer[0] = IBUS_REG1;

          i2cTransaction.readCount = 1;
          if (I2C_transfer(i2c, &i2cTransaction))
          {

              IBUS1 = rxBuffer[0];
              //Display_printf(display, 0, 0, "IBUS1 data %d",IBUS1);
             // Log_info1("IBUS1 data : %d ", IBUS1);

          }
          else
          {
              I2C_close(i2c);
              return 0;
          }
         // usleep(10000);


          i2cTransaction.slaveAddress = slaveAddress;
          txBuffer[0] = IBUS_REG2;

          i2cTransaction.readCount = 1;
          if (I2C_transfer(i2c, &i2cTransaction))
          {

              IBUS2 = rxBuffer[0];
            //  Log_info1("IBUS2 data : %d ", IBUS2);
          }
          else
          {
              I2C_close(i2c);
              return 0;
          }
         // usleep(10000);

          for (int i = 6; i >= 0; i--) {
              if (IBUS1 & (1 << i)) {
                  int n=1;
                  for (int j=0; j<(8+i); j++){
                      n *= 2;
                  }
                  busCurr += n;
              }
          }

          for (int i = 7; i >= 0; i--) {
              if (IBUS2 & (1 << i)) {
                  int n=1;
                  for (int j=0; j<i; j++){
                      n *= 2;
                  }
                  busCurr += n;
              }
          }
    //Display_printf(display, 0, 0, "bus current %f", busCurr);
    Log_info1("bus current : %d (A)", busCurr);
    discharge +=  busCurr*sample_t/3600000;
    return discharge;
}

/*********************************************************************
* @fn      OPENpwm
*
* @brief   Open PWM module
*
* @param   Duty ratio
*
* @return  none
*/
static void
OPENpwm(float PWMratio1, float PWMratio2){
     // Initialize the PWM driver.
     PWM_Params pwmParams,pwmParams1;
     PWM_init();

     // Initialize the PWM parameters
     PWM_Params_init(&pwmParams);
     pwmParams.idleLevel = PWM_IDLE_LOW;      // Output low when PWM is not running
     pwmParams.periodUnits = PWM_PERIOD_HZ;     // The duty is in microseconds.
     pwmParams.periodValue = 20000;             //Time period amount
     pwmParams.dutyUnits = PWM_DUTY_FRACTION; // Duty is in fractional percentage
     pwmParams.dutyValue = 0;                 // 0% initial duty cycle

     // Open the PWM instance
     PWM1 = PWM_open(CONFIG_PWM_0, &pwmParams);

    // PWM3 = PWM_open(Board_PWM2, &pwmParams1);

     if (PWM1 == NULL) {
         // PWM_open() failed
         while (1);
     }

     PWM_start(PWM1);
     dutyValue1 = (uint32_t) (((uint64_t) PWM_DUTY_FRACTION_MAX * 0) / 100);   // make here PWM value equal to 0
     PWM_setDuty(PWM1, dutyValue1);  // set duty cycle
}

/*********************************************************************
* @fn      initialCheck
*
* @brief   Achilles Initial Power on sequence
*
* @param   None
*
* @return  none
*/
static void
initialCheck(){
    //TMP116 com check
    initialBodyTemp = I2CTMT117(0x48); //Skin sensor
    initialWiretemp = I2CTMT117(0x49); //Wire sensor

    switch(I2CStatusTMP116){
        case 1:
            Error_Code = 12;
            errorFLag = 0;
            Current_Status = 5;
            break;
        case 2:
            Error_Code = 35;
            errorFLag = 0;
            Current_Status = 5;
            break;
        case 3:
            Error_Code = 36;
            errorFLag = 0;
            Current_Status = 5;
            break;
        case 4:
            Error_Code = 37;
            errorFLag = 0;
            Current_Status = 5;
            break;
        case 5:
            Error_Code = 38;
            errorFLag = 0;
            Current_Status = 5;
            break;
    }


    if(initialBodyTemp >= 37){
        Error_Code = 17;
        errorFLag = 0;
        Current_Status = 5;
    }

    if(initialWiretemp >= 40){
            Error_Code = 18;
            errorFLag = 0;
            Current_Status = 5;
        }

    if(initialBodyTemp <= 10){
           Error_Code = 19;
           errorFLag = 0;
           Current_Status = 5;
       }

    if(initialWiretemp <= 10){
           Error_Code = 20;
           errorFLag = 0;
           Current_Status = 5;
       }


    //BQ BMS Com check
    BQ25887();
    if(I2CStatusBQ25887 !=4){
            Error_Code = 13;
            errorFLag = 0;
            Current_Status = 5;
        }

/*
 * Heater initial testing / open circuit check
 * Turn on 5V buck
 * Heate check On GPIO Set to High
 * Read Heater ADC ---> Heater ADC should not be Zero and It should withing the range
 * Start Heart Beat
 * Digital Read failsafeOff GPIO --> if its High, Failsafe uC is started its operation
 * If Failsafe uC is started operation  ---> Heater ADC Should goes to Zero
 */
    Log_info0("############# heater check on ##################");
    GPIO_write(HeatercheckOn,1);
    Task_sleep(100 * (1000/Clock_tickPeriod));
    heaterOkAdc = scifTaskData.achButton.output.pAdcValue[2];
    Log_info1("################################ Heater ok ADC Check:%d###########################",heaterOkAdc);
//    //if(heaterAdc <  )
   // int state=GPIO_read(HeatercheckOn);
   // Log_info1("%%%% heater checkOn state:%d ##################",state);
    // Error_Code = 22;

    //Fail safe microcontroller status check
   // Error_Code = 21;
    heaterAdc = scifTaskData.achButton.output.pAdcValue[1];
    if(heaterAdc<10){
        Error_Code = 22;
        errorFLag = 0;
        Current_Status = 5;//should be change
    }else{
        fail_safe_fet_on=true;
         Log_info0("################fail safe fet set to on###############################");
    }
    dutyValue1 = (uint32_t) (((uint64_t) PWM_DUTY_FRACTION_MAX * 0) / 100);   // make here PWM value equal to 0
    PWM_setDuty(PWM1, dutyValue1);  // set duty cycle
    //Button pannel ADC check
    initialBP_ADC = scifTaskData.achButton.output.pAdcValue[0];
    if(initialBP_ADC > 200){
        Error_Code = 14;
        errorFLag = 0;
        Current_Status = 5;
    }
    //Watchdog initiation
    if(initialBP_ADC > 200){
            Error_Code = 15;
            errorFLag = 0;
            Current_Status = 5;
        }
}

/*********************************************************************
* @fn      user_stopHeater_ValueChangeHandler
*
* @brief   Stop command handler
*
* @param   pData
*
* @return  none
*/
void
user_stopHeater_ValueChangeHandler(pzCharacteristicData_t *pData)
{
  switch (pData->paramID)
  {
        case STOPHEATER_STOPHEATER_ID:
        Log_info0("Value Change msg for stopHeater :: stopHeater received");

        int StopVal = pData->data[1];

                if (StopVal==1){
                    //Ack_reply_SetData(0,9,1,0);

                    Log_info0("Heater Protocol Stop ");
                    stopFlag =1;

                 //   I2CWriteMSP430(12);
                    Clock_stop(Clock_handle(&stopClock));
                    Current_Status = 7 ;
                    countTick = 0;

                }
                if (StopVal==2){
                    PWM_stop(PWM1);
                    GPIO_write(irOutput, 0);
                    stopFlag = 1;
                    Log_info0("Heater Emergency Stop ");
                    Clock_stop(Clock_handle(&stopClock));
                    Current_Status = 7 ;
                    countTick = 0;
                }

        break;
      }

}

/*********************************************************************
* @fn      user_runHeater_ValueChangeCB
*
* @brief   Run command Service callback function
*
* @param   pData
*
* @return  none
*/

static void user_runHeater_ValueChangeCB(uint16_t connHandle,
                              uint8_t paramID, uint16_t len,
                              uint8_t *pValue)
{
    pzCharacteristicData_t *pValChange =
        ICall_malloc(sizeof(pzCharacteristicData_t) + len);

    if(pValChange != NULL)
    {
        pValChange->svcUUID = RUNHEATER_SERV_UUID;
        pValChange->paramID = paramID;
        memcpy(pValChange->data, pValue, len);
        pValChange->dataLen = len;

        AchillesPB_enqueueMsg(PZ_SERVICE_WRITE_EVT, pValChange);
    }
}


/*********************************************************************
* @fn      user_pauseHeater_ValueChangeCB
*
* @brief   Pause command Service callback function
*
* @param   pData
*
* @return  none
*/
static void user_pauseHeater_ValueChangeCB(uint16_t connHandle,
                              uint8_t paramID, uint16_t len,
                              uint8_t *pValue)
{
    pzCharacteristicData_t *pValChange =
        ICall_malloc(sizeof(pzCharacteristicData_t) + len);

    if(pValChange != NULL)
    {
        pValChange->svcUUID = PAUSEHEATER_SERV_UUID;
        pValChange->paramID = paramID;
        memcpy(pValChange->data, pValue, len);
        pValChange->dataLen = len;

        AchillesPB_enqueueMsg(PZ_SERVICE_WRITE_EVT, pValChange);
    }
}

/*********************************************************************
* @fn      user_stopHeater_ValueChangeCB
*
* @brief   Pause command Service callback function
*
* @param   pData
*
* @return  none
*/

static void user_stopHeater_ValueChangeCB(uint16_t connHandle,
                              uint8_t paramID, uint16_t len,
                              uint8_t *pValue)
{
    pzCharacteristicData_t *pValChange =
        ICall_malloc(sizeof(pzCharacteristicData_t) + len);

    if(pValChange != NULL)
    {
        pValChange->svcUUID = STOPHEATER_SERV_UUID;
        pValChange->paramID = paramID;
        memcpy(pValChange->data, pValue, len);
        pValChange->dataLen = len;

        AchillesPB_enqueueMsg(PZ_SERVICE_WRITE_EVT, pValChange);
    }
}


/******************************************************************************
 *****************************************************************************
 *
 *  Utility functions
 *
 ****************************************************************************
 *****************************************************************************/

/*********************************************************************
 * @fn     AchillesPB_enqueueMsg
 *
 * @brief  Utility function that sends the event and data to the application.
 *         Handled in the task loop.
 *
 * @param  event    Event type
 * @param  pData    Pointer to message data
 */
static status_t AchillesPB_enqueueMsg(uint8_t event, void *pData)
{
    uint8_t success;
    pzMsg_t *pMsg = ICall_malloc(sizeof(pzMsg_t));

    if(pMsg)
    {
        pMsg->event = event;
        pMsg->pData = pData;

        success = Util_enqueueMsg(appMsgQueueHandle, syncEvent, (uint8_t *)pMsg);
        return (success) ? SUCCESS : FAILURE;
    }

    return(bleMemAllocError);
}

/*********************************************************************
 * @fn     util_arrtohex
 *
 * @brief   Convert {0x01, 0x02} to "01:02"
 *
 * @param   src - source byte-array
 * @param   src_len - length of array
 * @param   dst - destination string-array
 * @param   dst_len - length of array
 *
 * @return  array as string
 */
char * util_arrtohex(uint8_t const *src, uint8_t src_len,
                     uint8_t *dst, uint8_t dst_len, uint8_t reverse)
{
    char hex[] = "0123456789ABCDEF";
    uint8_t *pStr = dst;
    uint8_t avail = dst_len - 1;
    int8_t inc = 1;
    if(reverse)
    {
        src = src + src_len - 1;
        inc = -1;
    }

    memset(dst, 0, avail);

    while(src_len && avail > 3)
    {
        if(avail < dst_len - 1)
        {
            *pStr++ = ':';
            avail -= 1;
        }

        *pStr++ = hex[*src >> 4];
        *pStr++ = hex[*src & 0x0F];
        src += inc;
        avail -= 2;
        src_len--;
    }

    if(src_len && avail)
    {
        *pStr++ = ':'; // Indicate not all data fit on line.
    }
    return((char *)dst);
}

/*********************************************************************
 * @fn     util_getLocalNameStr
 *
 * @brief   Extract the LOCALNAME from Scan/AdvData
 *
 * @param   data - Pointer to the advertisement or scan response data
 * @param   len  - Length of advertisment or scan repsonse data
 *
 * @return  Pointer to null-terminated string with the adv local name.
 */
static char * util_getLocalNameStr(const uint8_t *data, uint8_t len)
{
    uint8_t nuggetLen = 0;
    uint8_t nuggetType = 0;
    uint8_t advIdx = 0;

    static char localNameStr[32] = { 0 };
    memset(localNameStr, 0, sizeof(localNameStr));

    for(advIdx = 0; advIdx < len; )
    {
        nuggetLen = data[advIdx++];
        nuggetType = data[advIdx];
        if((nuggetType == GAP_ADTYPE_LOCAL_NAME_COMPLETE ||
            nuggetType == GAP_ADTYPE_LOCAL_NAME_SHORT) )
        {
            uint8_t len_temp = nuggetLen < (sizeof(localNameStr)-1)? (nuggetLen - 1):(sizeof(localNameStr)-2);
            // Only copy the first 31 characters, if name bigger than 31.
            memcpy(localNameStr, &data[advIdx + 1], len_temp);
            break;
        }
        else
        {
            advIdx += nuggetLen;
        }
    }

    return(localNameStr);
}

/*********************************************************************
 * @fn      AchillesPB_eraseExternalFlash
 *
 * @brief   Utility function that erases external flash content
 *
 * @return  void
 */
void AchillesPB_eraseExternalFlash(void)
{
    NVS_Handle nvsHandle;
    NVS_Attrs regionAttrs;
    NVS_Params nvsParams;

    NVS_init();
    NVS_Params_init(&nvsParams);

    Log_info0("Opening external flash for mass erase");
    nvsHandle = NVS_open(CONFIG_NVSEXTERNAL, &nvsParams);
    if (nvsHandle == NULL)
    {
        Log_error0("Failed to open external flash");
        return;
    }
    NVS_getAttrs(nvsHandle, &regionAttrs);
    Log_info3("ExtFlash regionBase: 0x%x, regionSize: 0x%x, sectorSize: 0x%x",
              (uintptr_t)regionAttrs.regionBase,
              (uintptr_t)regionAttrs.regionSize,
              (uintptr_t)regionAttrs.sectorSize);

    // Do sector by sector erase
    for (uint32_t offset = 0; offset < regionAttrs.regionSize; offset += regionAttrs.sectorSize)
    {
        uint16_t status = NVS_erase(nvsHandle, offset, regionAttrs.sectorSize);

        // Sleep every 16th erase to print log and serve other things like network
        uint32_t curSector = (offset / regionAttrs.sectorSize);
        if ((curSector & 0xf) == 0xf)
        {
            Log_info3("Erase sector %d..%d, status: %d", curSector-0xf, curSector, status);
            Task_sleep(20 * (1000/Clock_tickPeriod));
        }
    }

    // Block until ready with meaningless read.
    uint32_t buf;
    NVS_read(nvsHandle, 0, &buf, sizeof(buf));

    Log_info0("External flash erase complete");
    NVS_close(nvsHandle);
}

/*********************************************************************
 * @fn      AchillesPB_revertToFactoryImage
 *
 * @brief   Utility function that invalidates this image and reboots,
 *          in order for BIM to load the image in the factory slot.
 *
 * @return  void
 */
static void AchillesPB_revertToFactoryImage(void)
{
    extern const imgHdr_t _imgHdr;

    uint32_t key = HwiP_disable();

    uint8_t invalidCrc = CRC_INVALID;
    uint32_t retVal = FlashProgram(&invalidCrc,
                                   (uint32_t)&_imgHdr.fixedHdr.crcStat,
                                   sizeof(invalidCrc));
    if (retVal == FAPI_STATUS_SUCCESS)
    {
        Log_info0("CRC Status invalidated. Rebooting into BIM.");
        Task_sleep(50 * (1000 / Clock_tickPeriod));
        SysCtrlSystemReset();
        // We never reach here.
    }
    else
    {
        Log_error1("CRC Invalidate write failed. Status 0x%x", retVal);
        Log_error0("Continuing boot");
    }

    HwiP_restore(key);
}

/*********************************************************************
 * @fn      AchillesPB_bootManagerCheck
 *
 * @brief   Check IOs held during boot to determine whether the
 *          running image should be invalidated and BIM should revert
 *          to the factory image in external flash (Left button), or
 *          whether the external flash content should be erased (Left
 *          and Right button).
 *
 * @param   revertIo - IOID of the gpio that selects revert to factory
 * @param   eraseIo - IOID of the gpio that if held with revert will
 *                    erase the external flash.
 *
 * @return  void
 */
static void AchillesPB_bootManagerCheck(uint_least8_t revertIo,
                                         uint_least8_t eraseIo)
{
    uint32_t sleepDuration = 5000 * (1000/Clock_tickPeriod);
    uint32_t sleepInterval = 50 * (1000/Clock_tickPeriod);

    uint32_t revertIoInit = 1;
    uint32_t eraseIoInit = 1;

    if (revertIoInit)
    {
        Log_info0("Left button not held under boot, "
                "not reverting to factory.");
        Log_info0("Right+Left button not held under boot, "
                "not erasing external flash.");
        return;
    }

    if (revertIoInit == 0 && eraseIoInit == 0)
    {
        Log_warning0("Right+Left button held after reset.");
        Log_warning0("Hold for 5 seconds to erase external flash.");
        Log_warning0("Note that this will also remove the \"factory image\".");

        for (uint32_t i = 0; i < sleepDuration / sleepInterval; ++i)
        {
            Task_sleep(sleepInterval);

            if (GPIO_read(revertIo) || GPIO_read(eraseIo))
            {
                break;
            }
        }

        if (GPIO_read(revertIo) == 0 && GPIO_read(eraseIo) == 0)
        {
            AchillesPB_eraseExternalFlash();
            Log_warning0("There is now no factory image in external flash "
                    "to revert to");
            Log_warning0("Reset the device to make this the factory image");
        }
        else
        {
            Log_info0("Right+Left not held for 5 seconds, continuing boot.");
        }

        return;
    }

    if (revertIoInit == 0)
    {
        Log_warning0("Left button held after reset.");
        Log_warning0("Hold for 5 seconds to invalidate image and reboot");

        for (uint32_t i = 0; i < sleepDuration / sleepInterval; ++i)
        {
            Task_sleep(sleepInterval);

            if (GPIO_read(revertIo))
            {
                break;
            }
        }

        if (GPIO_read(revertIo) == 0)
        {
            AchillesPB_revertToFactoryImage();
        }
        else
        {
            Log_info0("Left not held for 5 seconds, continuing boot.");
        }

        return;
    }
}

/*********************************************************************
*********************************************************************/
//User fucntions

static
void notifyclockSwiFxn(UArg arg){
    AchillesPB_enqueueMsg(AC_NTFY_CLK_EVT, NULL);
    setRemainingTime();
  //  Util_startClock(&Notifyclock);

}
static
void
commondelayclockwiFxn(UArg arg){

}

void user_observerNotify_ValueChangeHandler(pzCharacteristicData_t *pData)
{
  switch (pData->paramID)
  {
        case OBSERVERNOTIFY_NOTIFYCHRAC_ID:
        Log_info0("Value Change msg for observerNotify :: notifyChrac received");
        // Do something useful with pData->data here
        // -------------------------
        break;
      }

}

static void user_observerNotify_ValueChangeCB(uint16_t connHandle,
                              uint8_t paramID, uint16_t len,
                              uint8_t *pValue)
{
    pzCharacteristicData_t *pValChange =
        ICall_malloc(sizeof(pzCharacteristicData_t) + len);

    if(pValChange != NULL)
    {
        pValChange->svcUUID = OBSERVERNOTIFY_SERV_UUID;
        pValChange->paramID = paramID;
        memcpy(pValChange->data, pValue, len);
        pValChange->dataLen = len;

        AchillesPB_enqueueMsg(PZ_SERVICE_WRITE_EVT, pValChange);
    }
}
