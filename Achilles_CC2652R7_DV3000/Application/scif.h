/** \mainpage Driver Overview
  *
  * \section section_drv_info Driver Information
  * This Sensor Controller Interface driver has been generated by the Texas Instruments Sensor Controller
  * Studio tool:
  * - <b>Project name</b>:     AchilliesButton
  * - <b>Project file</b>:     C:/Users/Dushan/Desktop/Sensor controller/Achilles_DV/Achilles_SCP_DV_22_05_2023.scp
  * - <b>Code prefix</b>:      -
  * - <b>Operating system</b>: TI-RTOS
  * - <b>Tool version</b>:     2.9.0.208
  * - <b>Tool patches</b>:     1, 2 and 3
  * - <b>Target chip</b>:      CC2652R7, package QFN48 7x7 RGZ, revision B (1.1)
  * - <b>Created</b>:          2023-06-14 16:42:59.715
  * - <b>Computer</b>:         DESKTOP-QRGPGQ2
  * - <b>User</b>:             Dushan
  *
  * No user-provided resource definitions were used to generate this driver.
  *
  * No user-provided procedure definitions were used to generate this driver.
  *
  * Do not edit the generated source code files other than temporarily for debug purposes. Any
  * modifications will be overwritten by the Sensor Controller Studio when generating new output.
  *
  * \section section_drv_modules Driver Modules
  * The driver is divided into three modules:
  * - \ref module_scif_generic_interface, providing the API for:
  *     - Initializing and uninitializing the driver
  *     - Task control (for starting, stopping and executing Sensor Controller tasks)
  *     - Task data exchange (for producing input data to and consume output data from Sensor Controller
  *       tasks)
  * - \ref module_scif_driver_setup, containing:
  *     - The AUX RAM image (Sensor Controller code and data)
  *     - I/O mapping information
  *     - Task data structure information
  *     - Driver setup data, to be used in the driver initialization
  *     - Project-specific functionality
  * - \ref module_scif_osal, for flexible OS support:
  *     - Interfaces with the selected operating system
  *
  * It is possible to use output from multiple Sensor Controller Studio projects in one application. Only
  * one driver setup may be active at a time, but it is possible to switch between these setups. When
  * using this option, there is one instance of the \ref module_scif_generic_interface and
  * \ref module_scif_osal modules, and multiple instances of the \ref module_scif_driver_setup module.
  * This requires that:
  * - The outputs must be generated using the same version of Sensor Controller Studio
  * - The outputs must use the same operating system
  * - The outputs must use different source code prefixes (inserted into all globals of the
  *   \ref module_scif_driver_setup)
  *
  *
  * \section section_project_info Project Description
  * No description entered
  *
  *
  * \subsection section_io_mapping I/O Mapping
  * Task I/O functions are mapped to the following pins:
  * - achButton:
  *     - <b>A: Analog sensor output 0</b>: DIO24
  *     - <b>A: Analog sensor output 1</b>: DIO25
  *
  *
  * \section section_task_info Task Description(s)
  * This driver supports the following task(s):
  *
  *
  * \subsection section_task_desc_ach_button achButton
  * No description entered
  *
  */




/** \addtogroup module_scif_driver_setup Driver Setup
  *
  * \section section_driver_setup_overview Overview
  *
  * This driver setup instance has been generated for:
  * - <b>Project name</b>:     AchilliesButton
  * - <b>Code prefix</b>:      -
  *
  * The driver setup module contains the generated output from the Sensor Controller Studio project:
  * - Location of task control and scheduling data structures in AUX RAM
  * - The AUX RAM image, and the size the image
  * - Task data structure information (location, size and buffer count)
  * - I/O pin mapping translation table
  * - Task resource initialization and uninitialization functions
  * - Hooks for run-time logging
  *
  * @{
  */
#ifndef SCIF_H
#define SCIF_H

#include <stdint.h>
#include <stdbool.h>
#include "scif_framework.h"
#include "scif_osal_tirtos.h"


/// Target chip name
#define SCIF_TARGET_CHIP_NAME_CC2652R7
/// Target chip package
#define SCIF_TARGET_CHIP_PACKAGE_QFN48_7X7_RGZ

/// Number of tasks implemented by this driver
#define SCIF_TASK_COUNT 1

/// achButton: Task ID
#define SCIF_ACH_BUTTON_TASK_ID 0


/// achButton: 
#define SCIF_ACH_BUTTON_CHANNEL_COUNT 2
/// achButton I/O mapping: Analog sensor output
#define SCIF_ACH_BUTTON_DIO_ASENSOR_OUTPUT { 24, 25 }


// All shared data structures in AUX RAM need to be packed
#pragma pack(push, 2)


/// achButton: Task configuration structure
typedef struct {
    uint16_t pAuxioASensorOutput[2]; ///< I/O mapping: Analog sensor output
    uint16_t thresholdtwo;           ///< 
    uint16_t tresholdone;            ///< 
} SCIF_ACH_BUTTON_CFG_T;


/// achButton: Task output data structure
typedef struct {
    uint16_t adcValue;     ///< 
    uint16_t pAdcValue[2]; ///< 
} SCIF_ACH_BUTTON_OUTPUT_T;


/// achButton: Task state structure
typedef struct {
    uint16_t adcValue;    ///< 
    uint16_t buttonState; ///< 
} SCIF_ACH_BUTTON_STATE_T;


/// Sensor Controller task data (configuration, input buffer(s), output buffer(s) and internal state)
typedef struct {
    struct {
        SCIF_ACH_BUTTON_CFG_T cfg;
        SCIF_ACH_BUTTON_OUTPUT_T output;
        SCIF_ACH_BUTTON_STATE_T state;
    } achButton;
} SCIF_TASK_DATA_T;

/// Sensor Controller task generic control (located in AUX RAM)
#define scifTaskData    (*((volatile SCIF_TASK_DATA_T*) 0x400E015C))


// Initialized internal driver data, to be used in the call to \ref scifInit()
extern const SCIF_DATA_T scifDriverSetup;


// Restore previous struct packing setting
#pragma pack(pop)


// AUX I/O re-initialization functions
void scifReinitTaskIo(uint32_t bvTaskIds);


// RTC-based tick generation control
void scifStartRtcTicks(uint32_t tickStart, uint32_t tickPeriod);
void scifStartRtcTicksNow(uint32_t tickPeriod);
void scifStopRtcTicks(void);


#endif
//@}


// Generated by DESKTOP-QRGPGQ2 at 2023-06-14 16:42:59.715
