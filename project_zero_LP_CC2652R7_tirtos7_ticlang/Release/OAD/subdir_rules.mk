################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
OAD/crc32.o: C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/common/cc26xx/crc/crc32.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"C:/ti/ti_cgt_tiarmclang_1.3.0.LTS/bin/tiarmclang.exe" -c @"C:/Users/Dushan/workspace_v12/Achilles_DV_V3012_23_05_2023.zip_expanded/project_zero_LP_CC2652R7_tirtos7_ticlang/Release/syscfg/ti_ble_app_config.opt" @"C:/Users/Dushan/workspace_v12/Achilles_DV_V3012_23_05_2023.zip_expanded/project_zero_LP_CC2652R7_tirtos7_ticlang/Release/syscfg/ti_build_config.opt" @"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/ble5stack/config/build_components.opt" @"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/ble5stack/config/factory_config.opt"  -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -mlittle-endian -mthumb -Oz -I"C:/Users/Dushan/workspace_v12/Achilles_DV_V3012_23_05_2023.zip_expanded/project_zero_LP_CC2652R7_tirtos7_ticlang" -I"C:/Users/Dushan/workspace_v12/Achilles_DV_V3012_23_05_2023.zip_expanded/project_zero_LP_CC2652R7_tirtos7_ticlang/Release" -I"C:/Users/Dushan/workspace_v12/Achilles_DV_V3012_23_05_2023.zip_expanded/project_zero_LP_CC2652R7_tirtos7_ticlang/Application" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/ble5stack" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/drivers/nvs/" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/common/cc26xx/oad/" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/ble5stack/profiles/oad/cc26xx" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/ble5stack/controller/cc26xx/inc" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/ble5stack/inc" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/ble5stack/rom" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/ble5stack/common/cc26xx" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/ble5stack/icall/inc" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/ble5stack/hal/src/target/_common" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/ble5stack/common/cc26xx/npi/stack" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/ble5stack/hal/src/inc" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/ble5stack/heapmgr" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/ble5stack/profiles/dev_info" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/ble5stack/profiles/simple_profile" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/ble5stack/icall/src/inc" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/ble5stack/osal/src/inc" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/ble5stack/services/src/saddr" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/ble5stack/services/src/sdata" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/common/nv" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/common/cc26xx" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/ble5stack/icall/src" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/ble5stack/common/cc26xx/rcosc" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/devices/cc13x2x7_cc26x2x7" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/kernel/tirtos7/packages" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/posix/ticlang" -Duartlog_FILE="\"crc32.c\"" -DSECURITY -DDeviceFamily_CC26X2X7 -DFLASH_ROM_BUILD -DNVOCMP_NWSAMEITEM=1 -DHEAPMGR_CONFIG=0x80 -DHEAPMGR_SIZE=0x0 -gdwarf-3 -march=armv7e-m -MMD -MP -MF"OAD/crc32.d_raw" -MT"OAD/crc32.o" -I"C:/Users/Dushan/workspace_v12/Achilles_DV_V3012_23_05_2023.zip_expanded/project_zero_LP_CC2652R7_tirtos7_ticlang/Release/syscfg" -std=gnu9x $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

OAD/flash_interface_ext_rtos_NVS.o: C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/common/cc26xx/flash_interface/external/flash_interface_ext_rtos_NVS.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"C:/ti/ti_cgt_tiarmclang_1.3.0.LTS/bin/tiarmclang.exe" -c @"C:/Users/Dushan/workspace_v12/Achilles_DV_V3012_23_05_2023.zip_expanded/project_zero_LP_CC2652R7_tirtos7_ticlang/Release/syscfg/ti_ble_app_config.opt" @"C:/Users/Dushan/workspace_v12/Achilles_DV_V3012_23_05_2023.zip_expanded/project_zero_LP_CC2652R7_tirtos7_ticlang/Release/syscfg/ti_build_config.opt" @"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/ble5stack/config/build_components.opt" @"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/ble5stack/config/factory_config.opt"  -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -mlittle-endian -mthumb -Oz -I"C:/Users/Dushan/workspace_v12/Achilles_DV_V3012_23_05_2023.zip_expanded/project_zero_LP_CC2652R7_tirtos7_ticlang" -I"C:/Users/Dushan/workspace_v12/Achilles_DV_V3012_23_05_2023.zip_expanded/project_zero_LP_CC2652R7_tirtos7_ticlang/Release" -I"C:/Users/Dushan/workspace_v12/Achilles_DV_V3012_23_05_2023.zip_expanded/project_zero_LP_CC2652R7_tirtos7_ticlang/Application" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/ble5stack" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/drivers/nvs/" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/common/cc26xx/oad/" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/ble5stack/profiles/oad/cc26xx" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/ble5stack/controller/cc26xx/inc" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/ble5stack/inc" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/ble5stack/rom" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/ble5stack/common/cc26xx" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/ble5stack/icall/inc" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/ble5stack/hal/src/target/_common" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/ble5stack/common/cc26xx/npi/stack" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/ble5stack/hal/src/inc" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/ble5stack/heapmgr" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/ble5stack/profiles/dev_info" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/ble5stack/profiles/simple_profile" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/ble5stack/icall/src/inc" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/ble5stack/osal/src/inc" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/ble5stack/services/src/saddr" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/ble5stack/services/src/sdata" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/common/nv" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/common/cc26xx" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/ble5stack/icall/src" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/ble5stack/common/cc26xx/rcosc" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/devices/cc13x2x7_cc26x2x7" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/kernel/tirtos7/packages" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/posix/ticlang" -Duartlog_FILE="\"flash_interface_ext_rtos_NVS.c\"" -DSECURITY -DDeviceFamily_CC26X2X7 -DFLASH_ROM_BUILD -DNVOCMP_NWSAMEITEM=1 -DHEAPMGR_CONFIG=0x80 -DHEAPMGR_SIZE=0x0 -gdwarf-3 -march=armv7e-m -MMD -MP -MF"OAD/flash_interface_ext_rtos_NVS.d_raw" -MT"OAD/flash_interface_ext_rtos_NVS.o" -I"C:/Users/Dushan/workspace_v12/Achilles_DV_V3012_23_05_2023.zip_expanded/project_zero_LP_CC2652R7_tirtos7_ticlang/Release/syscfg" -std=gnu9x $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

OAD/oad.o: C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/ble5stack/profiles/oad/cc26xx/oad.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"C:/ti/ti_cgt_tiarmclang_1.3.0.LTS/bin/tiarmclang.exe" -c @"C:/Users/Dushan/workspace_v12/Achilles_DV_V3012_23_05_2023.zip_expanded/project_zero_LP_CC2652R7_tirtos7_ticlang/Release/syscfg/ti_ble_app_config.opt" @"C:/Users/Dushan/workspace_v12/Achilles_DV_V3012_23_05_2023.zip_expanded/project_zero_LP_CC2652R7_tirtos7_ticlang/Release/syscfg/ti_build_config.opt" @"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/ble5stack/config/build_components.opt" @"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/ble5stack/config/factory_config.opt"  -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -mlittle-endian -mthumb -Oz -I"C:/Users/Dushan/workspace_v12/Achilles_DV_V3012_23_05_2023.zip_expanded/project_zero_LP_CC2652R7_tirtos7_ticlang" -I"C:/Users/Dushan/workspace_v12/Achilles_DV_V3012_23_05_2023.zip_expanded/project_zero_LP_CC2652R7_tirtos7_ticlang/Release" -I"C:/Users/Dushan/workspace_v12/Achilles_DV_V3012_23_05_2023.zip_expanded/project_zero_LP_CC2652R7_tirtos7_ticlang/Application" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/ble5stack" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/drivers/nvs/" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/common/cc26xx/oad/" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/ble5stack/profiles/oad/cc26xx" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/ble5stack/controller/cc26xx/inc" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/ble5stack/inc" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/ble5stack/rom" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/ble5stack/common/cc26xx" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/ble5stack/icall/inc" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/ble5stack/hal/src/target/_common" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/ble5stack/common/cc26xx/npi/stack" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/ble5stack/hal/src/inc" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/ble5stack/heapmgr" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/ble5stack/profiles/dev_info" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/ble5stack/profiles/simple_profile" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/ble5stack/icall/src/inc" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/ble5stack/osal/src/inc" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/ble5stack/services/src/saddr" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/ble5stack/services/src/sdata" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/common/nv" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/common/cc26xx" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/ble5stack/icall/src" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/ble5stack/common/cc26xx/rcosc" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/devices/cc13x2x7_cc26x2x7" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/kernel/tirtos7/packages" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/posix/ticlang" -Duartlog_FILE="\"oad.c\"" -DSECURITY -DDeviceFamily_CC26X2X7 -DFLASH_ROM_BUILD -DNVOCMP_NWSAMEITEM=1 -DHEAPMGR_CONFIG=0x80 -DHEAPMGR_SIZE=0x0 -gdwarf-3 -march=armv7e-m -MMD -MP -MF"OAD/oad.d_raw" -MT"OAD/oad.o" -I"C:/Users/Dushan/workspace_v12/Achilles_DV_V3012_23_05_2023.zip_expanded/project_zero_LP_CC2652R7_tirtos7_ticlang/Release/syscfg" -std=gnu9x $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

OAD/oad_image_header_app.o: C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/ble5stack/profiles/oad/cc26xx/oad_image_header_app.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"C:/ti/ti_cgt_tiarmclang_1.3.0.LTS/bin/tiarmclang.exe" -c @"C:/Users/Dushan/workspace_v12/Achilles_DV_V3012_23_05_2023.zip_expanded/project_zero_LP_CC2652R7_tirtos7_ticlang/Release/syscfg/ti_ble_app_config.opt" @"C:/Users/Dushan/workspace_v12/Achilles_DV_V3012_23_05_2023.zip_expanded/project_zero_LP_CC2652R7_tirtos7_ticlang/Release/syscfg/ti_build_config.opt" @"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/ble5stack/config/build_components.opt" @"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/ble5stack/config/factory_config.opt"  -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -mlittle-endian -mthumb -Oz -I"C:/Users/Dushan/workspace_v12/Achilles_DV_V3012_23_05_2023.zip_expanded/project_zero_LP_CC2652R7_tirtos7_ticlang" -I"C:/Users/Dushan/workspace_v12/Achilles_DV_V3012_23_05_2023.zip_expanded/project_zero_LP_CC2652R7_tirtos7_ticlang/Release" -I"C:/Users/Dushan/workspace_v12/Achilles_DV_V3012_23_05_2023.zip_expanded/project_zero_LP_CC2652R7_tirtos7_ticlang/Application" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/ble5stack" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/drivers/nvs/" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/common/cc26xx/oad/" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/ble5stack/profiles/oad/cc26xx" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/ble5stack/controller/cc26xx/inc" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/ble5stack/inc" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/ble5stack/rom" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/ble5stack/common/cc26xx" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/ble5stack/icall/inc" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/ble5stack/hal/src/target/_common" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/ble5stack/common/cc26xx/npi/stack" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/ble5stack/hal/src/inc" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/ble5stack/heapmgr" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/ble5stack/profiles/dev_info" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/ble5stack/profiles/simple_profile" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/ble5stack/icall/src/inc" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/ble5stack/osal/src/inc" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/ble5stack/services/src/saddr" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/ble5stack/services/src/sdata" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/common/nv" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/common/cc26xx" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/ble5stack/icall/src" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/ble5stack/common/cc26xx/rcosc" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/devices/cc13x2x7_cc26x2x7" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/kernel/tirtos7/packages" -I"C:/ti/simplelink_cc13xx_cc26xx_sdk_6_41_00_17/source/ti/posix/ticlang" -Duartlog_FILE="\"oad_image_header_app.c\"" -DSECURITY -DDeviceFamily_CC26X2X7 -DFLASH_ROM_BUILD -DNVOCMP_NWSAMEITEM=1 -DHEAPMGR_CONFIG=0x80 -DHEAPMGR_SIZE=0x0 -gdwarf-3 -march=armv7e-m -MMD -MP -MF"OAD/oad_image_header_app.d_raw" -MT"OAD/oad_image_header_app.o" -I"C:/Users/Dushan/workspace_v12/Achilles_DV_V3012_23_05_2023.zip_expanded/project_zero_LP_CC2652R7_tirtos7_ticlang/Release/syscfg" -std=gnu9x $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


