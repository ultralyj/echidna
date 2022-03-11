/*
 * tjrc_systemInit.h
 *
 *  Created on: 2022年3月10日
 *      Author: 11657
 */

#ifndef USER_TJRC_SYSTEMINIT_H_
#define USER_TJRC_SYSTEMINIT_H_

/* hardware：与底层片上外设配置相关代码 */
#include "../Libraries/tjrc_libraries/tjrc_hardware_conf/tjrc_hardware.h"
#include "../Libraries/tjrc_libraries/tjrc_perpherals/tjrc_peripherals.h"
#include "ff.h"


#define SYSTEM_LED LED0_PIN
#define CAMERA_LED LED1_PIN
#define SDMMC_LED LED2_PIN
#define STATUS_LED LED3_PIN //用于检测系统状态，保持闪烁


void tjrc_setHardware(void);
void tjrc_setPeripherals(void);
void tjrc_setFat32(void);

#endif /* USER_TJRC_SYSTEMINIT_H_ */
