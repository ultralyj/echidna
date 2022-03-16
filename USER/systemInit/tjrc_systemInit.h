/**
 * @file tjrc_systemInit.h
 * @author YYYDS team (1951578@tongji.edu.cn)
 * @brief 
 * @version 0.1
 * @date 2022-03-13
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef USER_TJRC_SYSTEMINIT_H_
#define USER_TJRC_SYSTEMINIT_H_

/* hardware：与底层片上外设配置相关代码 */
#include "tjrc_hardware.h"
#include "tjrc_peripherals.h"
#include "tjrc_algorithm.h"
#include "ff.h"


#define SYSTEM_LED LED0_PIN
#define CAMERA_LED LED1_PIN
#define SDMMC_LED LED2_PIN
#define STATUS_LED LED3_PIN //用于检测系统状态，保持闪烁


void tjrc_setHardware(void);
void tjrc_setPeripherals(void);
void tjrc_setFat32(void);

#endif /* USER_TJRC_SYSTEMINIT_H_ */
