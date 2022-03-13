/**
 * @file tjrc_vadc.h
 * @author YYYDS team (1951578@tongji.edu.cn)
 * @brief VADC硬件配置层头文件，用于采集模拟电压信号
 * @version 0.1
 * @date 2022-03-13
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef LIBRARIES_TJRC_LIBRARIES_TJRC_VADC_H_
#define LIBRARIES_TJRC_LIBRARIES_TJRC_VADC_H_

#include "rtthread.h"
#include "stdint.h"

#include "IfxVadc_Adc.h"

#define ADC_3V3_Pin 7
#define ADC_5V0_Pin 6
#define ADC_VBAT_Pin 8

void tjrc_setVadc_pin(uint8_t apinIndex);
uint16_t tjrc_vadc_getAnalog(uint8_t apinIndex);

#endif /* LIBRARIES_TJRC_LIBRARIES_TJRC_VADC_H_ */
