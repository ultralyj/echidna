/*
 * tjrc_vadc.h
 *
 *  Created on: 2022年2月23日
 *      Author: 11657
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
