/*
 * tjrc_ccu6.h
 *
 *  Created on: 2022年2月23日
 *      Author: 11657
 */

#ifndef LIBRARIES_TJRC_LIBRARIES_TJRC_CCU6_H_
#define LIBRARIES_TJRC_LIBRARIES_TJRC_CCU6_H_

#include "IfxCcu6_PwmHl.h"
#include "rtthread.h"
#include "stdint.h"

void tjrc_setCcu60_pwm(void);
void tjrc_setCcu61_pwm(void);
void tjrc_ccu60pwm_setDutyCycle(uint32_t *dutyCycle, uint8_t chanNum);
void tjrc_ccu61pwm_setDutyCycle(uint32_t *dutyCycle, uint8_t chanNum);

#endif /* LIBRARIES_TJRC_LIBRARIES_TJRC_CCU6_H_ */
