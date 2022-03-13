/**
 * @file tjrc_ccu6.h
 * @author YYYDS team (1951578@tongji.edu.cn)
 * @brief CCU6硬件配置层头文件，产生互补PWM波
 * @version 0.1
 * @date 2022-03-13
 * 
 * @copyright Copyright (c) 2022
 * 
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
