/**
 * @file tjrc_gtm.h
 * @author YYYDS team (1951578@tongji.edu.cn)
 * @brief GTM硬件配置层头文件，用于产生PWM波
 * @version 0.1
 * @date 2022-03-13
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef LIBRARIES_TJRC_LIBRARIES_TJRC_GTM_H_
#define LIBRARIES_TJRC_LIBRARIES_TJRC_GTM_H_

#include "IfxGtm_Tom_Pwm.h"
#include "IfxGtm_Atom_Pwm.h"
#include "rtthread.h"
#include "stdint.h"

/* 目前，舵机输出信号引脚为P21.2，即 IfxGtm_ATOM0_0_TOUT53_P21_2_OUT */
#define SERVO_GTM_ATOM_CHANNEL &IfxGtm_ATOM0_0_TOUT53_P21_2_OUT

void tjrc_setGtmAtom_pwm(IfxGtm_Atom_ToutMap *GtmToutMap, uint32_t PWM_Freq);
void tjrc_gtmAtom_setDutyCycle(IfxGtm_Atom_ToutMap *GtmToutMap, uint32_t dutyCycle);

#endif /* LIBRARIES_TJRC_LIBRARIES_TJRC_GTM_H_ */
