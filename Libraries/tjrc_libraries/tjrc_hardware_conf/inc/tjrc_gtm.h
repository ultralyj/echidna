/*
 * tjrc_gtm.h
 *
 *  Created on: 2022年2月23日
 *      Author: 11657
 */

#ifndef LIBRARIES_TJRC_LIBRARIES_TJRC_GTM_H_
#define LIBRARIES_TJRC_LIBRARIES_TJRC_GTM_H_

#include "IfxGtm_Tom_Pwm.h"
#include "IfxGtm_Atom_Pwm.h"
#include "rtthread.h"
#include "stdint.h"

#define SERVO_GTM_ATOM_CHANNEL &IfxGtm_ATOM0_0_TOUT53_P21_2_OUT
/* 目前，舵机输出信号引脚为P21.2，即 IfxGtm_ATOM0_0_TOUT53_P21_2_OUT */
void tjrc_setGtmAtom_pwm(IfxGtm_Atom_ToutMap *GtmToutMap, uint32_t PWM_Freq);
void tjrc_gtmAtom_setDutyCycle(IfxGtm_Atom_ToutMap *GtmToutMap, uint32_t dutyCycle);

#endif /* LIBRARIES_TJRC_LIBRARIES_TJRC_GTM_H_ */
