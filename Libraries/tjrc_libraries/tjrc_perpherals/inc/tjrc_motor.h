/*
 * tjrc_motor.h
 *
 *  Created on: 2022年3月10日
 *      Author: 11657
 */

#ifndef LIBRARIES_TJRC_LIBRARIES_TJRC_PERPHERALS_TJRC_MOTOR_H_
#define LIBRARIES_TJRC_LIBRARIES_TJRC_PERPHERALS_TJRC_MOTOR_H_

#include "tjrc_peripherals.h"

#define FLYWHEEL_MOTOR_DIR_PIN &MODULE_P33, 12
#define FLYWHEEL_MOTOR_EN_PIN &MODULE_P20, 6

#define FLYWHEEL_MOTOR_GTM_ATOM_CHANNEL IfxGtm_ATOM2_5_TOUT35_P33_13_OUT

void tjrc_setFlyWheelMotor(void);
void tjrc_flyWheelMotor_pwm(int32_t duty);

#endif /* LIBRARIES_TJRC_LIBRARIES_TJRC_PERPHERALS_TJRC_MOTOR_H_ */
