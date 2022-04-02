/**
 * @file tjrc_motor.h
 * @author YYYDS team (1951578@tongji.edu.cn)
 * @brief 电机驱动程序，目前支持按照DRV8701E和DRV8701P的逻辑驱动电机
 * @version 0.1
 * @date 2022-03-13
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef LIBRARIES_TJRC_LIBRARIES_TJRC_PERPHERALS_TJRC_MOTOR_H_
#define LIBRARIES_TJRC_LIBRARIES_TJRC_PERPHERALS_TJRC_MOTOR_H_

#include "tjrc_peripherals.h"


#define FLYWHEEL_MOTOR_DIR_PIN &MODULE_P33, 12
#define DRIVE_MOTOR_DIR_PIN &MODULE_P02, 0
#define FLYWHEEL_MOTOR_EN_PIN &MODULE_P20, 6

#define FLYWHEEL_MOTOR_GTM_ATOM_CHANNEL IfxGtm_ATOM2_5_TOUT35_P33_13_OUT
#define DRIVE_MOTOR_GTM_ATOM_CHANNEL    IfxGtm_ATOM0_1_TOUT1_P02_1_OUT

void tjrc_setFlyWheelMotor(void);
void tjrc_flyWheelMotor_pwm(int32_t duty);
void tjrc_driveMotor_pwm(int32_t duty);

#endif /* LIBRARIES_TJRC_LIBRARIES_TJRC_PERPHERALS_TJRC_MOTOR_H_ */
