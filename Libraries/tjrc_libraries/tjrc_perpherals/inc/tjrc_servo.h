/**
 * @file tjrc_servo.h
 * @author YYYDS team (1951578@tongji.edu.cn)
 * @brief 舵机驱动
 * @version 0.1
 * @date 2022-03-13
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef LIBRARIES_TJRC_LIBRARIES_TJRC_PERPHERALS_TJRC_SERVO_H_
#define LIBRARIES_TJRC_LIBRARIES_TJRC_PERPHERALS_TJRC_SERVO_H_

#include "tjrc_peripherals.h"

#define SERVO_MAX_ANGLE 26.0f

float tjrc_servo_setAngle(float servoAngle);



#endif /* LIBRARIES_TJRC_LIBRARIES_TJRC_PERPHERALS_TJRC_SERVO_H_ */
