/**
 * @file control.h
 * @author YYYDS team (1951578@tongji.edu.cn)
 * @brief 包含PID控制与电门输出函数的头文件
 * @version 0.1
 * @date 2022-03-11
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef CODE_CONTROL_H_
#define CODE_CONTROL_H_

#include "stdint.h"

#include "tjrc_peripherals.h"
#include "Common_Math.h"
#include "tjrc_wireless.h"


/* 直立环可控参数 */
extern float angle_kp, angle_kd, angle_ki;
/* 速度环可控参数 */
extern float V_S_Kp, V_S_Ki, V_S_Kd;
/* 驱动环可控参数 */
extern float Dr_kp, Dr_ki, Dr_kd;
/* 速度环可控参数 */
extern float direct_kp, direct_ki, direct_kd;
extern float target_speed;
/* PID控制函数 */
int32_t tjrc_pid_balance(float angle_kalman, float angle_dot, float camera_pixelError);
float tjrc_pid_speedLoop(float _V_S_Bias);
int32_t tjrc_pid_drive(float speed);

float tjrc_flyWheel_getSpeed(void);
float tjrc_drive_getSpeed(void);
extern float target_speed;

//----------------方向
float tjrc_pid_servo_balance(float angle_kalman, float angle_dot);
float tjrc_pid_servo_trace(float mid, float slope);

void tjrc_motionControl(void);
#endif /* CODE_CONTROL_H_ */
