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

#include "../../tjrc_perpherals/tjrc_peripherals.h"
#include "tjrc_gpt12.h"
#include "Common_Math.h"
#include "Peripheral.h"


/* 直立环可控参数 */
extern float angle_kp, angle_kd, angle_ki;
/* 速度环可控参数 */
extern float V_S_Kp, V_S_Ki, V_S_Kd;

/* PID控制函数 */
int32_t tjrc_pid_balance(float angle_kalman, float angle_dot);
float tjrc_pid_speedLoop(void);
int32_t tjrc_pid_drive(void);

extern float target_speed;

//----------------方向
float Turn_out(void);
extern float Turn_delta;
#endif /* CODE_CONTROL_H_ */
