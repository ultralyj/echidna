/**
 * @file Car_Balance.h
 * @author YYYDS team (1951578@tongji.edu.cn)
 * @brief 卡尔曼滤波，平滑姿态的头文件，调用主函数，即可采集imu数据并进行卡尔曼滤波输出角度
 * @version 0.1
 * @date 2022-03-10
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef __CAR_BALANCE_H
#define __CAR_BALANCE_H

#include "tjrc_icm20602.h"

#define	    pi	(3.141592654)				//圆周率
#define     Gyro_Parameter ( pi / 180 )   //  1sec/s =  Gyro_Parameter *  rad/s

void tjrc_getAngle_kalman(float* angle_kalman, float* angle_dot);

#endif
