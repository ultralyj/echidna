/**
 * @file Car_Balance.c
 * @author YYYDS team (1951578@tongji.edu.cn)
 * @brief 卡尔曼滤波算法的程序主体
 * @version 0.1
 * @date 2022-03-10
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "tjrc_kalmam.h"

static void tjrc_kalman(float angle_m, float gyro_m , float* angle_kalman,float*angle_dot);

/**
 * @brief 采集IMU数据，对IMU数据进行换算，最终使用卡尔曼滤波解算姿态信息
 * 
 * @param angle_kalman （输出）卡尔曼滤波估计的角度
 * @param angle_dot （输出）卡尔曼滤波估计的角速度
 */
void tjrc_getAngle_kalman(float* angle_kalman, float* angle_dot)
{
    extern int16_t icm_gyro_x,icm_gyro_y,icm_gyro_z;
    extern int16_t icm_acc_x,icm_acc_y,icm_acc_z;
    float Gyro_Speed=0,Gyro_Speed_X=0;

    /* 获得原始的姿态数据 */
    tjrc_icm20602_getAccel();
    tjrc_icm20602_getGyro();

    /* 对原始数据进行换算 */
	float angleRaw = (float) icm_acc_z/4096;
	Gyro_Speed= -(float)((float)icm_gyro_y*2000/(32768) )*(float)Gyro_Parameter;
	Gyro_Speed_X=-(float)((float)icm_gyro_x*2000/(32768) )*(float)Gyro_Parameter;

	/* 卡尔曼滤波 */
	tjrc_kalman(-angleRaw,Gyro_Speed, angle_kalman, angle_dot);
}

/***********************卡尔曼滤波参数*************************/
const float      dt = 0.0074;                       //0.00568
const float     Q_angle = 0.0005;                   //角度方差0.0057
float           Q_gyro  = 0.000003;                 //角速度方差
float           R_angle = 0.02;                     //测量噪声的方差
const char      C_0     = 1;
float q_bias = 0;                                   //角速度偏差
float angle_err;                                    //角度偏差
float PCt_0, PCt_1, E, K_0, K_1, t_0, t_1;
float P[2][2] = {{ 1, 0 },{ 0, 1 }};

/**
 * @brief 卡尔曼滤波函数：融合重力加速度的角度分量与角速度，系统方程如下，其余可以套用卡尔曼滤波的公式
 *  ^     |x_k - qg_k*t| |t|
    x_k+1=|   qg_k     |+|0|w_k
    y_k = Cx_k+D
    注：设角速度为w，加速度为a，则可以用角度和陀螺仪零漂作为状态变量X=[x,qg]^T，
    卡尔曼滤波在这里的主要思想是，使用陀螺仪测得的角速度作为系统输入，加速度计可以通过分解重量得到角度作为量测量。
 * @param angle_m 原始角度
 * @param gyro_m 原始角速度
 * @param angle_kalman （输出）卡尔曼滤波后的角度
 * @param angle_dot  （输出）卡尔曼滤波后的角速度
 */
static void tjrc_kalman(float angle_m, float gyro_m , float* angle_kalman,float*angle_dot)
{
    /* 先验估计，即求角加速度积分 */
    *angle_kalman += (gyro_m - q_bias) * dt;

    /* 求先验估计的协方差矩阵 */
    P[0][0] += (Q_angle - P[0][1] - P[1][0]) * dt;
    P[0][1] += (-P[1][1]) * dt;
    P[1][0] += (-P[1][1]) * dt;
    P[1][1] += Q_gyro * dt;

    angle_err = angle_m - *angle_kalman;

    PCt_0 = C_0 * P[0][0];
    PCt_1 = C_0 * P[1][0];

    /* 计算卡尔曼增益系数 */
    E = R_angle + C_0 * PCt_0;
    K_0 = PCt_0 / E;
    K_1 = PCt_1 / E;

    /* 更新协方差矩阵 */
    t_0 = PCt_0;
    t_1 = C_0 * P[0][1];
    P[0][0] -= K_0 * t_0;
    P[0][1] -= K_0 * t_1;
    P[1][0] -= K_1 * t_0;
    P[1][1] -= K_1 * t_1;

    /* 卡尔曼最优估计 */
    *angle_kalman += K_0 * angle_err;
    q_bias += K_1 * angle_err;
    *angle_dot = gyro_m - q_bias;
}
