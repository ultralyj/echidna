/**
 * @file Scheduling.c
 * @author YYYDS team (1951578@tongji.edu.cn)
 * @brief
 * @version 0.1
 * @date 2022-03-10
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "Scheduling.h"

/* 速度环使能标志变量 */
const uint8_t speedLoop_enable = 1;
/* 后轮驱动使能标志变量 */
const uint8_t drive_enable = 1;
/* 转向控制使能标志变量 */
const uint8_t turn_enable = 1;

/**
 * @brief IMU中断信号量
 *
 */
rt_sem_t imu_irq_sem = RT_NULL;

/**
 * @brief PID控制线程入口函数
 * 
 * @param parameter 
 */
static void balance_entry(void *parameter)
{
    static uint8_t motor_enable;
    /* 速度环分频变量：1/20，后轮驱动分频：1/5，转向分频：1/2 */
    static uint8_t speedLoop_cnt = 0, drive_cnt = 0, turn_cnt = 0;
    /* 卡尔曼滤波初始化计数器 */
    static uint8_t kalman_initCnt = 0;
    /* 函数中间参数传递变量 */
    float b_angle_delta = 0;
    float b_angle_kalman, b_angle_dot;
    float b_turn_delta = 0;
    /* PWM输出变量 */
    int flywheel_pwmOut = 0, drive_pwmOut = 0;
    static uint32_t dutyr[3] = {50, 50, 50};

    while (1)
    {
        /* 当IMU发出中断信号，依次进行滤波与串级PID */
        if (rt_sem_take(imu_irq_sem, RT_WAITING_FOREVER) == RT_EOK)
        {
            /* 特别注意：等待IMU角度稳定后(约200*8ms=1.6s)再开启电机 */
            if (kalman_initCnt > 200 && motor_enable == 0)
            {
                motor_enable = 1;
                IfxPort_setPinHigh(FLYWHEEL_MOTOR_EN_PIN);
                kalman_initCnt = 0;
            }
            else
            {
                kalman_initCnt++;
            }
            /* 获取IMU信息，进行卡尔曼滤波，再输入到直立环PID得到动量轮PWM输出 */
            tjrc_getAngle_kalman(&b_angle_kalman, &b_angle_dot); //得到计算的初始角度
            flywheel_pwmOut = tjrc_pid_balance(b_angle_kalman, b_angle_dot);
            
            /* 在电机使能的状态下，平行处理动量轮速度环，后轮驱动和舵机 */
            if (motor_enable)
            {
                /* 动量轮速度环（1/20） */
                if (speedLoop_enable)
                {
                    if (speedLoop_cnt == 20)
                    {
                        speedLoop_cnt = 0;
                        b_angle_delta = tjrc_pid_speedLoop();
                    }
                    speedLoop_cnt++;
                }
                

                /* 舵机方向（1/2） */
                if (turn_enable)
                {
                    if (turn_cnt == 2)
                    {
                        turn_cnt = 0;
                        b_turn_delta = Turn_out();
                    }
                    turn_cnt++;
                }
                /* 后轮驱动（1/5） */
                if (drive_enable)
                {
                    if (drive_cnt == 5)
                    {
                        drive_cnt = 0;
                        drive_pwmOut = tjrc_pid_drive();
                        /* 输入CCU60，输出互补PWM波 */
                        dutyr[0] = (uint32_t)s32_AmpConstrain(0, 100, drive_pwmOut + 50);
                        tjrc_ccu60pwm_setDutyCycle(dutyr, 3);
                    }
                    drive_cnt++;
                }

                /* 偏移角过大，停转保护 */
                if (b_angle_kalman > 0.35 || b_angle_kalman < -0.6)
                {
                    flywheel_pwmOut = 0;
                    IfxPort_setPinLow(FLYWHEEL_MOTOR_EN_PIN);
                }
                extern float Angle_zero;
                Angle_zero += b_angle_delta + b_turn_delta;                         //叠加串联输出
                flywheel_pwmOut = s32_AmpConstrain(-9000, 9000, flywheel_pwmOut); //平衡环限幅
                tjrc_flyWheelMotor_pwm(flywheel_pwmOut);
            }
        }
    }
}

static rt_thread_t balance_tid = RT_NULL;

uint8_t balance_thread_flag = 0;
void balance_thread_init(void)
{
    balance_tid = rt_thread_create("balance_thread", balance_entry, RT_NULL, 1024, 3, 30);
    if (balance_tid != RT_NULL)
    {
        rt_thread_startup(balance_tid);
    }
    balance_thread_flag = 1;
}

static void Run_entry(void *parameter)
{
    while (1)
    {
        if (rt_sem_take(Run_sem, RT_WAITING_FOREVER) == RT_EOK)
        {
            if (run_direct == 1)
                target_speed += 100;
            else
                target_speed -= 100;
        }
    }
}

static rt_thread_t Run_tid = RT_NULL;
rt_sem_t Run_sem = RT_NULL;
void Run_init(void)
{
    uint32_t duty[3] = {50, 50, 50};
    Run_tid = rt_thread_create("Run_thread", Run_entry, RT_NULL, 512, 26, 30);
    if (Run_tid != RT_NULL)
    {
        rt_thread_startup(Run_tid);
    }
    tjrc_setCcu60_pwm();
    tjrc_ccu60pwm_setDutyCycle(duty, 1);
    Run_sem = rt_sem_create("Run_sem", 0, RT_IPC_FLAG_FIFO);
}
