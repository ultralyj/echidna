/**
 * @file thread_balance.c
 * @author YYYDS team (1951578@tongji.edu.cn)
 * @brief 平衡线程
 * @version 0.1
 * @date 2022-03-13
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "tjrc_threads.h"
#include "math.h"
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
rt_sem_t Run_sem = RT_NULL;

uint8_t balance_thread_flag = 0;
rt_thread_t balance_tid = RT_NULL;
rt_thread_t Run_tid = RT_NULL;

static void balance_entry(void *parameter);
static void Run_entry(void *parameter);

void tjrc_thread_balance_init(void)
{
#if IMU_BANNED
    rt_kprintf("[WARNNING]IMU is banned!!\r\n");
#else
    rt_err_t rtt_res;
    balance_tid = rt_thread_create("balance", balance_entry, RT_NULL, 1024, 3, 30);
    if (balance_tid != RT_NULL)
    {
        rtt_res = rt_thread_startup(balance_tid);
        rt_kprintf("[rt-thread]create and startup thread: balance(%d)\r\n",rtt_res);
    }
    balance_thread_flag = 1;
#endif
}

void tjrc_thread_run_init(void)
{

    rt_err_t rtt_res;
    Run_sem = rt_sem_create("Run_sem", 0, RT_IPC_FLAG_FIFO);
    Run_tid = rt_thread_create("run", Run_entry, RT_NULL, 512, 26, 30);
    if (Run_tid != RT_NULL)
    {
        rtt_res = rt_thread_startup(Run_tid);
        rt_kprintf("[rt-thread]create and startup thread: run(%d)\r\n",rtt_res);
    }
}


static void Run_entry(void *parameter)
{
    extern float target_speed;
    extern uint8_t run_direct;
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


/**
 * @brief PID控制线程入口函数
 *
 * @param parameter
 */
static void balance_entry(void *parameter)
{
    extern float Angle_zero;
    static int32_t motor_enable;
    /* 速度环分频变量：1/20，后轮驱动分频：1/5，转向分频：1/2 */
    static uint8_t speedLoop_cnt = 0, drive_cnt = 0, turn_cnt = 0;
    /* 卡尔曼滤波初始化计数器 */
    static int32_t kalman_initCnt = 0;
    /* 函数中间参数传递变量 */
    float b_angle_delta = 0.0f;
    float b_angle_kalman = 0.0f , b_angle_dot = 0.0f;
    float b_turn_delta = 0.0f;

    float b_flyWheel_speed = 0.0f, b_drive_speed = 0.0f;

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
            if (motor_enable==1)
            {
                /* 动量轮速度环（1/20） */
                if (speedLoop_enable)
                {
                    if (speedLoop_cnt == 20)
                    {
                        speedLoop_cnt = 0;
                        b_flyWheel_speed = tjrc_flyWheel_getSpeed();
                        b_angle_delta = tjrc_pid_speedLoop(b_flyWheel_speed);
                        uint8_t str[30];
                        sprintf((char*)str,"%f,%f,%f\n",b_flyWheel_speed,b_angle_kalman,Angle_zero);
                        tjrc_asclin1_sendStr(str);
                        printf("%s\r\n",str);
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
                        b_drive_speed = tjrc_drive_getSpeed();
                        drive_pwmOut = tjrc_pid_drive(b_drive_speed);
                        /* 输入CCU60，输出互补PWM波 */
                        tjrc_driveMotor_pwm(drive_pwmOut);
                    }
                    drive_cnt++;
                }

                /* 偏移角过大，停转保护 */
                if (b_angle_kalman > 0.35 || b_angle_kalman < -0.6 || fabs(b_flyWheel_speed) > 20000)
                {
                    flywheel_pwmOut = 0;
                    motor_enable = -1;
                    IfxPort_setPinLow(FLYWHEEL_MOTOR_EN_PIN);
                }
                extern float Angle_zero;
                Angle_zero -= b_angle_delta + b_turn_delta;                         //叠加串联输出
                flywheel_pwmOut = s32_AmpConstrain(-9000, 9000, flywheel_pwmOut); //平衡环限幅
                tjrc_flyWheelMotor_pwm(-flywheel_pwmOut);
            }
        }
    }
}



