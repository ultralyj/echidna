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
    rt_kprintf("[WARNNING*]IMU is banned!!\r\n");
#else
    rt_err_t rtt_res;
    balance_tid = rt_thread_create("balance", balance_entry, RT_NULL, 1024, 3, 2);
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
    Run_tid = rt_thread_create("run", Run_entry, RT_NULL, 256, 3, 2);
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
    while (1)
    {
        /* 当IMU发出中断信号，依次进行滤波与串级PID */
        if (rt_sem_take(imu_irq_sem, RT_WAITING_FOREVER) == RT_EOK)
        {
            tjrc_motionControl();
        }
    }
}



