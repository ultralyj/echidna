/**
 * @file thread_life.c 
 * @author YYYDS team (1951578@tongji.edu.cn)
 * @brief 生命检测线程，判读程序有没有跑飞的第一判据
 * @version 0.1
 * @date 2022-03-13
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "tjrc_threads.h"

static void thread_life_entry(void *param);

rt_thread_t tid_life = RT_NULL;

/**
 * @brief 初始化生命检测线程
 * 
 */
void tjrc_thread_life_init(void)
{
    rt_err_t rtt_res;
    /* 创建，并运行按键扫描线程 */
    tid_life = rt_thread_create("life",thread_life_entry, RT_NULL,128,9,2);
    if(tid_life != RT_NULL)
    {
        rtt_res = rt_thread_startup(tid_life);
        rt_kprintf("[rt-thread]create and startup thread: life(%d)\r\n",rtt_res);

    }
}

/**
 * @brief 程序生命监测入口，通过状态灯的闪烁，判断程序是否死机
 *
 * @param param
 */
static void thread_life_entry(void *param)
{
    while(1)
    {
        IfxPort_togglePin(STATUS_LED);
        rt_thread_mdelay(500);
    }
}

