/*
 * thread_key.c
 *
 *  Created on: 2022年3月12日
 *      Author: 11657
 */

#include "tjrc_threads.h"

static void thread_life_entry(void *param);

rt_thread_t tid_life = RT_NULL;

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

