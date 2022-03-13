/*
 * thread_key.c
 *
 *  Created on: 2022年3月12日
 *      Author: 11657
 */

#include "tjrc_threads.h"

static void tjrc_thread_key_entry(void *param);

rt_sem_t key0_sem,key1_sem,key2_sem,key3_sem;
rt_thread_t key_tid = RT_NULL;

void tjrc_thread_key_init(void)
{
    rt_err_t rtt_res;
    /* 创建信号量 */
    key0_sem = rt_sem_create("k0sem", 0, RT_IPC_FLAG_FIFO);
    key1_sem = rt_sem_create("k1sem", 0, RT_IPC_FLAG_FIFO);
    key2_sem = rt_sem_create("k2sem", 0, RT_IPC_FLAG_FIFO);
    key3_sem = rt_sem_create("k3sem", 0, RT_IPC_FLAG_FIFO);
    /* 创建，并运行按键扫描线程 */
    key_tid = rt_thread_create("key", tjrc_thread_key_entry, RT_NULL, 256, 5, 2);
    if (key_tid != RT_NULL)
    {
        rtt_res = rt_thread_startup(key_tid);
        rt_kprintf("[rt-thread]create and startup thread: key(%d)\r\n",rtt_res);
    }
}

static void tjrc_thread_key_entry(void *param)
{
    while(1)
    {
        if(tjrc_keyChannel_check(KEY0)==KEY_Pressed)
        {
            rt_sem_release(key0_sem);
        }
        if(tjrc_keyChannel_check(KEY1)==KEY_Pressed)
        {
            rt_sem_release(key1_sem);
        }
        if(tjrc_keyChannel_check(KEY2)==KEY_Pressed)
        {
            rt_sem_release(key2_sem);
        }
        if(tjrc_keyChannel_check(KEY3)==KEY_Pressed)
        {
            rt_sem_release(key3_sem);
        }
        rt_thread_mdelay(10);
    }

}

