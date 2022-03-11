/**
 * @file Cpu0_Main.c
 * @author YYYDS team (1951578@tongji.edu.cn)
 * @brief CPU0 main文件
 * @version 0.1
 * @date 2022-02-25
 * 
 * @copyright Copyright (c) 2022
 * 
 */

/* rt-thread头文件 */
#include <tjrc_hardware.h>
#include <tjrc_peripherals.h>
#include "rtthread.h"
/* hardware：与底层片上外设配置相关代码 */
#include "tjrc_algorithm.h"
#include "ff.h"
#include "tjrc_systemInit.h"

IFX_ALIGN(4) IfxCpu_syncEvent g_cpuSyncEvent = 0;


void thread_life_entry(void *param);
void thread_camera_entry(void *param);

/**
 * @brief main函数
 * 
 * @return int 
 */
int main(void)
{
    IfxCpu_emitEvent(&g_cpuSyncEvent);
    IfxCpu_waitEvent(&g_cpuSyncEvent, 0xFFFF);

    extern rt_sem_t imu_irq_sem;
    imu_irq_sem = rt_sem_create("balance", 0, RT_IPC_FLAG_FIFO);
    /* 配置片上外设和片外模块 */


    tjrc_setHardware();
    tjrc_setPeripherals();

    //balance_thread_init();
    /* 创建生命监测线程 */
    rt_thread_t tid_life;
    rt_err_t rtt_res;
    tid_life = rt_thread_create("life",thread_life_entry, RT_NULL,256,9,2);
    if(tid_life != RT_NULL)
    {
        rtt_res = rt_thread_startup(tid_life);
        rt_kprintf("[rt-thread]create and startup thread: life(%d)\r\n",rtt_res);
    }

    rt_thread_t tid_camera;
    tid_camera = rt_thread_create("camera",thread_camera_entry, RT_NULL,256,8,2);
    if(tid_camera != RT_NULL)
    {
        rtt_res = rt_thread_startup(tid_camera);
        rt_kprintf("[rt-thread]create and startup thread: camera(%d)\r\n",rtt_res);
    }
    /* 主循环 */


    while(1)
    {
        rt_thread_mdelay(500);
    }
}

/**
 * @brief 程序生命监测入口，通过状态灯的闪烁，判断程序是否死机
 *
 * @param param
 */
void thread_life_entry(void *param)
{
    while(1)
    {
        IfxPort_togglePin(STATUS_LED);
        rt_thread_mdelay(500);
    }
}

void thread_camera_entry(void *param)
{
    extern uint8 MT9V03X_image[MT9V03X_H][MT9V03X_W];
    uint8_t str_buff[40];
    uint16_t cnt=0;
    while(1)
    {
        IfxPort_togglePin(CAMERA_LED);
        while (!tjrc_mt9v032_getFinishFlag());
        sprintf((char*)str_buff,"cnt=%d   ",cnt++);
        tjrc_st7735_dispStr612(0,0,str_buff,RGB565_YELLOW);

        Ifx_MT9V03X_displayImage(MT9V03X_image[0],30);
        tjrc_mt9v03x_clearFinishFlag();
        /* 延时，为了切出到其他线程 */
        rt_thread_mdelay(100);
    }
}
