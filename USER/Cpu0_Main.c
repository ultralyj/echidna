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

#include "Cpu0_Main.h"

IFX_ALIGN(4) IfxCpu_syncEvent g_cpuSyncEvent = 0;

uint8_t sysInit_cpltFlag = 0;
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
    extern rt_sem_t camera_irq_sem;
    imu_irq_sem = rt_sem_create("basem", 0, RT_IPC_FLAG_FIFO);
    camera_irq_sem = rt_sem_create("casem", 0, RT_IPC_FLAG_FIFO);
    /* 配置片上外设和片外模块 */


    tjrc_setHardware();
    tjrc_setPeripherals();

    balance_thread_init();
    tjrc_thread_life_init();
    tjrc_thread_key_init();
    tjrc_thread_camera_init();
    sysInit_cpltFlag = 1;
    /* 主循环 */
    while(1)
    {
        rt_thread_mdelay(500);
    }
}



