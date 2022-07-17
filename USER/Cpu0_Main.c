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

#include <main.h>

IfxCpu_syncEvent g_cpuSyncEvent = 0;

/**
 * @brief main函数
 * 
 * @return int 
 */
int main(void)
{
    extern rt_sem_t imu_irq_sem;
    extern rt_sem_t camera_irq_sem;
    extern rt_sem_t camera_procCplt_sem;
    imu_irq_sem = rt_sem_create("basem", 0, RT_IPC_FLAG_FIFO);
    camera_irq_sem = rt_sem_create("casem", 0, RT_IPC_FLAG_FIFO);
    camera_procCplt_sem = rt_sem_create("ccpltm", 0, RT_IPC_FLAG_FIFO);
    /* 配置片上外设和片外模块 */
    tjrc_setHardware();
    tjrc_setPeripherals();

    /* 开启各个线程 */
    tjrc_thread_life_init();
    tjrc_thread_key_init();
    tjrc_thread_balance_init();
    tjrc_thread_camera_init();
    tjrc_thread_run_init();

    /* 发送内核同步信号 */
    IfxCpu_emitEvent(&g_cpuSyncEvent);
    //IfxCpu_waitEvent(&g_cpuSyncEvent, 0xFFFF);

    /* 主循环 */
    while(1)
    {
        rt_thread_mdelay(1000);
    }
}



