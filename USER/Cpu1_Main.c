/**
 * @file Cpu1_Main.c
 * @author YYYDS team (1951578@tongji.edu.cn)
 * @brief CPU1 main文件
 * @version 0.1
 * @date 2022-03-13
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "main.h"
#include "headfile.h"
#pragma section all "cpu1_dsram"


void core1_main(void)
{
	disableInterrupts();
	/* 发送内核同步信号 */
	IfxCpu_emitEvent(&g_cpuSyncEvent);
	IfxCpu_waitEvent(&g_cpuSyncEvent, 0xFFFF);
	/* 禁用看门狗 */
    IfxScuWdt_disableCpuWatchdog(IfxScuWdt_getCpuWatchdogPassword());

    enableInterrupts();
    /* 等待所有外设初始化完成 */
    extern uint8_t sysInit_cpltFlag;
    while(!sysInit_cpltFlag);
    while (TRUE)
    {

        printf("[cpu1] ok\r\n");
        while(1);
    }
}



#pragma section all restore
