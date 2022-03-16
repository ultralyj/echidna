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

extern IfxCpu_syncEvent g_cpuSyncEvent;
extern IfxCpu_syncEvent cameraCapture_event;

void core1_main(void)
{
    enableInterrupts();
	/* 发送内核同步信号 */
	IfxCpu_emitEvent(&g_cpuSyncEvent);
	/* 等待内核同步（即CPU0的所有初始化完成） */
	IfxCpu_waitEvent(&g_cpuSyncEvent, 0xFFFF);
	/* 禁用看门狗 */
    IfxScuWdt_disableCpuWatchdog(IfxScuWdt_getCpuWatchdogPassword());

    printf("[cpu1] turn to run\r\n");
    while (TRUE)
    {
        IfxCpu_emitEvent(&cameraCapture_event);
        IfxCpu_waitEvent(&cameraCapture_event, 0xFFFF);
        cameraCapture_event = 0;
       // printf("!");
    }
}

