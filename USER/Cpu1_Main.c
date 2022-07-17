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
IfxCpu_syncEvent cameraCapture_event = 0;
extern tjrc_image_info tjrc_imageProc_ctr;
rt_tick_t timeStamp_start=0,timeStamp_end=0;
uint8_t str_buff[40];
uint16_t cnt=0;

void core1_camera_service(void);

void core1_main(void)
{
    disableInterrupts();
    /* 发送内核同步信号 */
    IfxCpu_emitEvent(&g_cpuSyncEvent);
    /* 等待内核同步（即CPU0的所有初始化完成） */
    IfxCpu_waitEvent(&g_cpuSyncEvent, 0xFFFF);
    /* 禁用看门狗 */
    IfxScuWdt_disableCpuWatchdog(IfxScuWdt_getCpuWatchdogPassword());
    enableInterrupts();
    printf("[cpu1] turn to run\r\n");

    while (1)
    {
#if !CAMERA_BANNED
        core1_camera_service();
#endif
    }
}

void core1_camera_service(void)
{
    /* 等待DMA采集完成  */
    IfxCpu_emitEvent(&cameraCapture_event);
    IfxCpu_waitEvent(&cameraCapture_event, 0xFFFF);
    cameraCapture_event = 0;
    /* 翻转camera指示灯 */
    IfxPort_togglePin(CAMERA_LED);

    timeStamp_start=rt_tick_get();
    /* 显示灰度图像 */
//    tjrc_mt9v03x_displayImage_gray((uint8_t*)MT9V03X_image[0]);
//    tjrc_mt9v03x_displayImage((uint8_t*)MT9V03X_image[0], 60);
    tjrc_imageProcess((uint8_t*)MT9V03X_image[0],&tjrc_imageProc_ctr);
//    uint8_t* image_out = tjrc_imageProc((uint8_t*)MT9V03X_image[0]);
    timeStamp_end=rt_tick_get();
//    tjrc_st7735_dispImage_gray(image_out,IMAGE_WIDTH, IMAGE_HEIGHT, 0,24);
//    tjrc_st7735_dispStr612(36,12,str_buff,RGB565_MAGENTA);
                /* 显示灰度图像 */
    //            tjrc_mt9v03x_displayImage_gray((uint8_t*)MT9V03X_image[0]);
    //            tjrc_mt9v03x_displayImage((uint8_t*)MT9V03X_image[0], 60);


    sprintf((char*)str_buff,"cnt=%d, t=%dms   ",cnt++,timeStamp_end-timeStamp_start);
//    tjrc_st7735_dispbin((uint8_t*)MT9V03X_image[0], 120, 80, 4, 28, 0xFFFF,90);
    tjrc_st7735_dispStr612(12,12,str_buff,RGB565_MAGENTA);
    /* 清除标志位，开始下一帧采样  */
    tjrc_mt9v03x_clearFinishFlag();
}
