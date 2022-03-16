/**
 * @file thread_camera.c
 * @author YYYDS team (1951578@tongji.edu.cn)
 * @brief 摄像头线程
 * @version 0.1
 * @date 2022-03-13
 * 
 * @copyright Copyright (c) 2022
 * 
 */


#include "tjrc_threads.h"

rt_sem_t camera_irq_sem;
rt_thread_t tid_camera;

IfxCpu_syncEvent cameraCapture_event = 0;

static void thread_camera_entry(void *param);

/**
 * @brief 初始化摄像头线程
 * 
 */
void tjrc_thread_camera_init(void)
{
    rt_err_t rtt_res;
    tid_camera = rt_thread_create("camera",thread_camera_entry, RT_NULL,256,8,2);
    if(tid_camera != RT_NULL)
    {
        rtt_res = rt_thread_startup(tid_camera);
        rt_kprintf("[rt-thread]create and startup thread: camera(%d)\r\n",rtt_res);
    }
}

/**
 * @brief 摄像头入口函数
 * 
 * @param param 
 */
static void thread_camera_entry(void *param)
{
    extern uint8_t MT9V03X_image[MT9V03X_H][MT9V03X_W];
    extern rt_sem_t key0_sem;
    uint8_t str_buff[40];
    uint16_t cnt=0;
    while(1)
    {
        /* 等待摄像头采集完毕的信号量 */
        if(rt_sem_take(camera_irq_sem, RT_WAITING_FOREVER) == RT_EOK)
        {
            IfxPort_togglePin(CAMERA_LED);
            sprintf((char*)str_buff,"cnt=%d   ",cnt++);
            tjrc_st7735_dispStr612(36,12,str_buff,RGB565_MAGENTA);
            /* 显示灰度图像 */
            tjrc_mt9v03x_displayImage_gray((uint8_t*)MT9V03X_image[0]);
            /* 发送事件通知CPU1处理图像 */
            IfxCpu_emitEvent(&cameraCapture_event);
            /* 非阻塞查询是否有按键信号量，若有则进入拍照状态 */
            if(rt_sem_take(key0_sem, RT_WAITING_NO) == RT_EOK)
            {
                /* 拍摄当前图像 */
                uint8_t extern sdmmc_detected_flag;
                if(sdmmc_detected_flag)
                {
                    extern uint8_t MT9V03X_image_div4[MT9V03X_H*MT9V03X_W/4];
                    tjrc_st7735_drawRectangle(0,154,4,4,RGB565_RED);
                    sprintf((char*)str_buff,"tjrc/b%03d/c%05d.bmp",tjrc_conf_inf.boot_cnt,cnt);
                    tjrc_fileIo_camera2bmp((char*)str_buff,MT9V03X_image_div4);
                    tjrc_st7735_drawRectangle(0,154,4,4,RGB565_BLACK);
                }
            }
            tjrc_mt9v03x_clearFinishFlag();
        }

    }
}

