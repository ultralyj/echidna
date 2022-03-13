/*
 * thread_camera.c
 *
 *  Created on: 2022年3月12日
 *      Author: 11657
 */


#include "tjrc_threads.h"

rt_sem_t camera_irq_sem;
rt_thread_t tid_camera;

static void thread_camera_entry(void *param);

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

static void thread_camera_entry(void *param)
{
    extern uint8_t MT9V03X_image[MT9V03X_H][MT9V03X_W];
    extern rt_sem_t key0_sem;
    uint8_t str_buff[40];
    uint16_t cnt=0;
    while(1)
    {
        if(rt_sem_take(camera_irq_sem, RT_WAITING_FOREVER) == RT_EOK)
        {
            IfxPort_togglePin(CAMERA_LED);
            sprintf((char*)str_buff,"cnt=%d   ",cnt++);
            tjrc_st7735_dispStr612(0,0,str_buff,RGB565_YELLOW);
            /* 显示灰度图像 */
            Ifx_MT9V03X_displayImage_gray((uint8_t*)MT9V03X_image[0]);
            if(rt_sem_take(key0_sem, RT_WAITING_NO) == RT_EOK)
            {
                /* 拍摄当前图像 */
                uint8_t extern sdmmc_detected_flag;
                if(sdmmc_detected_flag)
                {
                    extern uint8_t MT9V03X_image_div4[MT9V03X_H*MT9V03X_W/4];
                    sprintf((char*)str_buff,"tjrc/c%05d.bmp",cnt);
                    tjrc_fileIo_camera2bmp((char*)str_buff,MT9V03X_image_div4);
                }

            }
            tjrc_mt9v03x_clearFinishFlag();
        }

    }
}


