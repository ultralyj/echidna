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
#include "tjrc_imageProc.h"

rt_sem_t camera_irq_sem;
rt_sem_t camera_procCplt_sem;
rt_thread_t tid_camera;

static void thread_camera_entry(void *param);

/**
 * @brief 初始化摄像头线程
 * 
 */
void tjrc_thread_camera_init(void)
{
#if CAMERA_BANNED
    rt_kprintf("[WARNNING]camera is banned!!\r\n");
#else
    rt_err_t rtt_res;
    tid_camera = rt_thread_create("camera",thread_camera_entry, RT_NULL,256,5,5);
    if(tid_camera != RT_NULL)
    {
        rtt_res = rt_thread_startup(tid_camera);
        rt_kprintf("[rt-thread]create and startup thread: camera(%d)\r\n",rtt_res);
    }
#endif
}

/**
 * @brief 摄像头入口函数
 * 
 * @param param 
 */
static void thread_camera_entry(void *param)
{
    while(1)
    {
        IfxPort_togglePin(CAMERA_LED);
        rt_thread_mdelay(30);
//        if(rt_sem_take(camera_procCplt_sem, RT_WAITING_FOREVER) == RT_EOK)
//        {
//            /* 翻转camera指示灯 */
//            //printf("[camera]get\r\n");
//            /* 拍摄当前图像 */
//            uint8_t extern sdmmc_detected_flag;
//            if(sdmmc_detected_flag)
//            {
//                extern uint8_t MT9V03X_image_div4[MT9V03X_H*MT9V03X_W/4];
//                tjrc_st7735_drawRectangle(0,154,4,4,RGB565_RED);
//                sprintf((char*)str_buff,"tjrc/b%03d/c%05d.bmp",tjrc_conf_inf.boot_cnt,cnt);
//                tjrc_fileIo_camera2bmp((char*)str_buff,(uint8_t*)MT9V03X_image[0]);
//                tjrc_st7735_drawRectangle(0,154,4,4,RGB565_BLACK);
//            }
//        }
    }
}


