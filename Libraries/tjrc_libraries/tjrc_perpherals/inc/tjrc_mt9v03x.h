/**
 * @file tjrc_mt9v03x.h
 * @author YYYDS team (1951578@tongji.edu.cn)
 * @brief MT9V03X系列灰度摄像头驱动程序（总钻风）
 * @version 0.1
 * @date 2022-03-13
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef LIBRARIES_TJRC_LIBRARIES_TJRC_PERPHERALS_TJRC_MT9V032_H_
#define LIBRARIES_TJRC_LIBRARIES_TJRC_PERPHERALS_TJRC_MT9V032_H_

#include "tjrc_asclin.h"
#include "tjrc_peripherals.h"

/* 摄像头读取数据的长宽 */
#define MT9V03X_W 120
#define MT9V03X_H 80

/* 占用DMA通道 */
#define MT9V03X_DMA_CH IfxDma_ChannelId_5

/* 摄像头配置命令枚举 */
typedef enum
{
    INIT = 0,               //摄像头初始化命令
    AUTO_EXP,               //自动曝光命令
    EXP_TIME,               //曝光时间命令
    FPS,                    //摄像头帧率命令
    SET_COL,                //图像列命令
    SET_ROW,                //图像行命令
    LR_OFFSET,              //图像左右偏移命令
    UD_OFFSET,              //图像上下偏移命令
    GAIN,                   //图像偏移命令
    CONFIG_FINISH,          //非命令位，主要用来占位计数

    COLOR_GET_WHO_AM_I = 0xEF,
    SET_EXP_TIME = 0XF0,    //单独设置曝光时间命令
    GET_STATUS,             //获取摄像头配置命令
    GET_VERSION,            //固件版本号命令

    SET_ADDR = 0XFE,        //寄存器地址命令
    SET_DATA                //寄存器数据命令
}MT9V03_CMD;

extern uint8_t MT9V03X_image[MT9V03X_H][MT9V03X_W];

/* 摄像头采集状态感知 */
uint8_t  tjrc_mt9v03x_getFinishFlag(void);
void  tjrc_mt9v03x_clearFinishFlag(void);

/* 摄像头基本配置信息的建立与读取 */
uint8_t tjrc_setMt9v03x(void);
uint8_t tjrc_mt9v03x_setConf(sint16 buff[CONFIG_FINISH - 1][2]);
uint8_t tjrc_mt9v03x_getConf(sint16 buff[CONFIG_FINISH - 1][2]);
uint16_t tjrc_mt9v03x_getVision(void);
/* 测试函数 */
void tjrc_mt9v03x_displayImage(uint8_t* image, uint8_t threshold);
void tjrc_mt9v03x_displayImage_gray(uint8_t* image);
/* 中断回调函数 */
void tjrc_mt9v03x_uartCallBack(void);
void tjrc_mt9v03x_vSync(void);
void tjrc_mt9v03x_dmaCallBack(void);

#endif /* LIBRARIES_TJRC_LIBRARIES_TJRC_PERPHERALS_TJRC_MT9V032_H_ */
