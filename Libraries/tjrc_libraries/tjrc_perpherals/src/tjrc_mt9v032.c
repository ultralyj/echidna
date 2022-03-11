/*
 * Ifx_MT9V03X.c
 *
 *  Created on: 2021年3月14日
 *      Author: 11657
 */

#include "tjrc_mt9v032.h"

/* 摄像头需要用到的全局变量 */
static uint8_t recBuff[3];
static uint8_t MT9V03X_linkListNum = 0, recFlag = 0, MT9V03X_finishFlag = 0, MT9V03X_dmaCnt = 0;

/* 摄像头存储单帧灰度图像的数组(4字节对齐) */
uint8_t *cameraBufferAddr = NULL;
IFX_ALIGN(4) uint8_t MT9V03X_image[MT9V03X_H][MT9V03X_W];

/**
 * @brief 从摄像头读取的配置文件
 * 
 */
int16_t MT9V03X_confGet[CONFIG_FINISH - 1][2] =
    {
        {AUTO_EXP, 0},
        {EXP_TIME, 0},
        {FPS, 0},
        {SET_COL, 0},
        {SET_ROW, 0},
        {LR_OFFSET, 0},
        {UD_OFFSET, 0},
        {GAIN, 0},
};

/**
 * @brief 向摄像头写入的配置文件
 * 
 */
int16_t MT9V03X_CFG[CONFIG_FINISH][2] =
    {
        {AUTO_EXP, 0},        //自动曝光设置      范围1-63 0为关闭 如果自动曝光开启  EXP_TIME命令设置的数据将会变为最大曝光时间，也就是自动曝光时间的上限
                              //一般情况是不需要开启这个功能，因为比赛场地光线一般都比较均匀，如果遇到光线非常不均匀的情况可以尝试设置该值，增加图像稳定性
        {EXP_TIME, 90},      //曝光时间          摄像头收到后会自动计算出最大曝光时间，如果设置过大则设置为计算出来的最大曝光值
        {FPS, 50},            //图像帧率          摄像头收到后会自动计算出最大FPS，如果过大则设置为计算出来的最大FPS
        {SET_COL, MT9V03X_W}, //图像列数量        范围1-752     K60采集不允许超过188
        {SET_ROW, MT9V03X_H}, //图像行数量        范围1-480
        {LR_OFFSET, 0},       //图像左右偏移量    正值 右偏移   负值 左偏移  列为188 376 752时无法设置偏移    摄像头收偏移数据后会自动计算最大偏移，如果超出则设置计算出来的最大偏移
        {UD_OFFSET, 0},       //图像上下偏移量    正值 上偏移   负值 下偏移  行为120 240 480时无法设置偏移    摄像头收偏移数据后会自动计算最大偏移，如果超出则设置计算出来的最大偏移
        {GAIN, 32},           //图像增益          范围16-64     增益可以在曝光时间固定的情况下改变图像亮暗程度
        {INIT, 0}             //摄像头开始初始化
};

/**
 * @brief 设置读取图像完成标志位
 * @return NONE
 */
static void tjrc_mt9v03x_setFinshFlag(void)
{
    MT9V03X_finishFlag = 1;
}

/**
 * @brief 清除读取图像完成标志位
 * @return NONE
 */
uint8_t tjrc_mt9v032_getFinishFlag(void)
{
    return MT9V03X_finishFlag;
}

/**
 * @brief 清除读取图像完成标志位
 * @return NONE
 */
void tjrc_mt9v03x_clearFinishFlag(void)
{
    MT9V03X_finishFlag = 0;
}

/**
 * @brief 摄像头串口回调函数，用来解析串口给出的参数
 * @return NONE
 */
void tjrc_mt9v03x_uartCallBack(void)
{
    static uint32_t recNum = 0;
    uint8_t timeOut = 0;
    while (tjrc_asclin3_receive(&recBuff[recNum]) > 0 && timeOut++ < 2)
    {
        recNum++;
        if (1 == recNum && 0XA5 != recBuff[0])
            recNum = 0;
        if (3 == recNum)
        {
            recNum = 0;
            recFlag = 1;
        }
    }
}

/**
 * @brief 摄像头初始化函数，最后初始化摄像头
 * @return NONE
 */
uint8_t tjrc_setMt9v03x(void)
{
    uint8_t retval = 0;
    /* 给出灰度图像的内存存放位置指针 */
    cameraBufferAddr = MT9V03X_image[0];
    tjrc_setAsclin3_uart();
    /* 用串口读取摄像头的参数 */
    if(tjrc_mt9v03x_getConf(MT9V03X_confGet))
    {
        rt_kprintf("[camera*]failed to get conf\r\n");
        retval = 1;
        return retval;
    }
    recFlag = 0;

    /* 用串口设置摄像头的参数,并重新读取一遍来验证 */
    if(tjrc_mt9v03x_setConf(MT9V03X_CFG))
    {
        rt_kprintf("[camera*]failed to set conf\r\n");
        retval = 2;
        return retval;
    }
    /* 重新读取摄像头参数 */
    if(tjrc_mt9v03x_getConf(MT9V03X_confGet))
    {
        rt_kprintf("[camera*]failed to reget conf\r\n");
        retval = 3;
        return retval;
    }
    rt_kprintf("[camera]set camera configuration\r\n");

    /* 初始化引脚配置为上拉输入 */
    for (uint8_t i = 0; i < 8; i++)
        IfxPort_setPinModeInput(&MODULE_P00, i, IfxPort_InputMode_pullUp);

    /* 最最重要的，配置由外部中断触发的DMA  */
    MT9V03X_linkListNum = tjrc_initEmuDma(
        MT9V03X_DMA_CH,
        (uint8_t *)(&IfxPort_getAddress(IfxPort_Index_00)->IN),
        cameraBufferAddr,
        &IfxScu_REQ2_P10_2_IN,
        MT9V03X_W * MT9V03X_H);

    /* 配置场中断信号输入 */
    tjrc_initEru(&IfxScu_REQ3_P10_3_IN);
    rt_kprintf("[camera]set dma and eru\r\n");
    return retval;
}

/**
 * @brief 摄像头场信号中断回调函数，用于开启DMA
 * @return NONE
 */
void tjrc_mt9v03x_vSync(void)
{
    MT9V03X_dmaCnt = 0;
    if (!MT9V03X_finishFlag) //查看图像数组是否使用完毕，如果未使用完毕则不开始采集，避免出现访问冲突
    {
        if (1 == MT9V03X_linkListNum)
        {
            IfxDma_setChannelDestinationAddress(&MODULE_DMA, MT9V03X_DMA_CH,
                                                (void *)IFXCPU_GLB_ADDR_DSPR(IfxCpu_getCoreId(), cameraBufferAddr));
        }
        IfxDma_enableChannelTransaction(&MODULE_DMA, MT9V03X_DMA_CH);
    }
}

/**
 * @brief DMA完成中断回调函数，采集完成关闭中断
 * @return NONE
 */
void tjrc_mt9v03x_dmaCallBack(void)
{
    IfxDma_clearChannelInterrupt(&MODULE_DMA, MT9V03X_DMA_CH);
    MT9V03X_dmaCnt++;

    /* 完成采集一帧图像 */
    if (MT9V03X_dmaCnt >= MT9V03X_linkListNum)
    {
        MT9V03X_dmaCnt = 0;
        tjrc_mt9v03x_setFinshFlag(); //一副图像从采集开始到采集结束耗时3.8MS左右(50FPS、188*120分辨率)
        IfxDma_disableChannelTransaction(&MODULE_DMA, MT9V03X_DMA_CH);
    }
}

/**
 * @brief 通过串口设置摄像头参数
 * @param buff[CONFIG_FINISH - 1][2] (sint16)配置参数存放数组
 * @return NONE
 */
uint8_t tjrc_mt9v03x_setConf(sint16 buff[CONFIG_FINISH - 1][2])
{
    uint16_t temp, i;
    uint8_t send_buffer[4], retval = 0;

    recFlag = 0;

    for (i = 0; i < CONFIG_FINISH; i++)
    {
        send_buffer[0] = 0xA5;
        send_buffer[1] = (uint8_t)buff[i][0];
        temp = buff[i][1];
        send_buffer[2] = temp >> 8;
        send_buffer[3] = (uint8_t)temp;

        tjrc_asclin3_transmit(send_buffer, 4);
        rt_thread_mdelay(2);
    }

    uint32_t timeout = rt_tick_get();
    while (!recFlag)
    {
        if (rt_tick_get() - timeout > 1000)
        {
            retval = 1;
            break;
        }
    }
    while ((0xff != recBuff[1]) || (0xff != recBuff[2]))
    {
        if (rt_tick_get() - timeout > 2000)
        {
            retval = 1;
            break;
        }
    }
    return retval;
}

/**
 * @brief 通过串口读取摄像头参数
 * @param buff[CONFIG_FINISH - 1][2] (sint16)配置参数存放数组
 * @return NONE
 */
uint8_t tjrc_mt9v03x_getConf(sint16 buff[CONFIG_FINISH - 1][2])
{
    uint16_t temp, i;
    uint8_t send_buffer[4],retval = 0;

    for (i = 0; i < CONFIG_FINISH - 1; i++)
    {
        send_buffer[0] = 0xA5;
        send_buffer[1] = GET_STATUS;
        temp = buff[i][0];
        send_buffer[2] = temp >> 8;
        send_buffer[3] = (uint8_t)temp;
        tjrc_asclin3_transmit(send_buffer, 4);
        /* 等待接受回传数据 */
        uint32_t timeout = rt_tick_get();
        while (!recFlag)
        {
            if (rt_tick_get() - timeout > 1000)
            {
                break;
                retval = 1;
            }
        }
        recFlag = 0;
        buff[i][1] = recBuff[1] << 8 | recBuff[2];
    }
    return retval;
}

/**
 * @brief 通过串口获取版本号
 * @return NONE
 */
uint16_t tjrc_mt9v03x_getVision(void)
{
    uint16_t temp;
    uint8_t send_buffer[4];
    send_buffer[0] = 0xA5;
    send_buffer[1] = GET_STATUS;
    temp = GET_VERSION;
    send_buffer[2] = temp >> 8;
    send_buffer[3] = (uint8_t)temp;

    tjrc_asclin3_transmit(send_buffer, 4);

    uint32_t timeout = rt_tick_get();
    while (!recFlag)
    {
        if (rt_tick_get() - timeout > 100)
        {
            return 0;
        }
    }
    recFlag = 0;

    return ((uint16_t)(recBuff[1] << 8) | recBuff[2]);
}




void Ifx_MT9V03X_displayImage(uint8_t* image, uint8_t threshold)
{
    static uint8_t frameBuff[720];
    tjrc_st7735_setBusy(1);
    /* 简单缩放图像 */
    for (uint32_t i = 0; i < 60; i++)
    {
        for (uint32_t j = 0; j < 11; j++)
        {
            uint8_t temp = 0x00;
            for (uint8_t k = 0; k < 8; k++)
            {
                temp <<= 1;
                if (image[(i * 2)*MT9V03X_W+j * 16 + k*2] > threshold)
                    temp += 1;
                else
                    temp += 0;
            }
            frameBuff[i * 12 + j] = temp;
        }
    }
    tjrc_st7735_dispImage(frameBuff, 96, 60, 0, 24, RGB565_WHITE);
    tjrc_st7735_setBusy(0);
}
