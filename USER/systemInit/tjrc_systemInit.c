/**
 * @file tjrc_systemInit.c
 * @author YYYDS team (1951578@tongji.edu.cn)
 * @brief 系统初始化函数
 * @version 0.1
 * @date 2022-03-13
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "tjrc_systemInit.h"

static void tjrc_setUi(void);

/**
 * @brief 各种电压检测变量
 * 
 */
float VCC_3V3,VCC_5V0,VCC_BAT;

uint8_t sdmmc_detected_flag = 0;

/**
 * @brief 一次性初始化所有片上外设（均带有调试信息输出）
 *
 */
void tjrc_setHardware(void)
{
    /* 配置LED,按键和蜂鸣器的GPIO */
    tjrc_setLed_pin();
    tjrc_setKeys_pin();
    //tjrc_setBeep_pin();
    /* 配置CCU6，输出互补PWM */
    tjrc_setCcu60_pwm();
    //tjrc_setCcu61_pwm();
    /* 配置gpt12，获取编码器数值 */
    tjrc_setGpt12_encoder();
    /* 配置gtm(atom)产生舵机控制信号 */
    tjrc_setGtmAtom_pwm(SERVO_GTM_ATOM_CHANNEL, 50);
    /* 初始化i2c接口 */
    tjrc_setIic();
    /* 初始化模拟引脚 */
    tjrc_setVadc_pin(ADC_3V3_Pin);
    tjrc_setVadc_pin(ADC_5V0_Pin);
    tjrc_setVadc_pin(ADC_VBAT_Pin);
    /* 初步计算电压 */
    VCC_3V3 = tjrc_vadc_getAnalog(ADC_3V3_Pin)* 3.33f * 2.0f / 4096;
    VCC_5V0 = tjrc_vadc_getAnalog(ADC_5V0_Pin)* 3.33f * 2.0f / 4096;
    VCC_BAT = tjrc_vadc_getAnalog(ADC_VBAT_Pin)* 3.33f * 4.0f / 4096;
    printf("[vadc]Monitoring voltage: VCC_3V3:%.3fV, VCC_5V:%.3fV, DC_IN:%.3fV\r\n",VCC_3V3,VCC_5V0,VCC_BAT);
    /* 若电池电压正常，则亮绿灯 */
    if(VCC_BAT>7 &&VCC_BAT<8.4)
    {
        IfxPort_setPinLow(SYSTEM_LED);
    }
}

/**
 * @brief 初始化所有外设模块
 * 
 */
void tjrc_setPeripherals(void)
{
    /* 初始化电机 */
    tjrc_setFlyWheelMotor();
    /* 初始化IMU:ICM20602 */
    tjrc_setIcm20602();
    /* 初始化屏幕 */
    tjrc_setSt7735();
    tjrc_setUi();

    /* 初始化摄像头 */
    if(!tjrc_setMt9v03x())
    {
        /* 若初始化摄像头成功，则亮起红色LED */
        IfxPort_setPinLow(CAMERA_LED);
    }
    /* 检测TF卡，若成功则亮起白灯，且初始化文件系统，读取配置文件 */
    if(!tjrc_setSdmmc())
    {
        sdmmc_detected_flag = 1;
        IfxPort_setPinLow(SDMMC_LED);
        tjrc_setFat32();
    }

}

/**
 * @brief 初始化文件系统
 * 
 */
FATFS _fs,*fs=&_fs;
FILINFO fno;
void tjrc_setFat32(void)
{
    DIR dir;
    FIL fil_conf;
    FRESULT ff_res;
    /* 强制挂载SDMMC */
    ff_res = f_mount(fs, "", 1);
    rt_kprintf("[ff]mount sdmmc(%d)\r\n",ff_res);

    /* 获取剩余可用容量 */
    uint32_t fre_clust=0;
    ff_res = f_getfree("", &fre_clust, &fs);
    /* 计算总扇区数和可用扇区数 */
    uint32_t tot_sect = (fs->n_fatent - 2) * fs->csize;
    uint32_t fre_sect = fre_clust * fs->csize;
    rt_kprintf("[ff]cap:%dMB, available:%dMB\r\n", tot_sect / 2048, fre_sect / 2048);
    /* 检索根目录下有无文件夹tjrc，没有则新建文件夹 */
    ff_res = f_stat("tjrc", &fno);
    if(ff_res == FR_NO_FILE)
    {
        ff_res=f_mkdir("tjrc");
        rt_kprintf("[ff]directory tjrc does not exist, make directory tjrc(%d)\r\n",ff_res);
    }
    ff_res = f_opendir(&dir,"tjrc");
    rt_kprintf("[ff]open directory tjrc(%d)\r\n",ff_res);
    /* 检索根目录下有无配置文件tjrc_conf.txt，没有则新建文件并初始化 */
    ff_res = f_stat("tjrc/tjrc_conf.txt", &fno);
    if(ff_res == FR_NO_FILE)
    {
        ff_res=f_open(&fil_conf,"tjrc/tjrc_conf.txt",FA_CREATE_NEW|FA_READ|FA_WRITE);
        rt_kprintf("[ff]configure file lost, create new: tjrc_conf.txt(%d)\r\n",ff_res);
        f_printf(&fil_conf,"hello");
        ff_res = f_sync(&fil_conf);
        rt_kprintf("[ff]sync file: tjrc_conf.txt(%d)\r\n",ff_res);
    }
    ff_res=f_open(&fil_conf,"tjrc/tjrc_conf.txt",FA_OPEN_EXISTING|FA_WRITE);
    rt_kprintf("[ff]open file: tjrc_conf.txt(%d)\r\n",ff_res);
    f_printf(&fil_conf,"yoho");
    ff_res = f_close(&fil_conf);
    rt_kprintf("[ff]close file: tjrc_conf.txt(%d)\r\n",ff_res);

}

/**
 * @brief 初始化屏幕界面
 * 
 */
static void tjrc_setUi(void)
{
    char str[20];
    tjrc_st7735_clean(RGB565_BLACK);
    tjrc_st7735_dispStr612(28,0,(uint8_t*)"__ZX*YYYDS__",RGB565_YELLOW);
    tjrc_st7735_drawBox(32,26,64,44,RGB565_GRAY);
    sprintf(str,"BAT:%.02fV",VCC_BAT);
    if(VCC_BAT>7.0f && VCC_BAT<8.4f)
        tjrc_st7735_dispStr612(40,72,(uint8_t*)str,RGB565_GREEN);
    else
        tjrc_st7735_dispStr612(40,72,(uint8_t*)str,RGB565_RED);

}
