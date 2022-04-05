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

/**
 * @brief 拨码开关配置字配置
 *
 */
uint8_t switch_value = 0;

TJRC_CONFINO tjrc_conf_inf;
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
    tjrc_setBeep_pin();
    switch_value = tjrc_switch_scan();
    //tjrc_setCcu61_pwm();
    /* 配置gpt12，获取编码器数值 */
    tjrc_setGpt12_encoder();
    /* 初始化i2c接口 */
    tjrc_setIic();
    /* asclin1 */
    tjrc_setAsclin1_uart();
    /* 初始化模拟引脚 */
    tjrc_setVadc_pin(ADC_2V5_Pin);
    tjrc_setVadc_pin(ADC_3V3_Pin);
    tjrc_setVadc_pin(ADC_5V0_Pin);
    tjrc_setVadc_pin(ADC_VBAT_Pin);
    /* 初步计算电压 */
    uint16_t VREF_2V5 = tjrc_vadc_getAnalog(ADC_2V5_Pin);
    VCC_BAT = tjrc_vadc_getAnalog(ADC_VBAT_Pin)* 2.5f * 4.0f / VREF_2V5;
    VCC_3V3 = tjrc_vadc_getAnalog(ADC_3V3_Pin)* 2.5f * 2.0f / VREF_2V5;
    VCC_5V0 = tjrc_vadc_getAnalog(ADC_5V0_Pin)* 2.5f * 2.0f / VREF_2V5;
    VREF_2V5 = tjrc_vadc_getAnalog(ADC_2V5_Pin);
    VCC_BAT = tjrc_vadc_getAnalog(ADC_VBAT_Pin)* 2.5f * 4.16f / VREF_2V5;
    printf("[vadc]Monitoring voltage: VCC_3V3:%.3fV, VCC_5V:%.3fV, DC_IN:%.3fV\r\n",VCC_3V3,VCC_5V0,VCC_BAT);
    rt_kprintf("______________________________________________________________________\r\n");
    /* 若电池电压正常，则亮绿灯 */
    if(VCC_BAT>7 &&VCC_BAT<8.4)
    {
        IfxPort_setPinLow(SYSTEM_LED);
        IfxPort_setPinLow(BEEP_PIN);
        rt_thread_mdelay(50);
        IfxPort_setPinHigh(BEEP_PIN);
    }
    else
    {
        IfxPort_setPinLow(BEEP_PIN);
        rt_thread_mdelay(300);
        IfxPort_setPinHigh(BEEP_PIN);
    }
}

/**
 * @brief 初始化所有外设模块
 * 
 */
void tjrc_setPeripherals(void)
{
    /* 初始化电机 */
    tjrc_setMotors();
    /* 配置gtm(atom)产生舵机控制信号 */
    tjrc_setGtmAtom_pwm(SERVO_GTM_ATOM_CHANNEL, 50);
    tjrc_servo_setAngle(0);

#if !IMU_BANNED
    /* 初始化IMU:ICM20602 */
    tjrc_setIcm20602();
#endif
    /* 初始化屏幕 */
    tjrc_setSt7735();
    tjrc_setUi();

#if !CAMERA_BANNED
    /* 初始化摄像头 */
    if(!tjrc_setMt9v03x())
    {
        /* 若初始化摄像头成功，则亮起红色LED */
        IfxPort_setPinLow(CAMERA_LED);
    }
#endif

    /* 检测TF卡，若成功则亮起白灯，且初始化文件系统，读取配置文件 */
    if(!tjrc_setSdmmc())
    {
        sdmmc_detected_flag = 1;
        IfxPort_setPinLow(SDMMC_LED);
        tjrc_setFat32();
    }
    rt_kprintf("______________________________________________________________________\r\n");
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
    char str[20];
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
        tjrc_fileIo_initConfFile(&fil_conf);
        ff_res = f_sync(&fil_conf);
        rt_kprintf("[ff]sync file: tjrc_conf.txt(%d)\r\n",ff_res);
    }
    else
    {
        /* 打开配置文件 */
        ff_res=f_open(&fil_conf,"tjrc/tjrc_conf.txt",FA_OPEN_EXISTING|FA_WRITE|FA_READ);
        rt_kprintf("[ff]open file: tjrc_conf.txt(%d)\r\n",ff_res);
        /* 读取配置文件，并存储于tjrc_conf_inf */
        tjrc_fileIo_getConfFile(&fil_conf,&tjrc_conf_inf);
        /* 增加开机次数1次 */
        tjrc_conf_inf.boot_cnt +=1;
        /* 更新配置文件 */
        tjrc_fileIo_updateConfFile(&fil_conf,&tjrc_conf_inf);
        rt_kprintf("[tjrc]version:%d, boot_cnt:%d\r\n",tjrc_conf_inf.version,tjrc_conf_inf.boot_cnt);
        ff_res = f_close(&fil_conf);
        /* 创建新文件夹用于存储照片 */
        sprintf(str,"tjrc/b%03d",tjrc_conf_inf.boot_cnt);
        ff_res=f_mkdir(str);
    }

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
    /* 显示电源电压(0,104) */
    sprintf(str,"BAT:%.02fV",VCC_BAT);
    if(VCC_BAT>7.0f && VCC_BAT<8.4f)
        tjrc_st7735_dispStr612(2,104,(uint8_t*)str,RGB565_GREEN);
    else
        tjrc_st7735_dispStr612(2,104,(uint8_t*)str,RGB565_RED);

    /* 四个拨码开关 (60，104) */
    if(switch_value & SWITCH0)
        tjrc_st7735_drawBox(61,107,4,6,RGB565_YELLOW);
    else
        tjrc_st7735_drawBox(61,107,4,6,RGB565_GRAY);
    if(switch_value & SWITCH1)
        tjrc_st7735_drawBox(67,107,4,6,RGB565_YELLOW);
    else
        tjrc_st7735_drawBox(67,107,4,6,RGB565_GRAY);
    if(switch_value & SWITCH2)
        tjrc_st7735_drawBox(73,107,4,6,RGB565_YELLOW);
    else
        tjrc_st7735_drawBox(73,107,4,6,RGB565_GRAY);
    if(switch_value & SWITCH3)
        tjrc_st7735_drawBox(79,107,4,6,RGB565_YELLOW);
    else
        tjrc_st7735_drawBox(79,107,4,6,RGB565_GRAY);

    /* IMU和相机状态显示(84,104) */
    extern const uint8_t image_camera_16x16[];
    extern const uint8_t image_imu_16x16[];
#if CAMERA_BANNED
    tjrc_st7735_dispImage((uint8_t*)image_camera_16x16,16,16,88,106,RGB565_GRAY);
#else
    tjrc_st7735_dispImage((uint8_t*)image_camera_16x16,16,16,88,106,RGB565_GREEN);
#endif

#if IMU_BANNED
    tjrc_st7735_dispImage((uint8_t*)image_imu_16x16,16,16,102,104,RGB565_GRAY);
#else
    tjrc_st7735_dispImage((uint8_t*)image_imu_16x16,16,16,102,104,RGB565_GREEN);
#endif

    /* 平衡相关参数显示 */
    sprintf(str,"A0:%.03f AK:%.03f",0.0f,0.0f);
    tjrc_st7735_dispStr612(0,116,(uint8_t*)str,RGB565_BLUE);
    /*  */
}
