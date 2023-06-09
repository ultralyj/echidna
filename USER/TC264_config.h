/**
 * @file TC264_config.h
 * @author YYYDS team (1951578@tongji.edu.cn)
 * @brief 包含对TC264D芯片的配置选项
 * @version 0.1
 * @date 2022-03-27
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef _TC264_config_h
#define _TC264_config_h

#define AURIX_MCU_FREQUENCY         200*1000*1000//200M主频

#define PRINTF_ENABLE               1

/* 禁用IMU及其相关线程！ */
#define IMU_BANNED 0

/* 禁用摄像头及其相关线程！ */
#define CAMERA_BANNED 0
/* 使用RT-Thread 自带控制台 */
#define USE_FINSH_THREAD 0
/* 使用有源蜂鸣器 */
#define ACTIVE_BUZZER
/* 是否开启全局静音 */
#define BEEP_MUTE 0

#define USE_SOFTWATE_I2C 1
#endif
