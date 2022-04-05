/**
 * @file tjrc_basic.h
 * @author YYYDS team (1951578@tongji.edu.cn)
 * @brief 对于一些基本的GPIO，如BEEP,LED,KEY提供函数支持的头文件
 * @version 0.1
 * @date 2022-03-13
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef __TJRC_BASIC_H__
#define __TJRC_BASIC_H__

#include "Ifx_Types.h"
#include "rtthread.h"
#include "stdint.h"
#include "isr_config.h"
#include "IfxPort_PinMap.h"

/* GPIO定义  */
#define LED0_PIN &MODULE_P11, 2
#define LED1_PIN &MODULE_P11, 3
#define LED2_PIN &MODULE_P11, 6
#define LED3_PIN &MODULE_P11, 9

#define KEY0_PIN &MODULE_P15, 2
#define KEY1_PIN &MODULE_P15, 3
#define KEY2_PIN &MODULE_P15, 4
#define KEY3_PIN &MODULE_P15, 5

#define SWITCH0_PIN &MODULE_P22, 0
#define SWITCH1_PIN &MODULE_P22, 1
#define SWITCH2_PIN &MODULE_P22, 2
#define SWITCH3_PIN &MODULE_P22, 3

#define BEEP_PIN &MODULE_P11, 10

/**
 * @brief 按键通道定义
 * 
 */
typedef enum
{
    KEY0 = 0u,
    KEY1 = 1u,
    KEY2 = 2u,
    KEY3 = 3u,
} keyChannel;

/**
 * @brief 拨码开关通道定义
 *
 */
typedef enum
{
    SWITCH0 = 0x01u,
    SWITCH1 = 0x02u,
    SWITCH2 = 0x04u,
    SWITCH3 = 0x08u,
} switchChannel;
/**
 * @brief 按键状态定义
 * 
 */
typedef enum
{
    KEY_Released = 0u,
    KEY_Pressed = 1u,
    KEY_Longpressed = 2u,
} keyValue;

/* LED相关函数 */
void tjrc_setLed_pin(void);
/* 按键开关相关函数 */
void tjrc_setKeys_pin(void);
keyValue tjrc_keyChannel_check(keyChannel channel);
/* 蜂鸣器相关函数 */
void tjrc_setBeep_pin(void);
/* 拨码开关相关函数 */
uint8_t tjrc_switch_scan(void);

#endif


