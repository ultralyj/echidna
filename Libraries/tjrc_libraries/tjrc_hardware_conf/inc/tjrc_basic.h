/*
 * tjrc_basic.h
 *
 *  Created on: 2022年2月25日
 *      Author: 11657
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

#define BEEP_PIN &MODULE_P11, 10

typedef enum
{
    KEY0 = 0u,
    KEY1 = 1u,
    KEY2 = 2u,
    KEY3 = 3u,
} keyChannel;

typedef enum
{
    KEY_Released = 0u,
    KEY_Pressed = 1u,
    KEY_Longpressed = 2u,
} keyValue;


/* 按键开关相关函数 */
keyValue tjrc_checkKey(keyChannel channel);
void tjrc_setLed_pin(void);
void tjrc_setKeys_pin(void);
void tjrc_setBeep_pin(void);

#endif


