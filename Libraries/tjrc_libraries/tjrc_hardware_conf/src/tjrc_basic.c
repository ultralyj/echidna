/*
 * tjrc_basic.c
 *
 *  Created on: 2022年2月25日
 *      Author: 11657
 */

#include "tjrc_basic.h"

/**
 * @brief 初始化核心板上4颗LED（初始状态为熄灭）
 * 
 */
void tjrc_setLed_pin(void)
{
    rt_kprintf("[LED]led(P11.2, P11.3, P11.6, P11.9) are set\r\n");
    IfxPort_setPinModeOutput(LED0_PIN, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);
    IfxPort_setPinModeOutput(LED1_PIN, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);
    IfxPort_setPinModeOutput(LED2_PIN, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);
    IfxPort_setPinModeOutput(LED3_PIN, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);
    IfxPort_setPinHigh(LED0_PIN);
    IfxPort_setPinHigh(LED1_PIN);
    IfxPort_setPinHigh(LED2_PIN);
    IfxPort_setPinHigh(LED3_PIN);
}

/**
 * @brief 初始化按键
 * 
 */
void tjrc_setKeys_pin(void)
{
    rt_kprintf("[KEY]keys(P15.2-5) are set\r\n");
    IfxPort_setPinModeInput(KEY0_PIN, IfxPort_InputMode_pullUp);
    IfxPort_setPinModeInput(KEY1_PIN, IfxPort_InputMode_pullUp);
    IfxPort_setPinModeInput(KEY2_PIN, IfxPort_InputMode_pullUp);
    IfxPort_setPinModeInput(KEY3_PIN, IfxPort_InputMode_pullUp);
}

/**
 * @brief 初始化蜂鸣器（注意有源还是无源）
 * 
 */
void tjrc_setBeep_pin(void)
{
#ifdef ACTIVE_BUZZER
    IfxPort_setPinModeOutput(BEEP_PIN, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);
    IfxPort_setPinLow(BEEP_PIN);
#else
    /* 无源蜂鸣器 */
#endif
}

/**
 * @brief 扫描读取按键
 * @param channel (keyChannel)按键的通道，即选择哪一路按键
 * @return keyValue 按键的动作
 */

keyValue XLW_checkKey(keyChannel channel)
{
    const uint32_t buttonDebounceTime = 20;
    const uint32_t buttonLongPressTime = 500;
    static uint8_t buttonTiming = 0x00;
    static uint32_t buttonTimingStart[4] = {0, 0, 0, 0};
    static uint8_t buttonAction[4] = {0, 0, 0, 0};
    keyValue ans = KEY_Released;
    boolean buttonState = TRUE;
    switch (channel)
    {
    case KEY0:
        buttonState = IfxPort_getPinState(KEY0_PIN);
        break;
    case KEY1:
        buttonState = IfxPort_getPinState(KEY1_PIN);
        break;
    case KEY2:
        buttonState = IfxPort_getPinState(KEY2_PIN);
        break;
    case KEY3:
        buttonState = IfxPort_getPinState(KEY3_PIN);
        break;
    }
    if (buttonState == FALSE)
    {
        if (!((buttonTiming >> channel) & 0x01))
        {
            buttonTiming |= (0x01 << channel);
            buttonTimingStart[channel] = rt_tick_get();
        }
        else
        {
            //stm32f1xx_writeLED(0x00);
            if (rt_tick_get() >= (buttonTimingStart[channel] + buttonDebounceTime))
            {
                buttonAction[channel] = 1;
            }
            if (rt_tick_get() >= (buttonTimingStart[channel] + buttonLongPressTime))
            {
                buttonAction[channel] = 2;
            }
        }
        ans = KEY_Released;
    }
    else
    {
        buttonTiming &= (0xff - (0x01 << channel));
        if (buttonAction[channel] != 0)
        {
            if (buttonAction[channel] == 1)
            {
                ans = KEY_Pressed;
            }
            else if (buttonAction[channel] == 2)
            {
                ans = KEY_Longpressed;
            }
            buttonAction[channel] = 0;
        }
    }
    return ans;
}



