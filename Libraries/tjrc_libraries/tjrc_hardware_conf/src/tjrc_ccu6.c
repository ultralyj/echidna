/**
 * @file tjrc_ccu6.c
 * @author YYYDS team (1951578@tongji.edu.cn)
 * @brief 
 * @version 0.1
 * @date 2022-03-13
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "tjrc_ccu6.h"

/**
 * @brief CCU6互补PWM控制句柄
 */
IfxCcu6_PwmHl CCU60_PWMHandler;
IfxCcu6_PwmHl CCU61_PWMHandler;

IfxCcu6_TimerWithTrigger CCU60_TimerHandler;
IfxCcu6_TimerWithTrigger CCU61_TimerHandler;
/**
 * @brief CCU60的初始化函数，配置成三相互补pwm
 * @return NONE
 */
void tjrc_setCcu60_pwm(void)
{
    rt_kprintf("[ccu60]configured as complementary PWM mode\r\n");
    /* 互补PWM的参数，可修改 */
    const float32 CCU6PWMFreq = 20000;  // PWM频率，20K
    const uint8_t CCU6ChannelNum = 3;   // CCU6通道数，默认3对互补输出
    //boolean interruptState = IfxCpu_disableInterrupts();

    /* 步骤1：配置pwm用的定时器，定义结构体拉取默认配置 */

    IfxCcu6_TimerWithTrigger_Config timerConf;
    IfxCcu6_TimerWithTrigger_initConfig(&timerConf, &MODULE_CCU60);

    /* 步骤2：配置定时器的相关参数，包含pwm波的频率  */
    timerConf.base.frequency = CCU6PWMFreq;
    timerConf.base.countDir = IfxStdIf_Timer_CountDir_upAndDown;

    /* 步骤3：初始化定时器 */
    IfxCcu6_TimerWithTrigger_init(&CCU60_TimerHandler, &timerConf);

    /* 步骤4：配置互补PWM，定义结构体，拉取默认配置 */
    IfxCcu6_PwmHl_Config pwmHlConf;
    IfxCcu6_PwmHl_initConfig(&pwmHlConf);

    /* 步骤5：配置pwm的相关参数 */
    pwmHlConf.timer = &CCU60_TimerHandler;
    /* 配置开启多少对互补pwm通道 */
    pwmHlConf.base.channelCount = CCU6ChannelNum;
    /* 设置死区时间（单位为秒） */
    pwmHlConf.base.deadtime = (float32)2.0e-7;

    /* 步骤6：选择输出引脚 */
    pwmHlConf.cc0 = &IfxCcu60_CC60_P02_0_OUT;
    pwmHlConf.cc1 = &IfxCcu60_CC61_P02_2_OUT;
    pwmHlConf.cc2 = &IfxCcu60_CC62_P02_4_OUT;
    pwmHlConf.cout0 = &IfxCcu60_COUT60_P02_1_OUT;
    pwmHlConf.cout1 = &IfxCcu60_COUT61_P02_3_OUT;
    pwmHlConf.cout2 = &IfxCcu60_COUT62_P02_5_OUT;

    /* 步骤7：输入参数，获得pwm句柄 */
    IfxCcu6_PwmHl_init(&CCU60_PWMHandler, &pwmHlConf);

    /* 步骤8：设置互补pwm的对齐方式（1对的话就忽略好了） */
    IfxCcu6_PwmHl_setMode(&CCU60_PWMHandler, Ifx_Pwm_Mode_centerAligned);

    IfxCcu6_TimerWithTrigger_run(CCU60_PWMHandler.timer);

    uint32_t rduty[3] = {50,50,50};
    tjrc_ccu60pwm_setDutyCycle(rduty,3);
    //IfxCpu_restoreInterrupts(interruptState);
}


/**
 * @brief CCU61的初始化函数，配置成三相互补pwm
 * @return NONE
 */
void tjrc_setCcu61_pwm(void)
{
    rt_kprintf("[ccu61]configured as complementary PWM mode\r\n");
    /* 互补PWM的参数，可修改 */
    const float32 CCU6PWMFreq = 20000;  // PWM频率，20K
    const uint8_t CCU6ChannelNum = 1;   // CCU6通道数，默认1对互补输出
    //boolean interruptState = IfxCpu_disableInterrupts();

    /* 步骤1：配置pwm用的定时器，定义结构体拉取默认配置 */
    IfxCcu6_TimerWithTrigger_Config timerConf;
    IfxCcu6_TimerWithTrigger_initConfig(&timerConf, &MODULE_CCU61);

    /* 步骤2：配置定时器的相关参数，包含pwm波的频率  */
    timerConf.base.frequency = CCU6PWMFreq;
    timerConf.base.countDir = IfxStdIf_Timer_CountDir_upAndDown;

    /* 步骤3：初始化定时器 */
    IfxCcu6_TimerWithTrigger_init(&CCU61_TimerHandler, &timerConf);

    /* 步骤4：配置互补PWM，定义结构体，拉取默认配置 */
    IfxCcu6_PwmHl_Config pwmHlConf;
    IfxCcu6_PwmHl_initConfig(&pwmHlConf);

    /* 步骤5：配置pwm的相关参数 */
    pwmHlConf.timer = &CCU61_TimerHandler;
    /* 配置开启多少对互补pwm通道 */
    pwmHlConf.base.channelCount = CCU6ChannelNum;
    /* 设置死区时间（单位为秒） */
    pwmHlConf.base.deadtime = (float32)2.0e-7;

    /* 步骤6：选择输出引脚 */
    pwmHlConf.cc0 = &IfxCcu61_CC60_P33_13_OUT;
    //pwmHlConf.cc1 = &IfxCcu60_CC61_P11_11_OUT;
    //pwmHlConf.cc2 = &IfxCcu60_CC62_P11_10_OUT;
    pwmHlConf.cout0 = &IfxCcu61_COUT60_P33_12_OUT;
    //pwmHlConf.cout1 = &IfxCcu60_COUT61_P11_6_OUT;
    //pwmHlConf.cout2 = &IfxCcu60_COUT62_P11_3_OUT;

    /* 步骤7：输入参数，获得pwm句柄 */
    IfxCcu6_PwmHl_init(&CCU61_PWMHandler, &pwmHlConf);

    /* 步骤8：设置互补pwm的对齐方式（1对的话就忽略好了） */
    IfxCcu6_PwmHl_setMode(&CCU61_PWMHandler, Ifx_Pwm_Mode_centerAligned);

    IfxCcu6_TimerWithTrigger_run(CCU61_PWMHandler.timer);
    //IfxCpu_restoreInterrupts(interruptState);
}

/**
 * @brief 设置CCU60的互补pwm的占空比
 * @param dutuCycle (uint32_t*)指向一个数组，数组长度由使用的通道数决定，为百分比
 * @param chanNum   (uint8_t)通道数，1-3
 * @return NONE
 */
void tjrc_ccu60pwm_setDutyCycle(uint32_t *dutyCycle, uint8_t chanNum)
{
    Ifx_TimerValue cmpValues[3] = {0, 0, 0};
    for (uint8_t i = 0; i < 3; i++)
    {
        if(dutyCycle[i]>100)
            return;
    }
    for (uint8_t i = 0; i < 3; i++)
        cmpValues[i] = ((5000 / 100) * (100 - dutyCycle[i]));
    CCU60_PWMHandler.update(&CCU60_PWMHandler, cmpValues);
    IfxCcu6_TimerWithTrigger_applyUpdate(CCU60_PWMHandler.timer);
}

/**
 * @brief 设置CCU61的互补pwm的占空比
 * @param dutuCycle (uint32_t*)指向一个数组，数组长度由使用的通道数决定，为百分比
 * @param chanNum   (uint8_t)通道数，1-3
 * @return NONE
 */
void tjrc_ccu61pwm_setDutyCycle(uint32_t *dutyCycle, uint8_t chanNum)
{
    Ifx_TimerValue cmpValues[3] = {0, 0, 0};
    for (uint8_t i = 0; i < 3; i++)
        cmpValues[i] = ((5000 / 100) * (100 - dutyCycle[i]));
    CCU61_PWMHandler.update(&CCU61_PWMHandler, cmpValues);
    IfxCcu6_TimerWithTrigger_applyUpdate(CCU61_PWMHandler.timer);
}
