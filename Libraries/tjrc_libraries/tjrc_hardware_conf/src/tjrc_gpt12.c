/**
 * @file tjrc_gpt12.c
 * @author YYYDS team (1951578@tongji.edu.cn)
 * @brief 
 * @version 0.1
 * @date 2022-03-13
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "tjrc_gpt12.h"

/**
 * @brief 把GPT12的TIM2模块配置为外部输入脉冲的计数器
 * @return NONE
 */
void tjrc_setGpt12_encoder(void)
{
    rt_kprintf("[gpt12]T2(ENC0), T6(ENC1) configured as complementary capture mode\r\n");
    /* 步骤1：初始化GPT12模组 */
    IfxGpt12_enableModule(&MODULE_GPT120);
    IfxGpt12_setGpt1BlockPrescaler(&MODULE_GPT120, IfxGpt12_Gpt1BlockPrescaler_4);
    IfxGpt12_setGpt2BlockPrescaler(&MODULE_GPT120, IfxGpt12_Gpt2BlockPrescaler_4);

    /* 配置第一组（T2）输入捕获 */
    /* 步骤2：配置初始化引脚 */
    IfxGpt12_initTxInPinWithPadLevel(&IfxGpt120_T2INB_P33_7_IN,
                                     IfxPort_InputMode_pullUp, IfxPort_PadDriver_cmosAutomotiveSpeed1);
    IfxGpt12_initTxEudInPinWithPadLevel(&IfxGpt120_T2EUDB_P33_6_IN,
                                        IfxPort_InputMode_pullUp, IfxPort_PadDriver_cmosAutomotiveSpeed1);

    /* 步骤3：配置计数器模式 */
    IfxGpt12_T2_setCounterInputMode(&MODULE_GPT120, IfxGpt12_CounterInputMode_risingEdgeTxIN);
    IfxGpt12_T2_setDirectionSource(&MODULE_GPT120, IfxGpt12_TimerDirectionSource_external);
    IfxGpt12_T2_setMode(&MODULE_GPT120, IfxGpt12_Mode_counter);

    /* 配置第二组（T6）输入捕获 */
    /* 步骤2：配置初始化引脚 */
    IfxGpt12_initTxInPinWithPadLevel(&IfxGpt120_T6INA_P20_3_IN,
                                     IfxPort_InputMode_pullUp, IfxPort_PadDriver_cmosAutomotiveSpeed1);
    IfxGpt12_initTxEudInPinWithPadLevel(&IfxGpt120_T6EUDA_P20_0_IN,
                                        IfxPort_InputMode_pullUp, IfxPort_PadDriver_cmosAutomotiveSpeed1);
    /* 步骤3：配置计数器模式 */
    IfxGpt12_T6_setCounterInputMode(&MODULE_GPT120, IfxGpt12_CounterInputMode_risingEdgeTxIN);
    IfxGpt12_T6_setDirectionSource(&MODULE_GPT120, IfxGpt12_TimerDirectionSource_external);
    IfxGpt12_T6_setMode(&MODULE_GPT120, IfxGpt12_Mode_counter);

    /* 步骤4：打开计数器 */
    IfxGpt12_T2_run(&MODULE_GPT120, IfxGpt12_TimerRun_start);
    IfxGpt12_T6_run(&MODULE_GPT120, IfxGpt12_TimerRun_start);
}

/**
 * @brief 读取GPT12的TIM2模块当前计数值并且清空计数
 * @return uint16_t 当前计数值
 */
int16_t tjrc_gpt12_getT2(void)
{
    int16_t enc0 = (int16_t)IfxGpt12_T2_getTimerValue(&MODULE_GPT120);
    IfxGpt12_T2_setTimerValue(&MODULE_GPT120, 0);
    return enc0;
}

/**
 * @brief 读取GPT12的TIM6模块当前计数值并且清空计数
 * @return uint16_t 当前计数值
 */
int16_t tjrc_gpt12_getT6(void)
{
    int16_t enc1 = (int16_t)IfxGpt12_T6_getTimerValue(&MODULE_GPT120);
    IfxGpt12_T6_setTimerValue(&MODULE_GPT120, 0);
    return enc1;
}
