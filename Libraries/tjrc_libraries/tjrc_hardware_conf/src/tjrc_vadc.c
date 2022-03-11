/*
 * tjrc_vadc.c
 *
 *  Created on: 2022年2月23日
 *      Author: 11657
 */

#include "tjrc_vadc.h"

/**
 * @brief 初始化一个模拟量输入引脚（Axx）
 * @param apinIndex (uint8_t)引脚的代号，与芯片核心板印刷的对应
 * @return NONE
 */
void tjrc_setVadc_pin(uint8_t apinIndex)
{
    rt_kprintf("[vadc]initialize pin A%d as analog input\r\n", apinIndex);
    /* 根据代号换算出引脚属于第几组的第几通道 */
    const uint32_t ADC_SAMPLE_FREQUENCY = 10000000;
    IfxVadc_GroupId VADC_GROUP = (IfxVadc_GroupId)apinIndex / 16;
    IfxVadc_ChannelId CHANNEL_ID = (IfxVadc_ChannelId)apinIndex % 16;

    /* 步骤1：配置，开启Adc */
    static IfxVadc_Adc_Config adcConfig;
    IfxVadc_Adc_initModuleConfig(&adcConfig, &MODULE_VADC);
    static IfxVadc_Adc AdcHandler;
    IfxVadc_Adc_initModule(&AdcHandler, &adcConfig);

    /* 步骤2：配置，adc的group */
    static IfxVadc_Adc_GroupConfig adcGroupConfig;
    IfxVadc_Adc_initGroupConfig(&adcGroupConfig, &AdcHandler);

    adcGroupConfig.groupId = VADC_GROUP;
    adcGroupConfig.master = VADC_GROUP;
    /* 配置一些必要的参数，使得adc一直工作 */
    adcGroupConfig.arbiter.requestSlotBackgroundScanEnabled = TRUE;
    adcGroupConfig.backgroundScanRequest.autoBackgroundScanEnabled = TRUE;
    adcGroupConfig.backgroundScanRequest.triggerConfig.gatingMode = IfxVadc_GatingMode_always;
    adcGroupConfig.inputClass[0].resolution = IfxVadc_ChannelResolution_12bit;
    adcGroupConfig.inputClass[0].sampleTime = 1.0f / ADC_SAMPLE_FREQUENCY;
    adcGroupConfig.inputClass[1].resolution = IfxVadc_ChannelResolution_12bit;
    adcGroupConfig.inputClass[1].sampleTime = 1.0f / ADC_SAMPLE_FREQUENCY;

    static IfxVadc_Adc_Group adcGroupHandler;
    IfxVadc_Adc_initGroup(&adcGroupHandler, &adcGroupConfig);

    /* 步骤3：配置adc的通道 */
    static IfxVadc_Adc_ChannelConfig adcChannelConfigHandler;
    IfxVadc_Adc_initChannelConfig(&adcChannelConfigHandler, &adcGroupHandler);

    adcChannelConfigHandler.channelId = (IfxVadc_ChannelId)CHANNEL_ID;
    adcChannelConfigHandler.resultRegister = (IfxVadc_ChannelResult)(CHANNEL_ID % 16);
    adcChannelConfigHandler.backgroundChannel = TRUE;

    static IfxVadc_Adc_Channel adcChannelHandler;
    IfxVadc_Adc_initChannel(&adcChannelHandler, &adcChannelConfigHandler);

    /* 步骤4：开启adc */
    IfxVadc_Adc_setBackgroundScan(&AdcHandler, &adcGroupHandler, (1 << (IfxVadc_ChannelId)CHANNEL_ID), (1 << (IfxVadc_ChannelId)CHANNEL_ID));
    IfxVadc_Adc_startBackgroundScan(&AdcHandler);
}

/**
 * @brief 读取一个模拟量引脚（Axx）adc采样值（电压）
 * @param apinIndex (uint8_t)引脚的代号，与芯片核心板印刷的对应
 * @return NONE
 */
uint16_t tjrc_vadc_getAnalog(uint8_t apinIndex)
{
    Ifx_VADC_RES conversionResult;
    IfxVadc_GroupId VADC_GROUP = (IfxVadc_GroupId)apinIndex / 16;
    IfxVadc_ChannelId CHANNEL_ID = (IfxVadc_ChannelId)apinIndex % 16;
    do
    {
        conversionResult = IfxVadc_getResult(&MODULE_VADC.G[VADC_GROUP],
                                             (IfxVadc_ChannelResult)(CHANNEL_ID));
    } while (!conversionResult.B.VF);
    return conversionResult.B.RESULT;
}
