/**
 * @file tjrc_gtm.c
 * @author YYYDS team (1951578@tongji.edu.cn)
 * @brief 
 * @version 0.1
 * @date 2022-03-13
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "tjrc_gtm.h"

/**
 * @brief 初始化GTM的TOM模块产生PWM波
 * @param GtmToutMap   （IfxGtm_Tom_ToutMap*）指向TOM表的指针，like IfxGtm_TOM0_7_TOUT64_P20_8_OUT。
 *                      对着他按F3就可以访问到支持TOM产生PWM波的引脚了
 * @param PWM_Period   （uint32_t）PWM波的周期，一般50000,100000即可
 * @return NONE
 */
void tjrc_setGtmAtom_pwm(IfxGtm_Atom_ToutMap *GtmToutMap, uint32_t PWM_Freq)
{
#define CMU_CLK_FREQ 20000000.0f
    rt_kprintf("[gtm]set gtm(atom) as pwm output(default: P21.2)\r\n");
    /* 步骤1：使能GTM，并开启时钟 */
    IfxGtm_enable(&MODULE_GTM);

    IfxGtm_Cmu_setClkFrequency(&MODULE_GTM, IfxGtm_Cmu_Clk_0, CMU_CLK_FREQ);
    IfxGtm_Cmu_enableClocks(&MODULE_GTM, IFXGTM_CMU_CLKEN_CLK0);

    /* 步骤2：定义配置结构体并拉取默认配置 */
    static IfxGtm_Atom_Pwm_Config g_atomConfig;
    IfxGtm_Atom_Pwm_initConfig(&g_atomConfig, &MODULE_GTM);

    /* 步骤3：从ATOM表里拉取一些参数填充到配置结构体中 */
    g_atomConfig.atom = GtmToutMap->atom;
    g_atomConfig.atomChannel = GtmToutMap->channel;
    g_atomConfig.period = CMU_CLK_FREQ / PWM_Freq;
    g_atomConfig.pin.outputPin = GtmToutMap;
    g_atomConfig.synchronousUpdateEnabled = TRUE;

    /* 步骤4：定义PWM句柄，输入配置参数，最后开启PWM */
    IfxGtm_Atom_Pwm_Driver g_atomDriver;
    IfxGtm_Atom_Pwm_init(&g_atomDriver, &g_atomConfig);
    IfxGtm_Atom_Pwm_start(&g_atomDriver, TRUE);
}

/**
 * @brief 调整占空比
 * @param GtmToutMap    （IfxGtm_Tom_ToutMap*）指向TOM表的指针，like IfxGtm_TOM0_7_TOUT64_P20_8_OUT。
 *                       对着他按F3就可以访问到支持TOM产生PWM波的引脚了
 * @param dutyCycle     （uint32_t）占空比，范围0-10000
 * @return NONE
 */
void tjrc_gtmAtom_setDutyCycle(IfxGtm_Atom_ToutMap *GtmToutMap, uint32_t dutyCycle)
{
    uint32_t period = IfxGtm_Atom_Ch_getCompareZero(&MODULE_GTM.ATOM[GtmToutMap->atom], GtmToutMap->channel);
    uint32_t duty = dutyCycle * period / 10000;
    IfxGtm_Atom_Ch_setCompareOneShadow(&MODULE_GTM.ATOM[GtmToutMap->atom], GtmToutMap->channel, duty);
}

