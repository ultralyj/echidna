/**
 * @file common.c
 * @author YYYDS team (1951578@tongji.edu.cn)
 * @brief 获取时钟频率信息，重定义fputc等
 * @version 0.1
 * @date 2022-03-27
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "IfxScuEru.h"
#include "SysSe/Bsp/Bsp.h"
#include "TC264_config.h"
#include "common.h"

#include "tjrc_asclin.h"

App_Cpu0 g_AppCpu0; 

/**
 * @brief 获取时钟频率
 * 
 */
void get_clk(void)
{
	disableInterrupts();
	
    IfxScuWdt_disableCpuWatchdog(IfxScuWdt_getCpuWatchdogPassword());
    IfxScuWdt_disableSafetyWatchdog(IfxScuWdt_getSafetyWatchdogPassword());

    g_AppCpu0.info.pllFreq = IfxScuCcu_getPllFrequency();
    g_AppCpu0.info.cpuFreq = IfxScuCcu_getCpuFrequency(IfxCpu_getCoreIndex());
    g_AppCpu0.info.sysFreq = IfxScuCcu_getSpbFrequency();
    g_AppCpu0.info.stmFreq = IfxStm_getFrequency(&MODULE_STM0);
#if(PRINTF_ENABLE)
    tjrc_setAsclin0_uart();
#endif
}

/**
 * @brief 重定义fputc函数，实现串口0输出printf的信息
 * 
 * @param ch 字符数据
 * @param stream 
 * @return int 
 */
int fputc(int ch, FILE *stream)
{
    tjrc_asclin0_transmit((uint8_t*)&ch,1);
    return(ch);
}
