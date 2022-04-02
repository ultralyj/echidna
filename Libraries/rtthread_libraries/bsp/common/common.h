
#ifndef _common_h
#define _common_h

#include "PLATFORM_TYPES.H"
#include "IfxCpu.h"
#include "stdio.h"

typedef struct
{
    float sysFreq;                /**< \brief Actual SPB frequency */
    float cpuFreq;                /**< \brief Actual CPU frequency */
    float pllFreq;                /**< \brief Actual PLL frequency */
    float stmFreq;                /**< \brief Actual STM frequency */
} AppInfo;

/** \brief Application information */
typedef struct
{
    AppInfo info;                               /**< \brief Info object */
} App_Cpu0;

extern IfxCpu_syncEvent g_cpuSyncEvent;

void get_clk(void);

#endif
