/**
 * @file board.c
 * @author YYYDS team (1951578@tongji.edu.cn)
 * @brief 板级支持包（BSP）（Board Support Package）
 * 是构建嵌入式操作系统所需的引导程序(Bootload)、内核(Kernel)、
 * 根文件系统(Rootfs)和工具链(Toolchain) 提供完整的软件资源包
 * @version 0.1
 * @date 2022-03-27
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "board.h"

#pragma section all "cpu0_dsram"

//extern void __bss_end__[];
//extern void _lc_ub_heap[];

#define SYSTICK_PRIO 2
#define SERVICE_REQUEST_PRIO 1
static sint32 osticks = 0;
static IfxStm_CompareConfig g_STM0Conf;

//finsh组件接收串口数据，是通过在串口中断内发送邮件，finsh线程接收邮件进行获取的
rt_mailbox_t uart_mb;

/**
 * @brief 初始化调试用串口
 * 
 */
void rt_hw_usart_init(void)
{
    tjrc_setAsclin0_uart();
}

/**
 * @brief 初始化系统定时器
 * 
 */
void rt_hw_systick_init(void)
{
    osticks = IfxStm_getTicksFromMicroseconds(BSP_DEFAULT_TIMER, 1000000 / RT_TICK_PER_SECOND);

    IfxStm_initCompareConfig(&g_STM0Conf);                   /* Initialize the configuration structure with default values   */

    g_STM0Conf.triggerPriority = SYSTICK_PRIO;               /* Set the priority of the interrupt                            */
    g_STM0Conf.typeOfService = IfxSrc_Tos_cpu0;              /* Set the service provider for the interrupts                  */
    g_STM0Conf.ticks = 0;                                   /* Set the number of ticks after which the timer triggers an
                                                             * interrupt for the first time                                 */
    IfxStm_initCompare(BSP_DEFAULT_TIMER, &g_STM0Conf);      /* Initialize the STM with the user configuration               */
    IfxStm_updateCompare(BSP_DEFAULT_TIMER, g_STM0Conf.comparator, osticks + IfxStm_getLower(BSP_DEFAULT_TIMER));
    IfxStm_setSuspendMode(IfxStm_getAddress((IfxStm_Index)TRICORE_CPU_ID), IfxStm_SuspendMode_hard);
}

/**
 * @brief 系统定时器中断
 * 
 */
IFX_INTERRUPT(system_tick_handler, 0, SYSTICK_PRIO)
{
    /* enter interrupt */
    rt_interrupt_enter();

    IfxStm_clearCompareFlag(BSP_DEFAULT_TIMER, g_STM0Conf.comparator);
    IfxStm_updateCompare(BSP_DEFAULT_TIMER, g_STM0Conf.comparator, osticks + IfxStm_getLower(BSP_DEFAULT_TIMER));

    rt_tick_increase();

    /* leave interrupt */
    rt_interrupt_leave();
}



/**
 * This function will initial Tricore board.
 */
#if defined(RT_USING_USER_MAIN) && defined(RT_USING_HEAP)
#define RT_HEAP_SIZE 8192

static uint32_t rt_heap[RT_HEAP_SIZE];     // heap default size: 32K(8192 * 4)
RT_WEAK void *rt_heap_begin_get(void)
{
    return rt_heap;
}

RT_WEAK void *rt_heap_end_get(void)
{
    return rt_heap + RT_HEAP_SIZE;
}
#endif

/**
 * @brief 系统预初始化
 * 
 */
void rt_hw_board_init()
{
    get_clk();

    rt_hw_systick_init();

        /* USART driver initialization is open by default */
    #ifdef RT_USING_SERIAL
        rt_hw_usart_init();
    #endif

        /* Set the shell console output device */
    #ifdef RT_USING_CONSOLE
        //rt_console_set_device(RT_CONSOLE_DEVICE_NAME);
    #endif

    #ifdef RT_USING_HEAP
        //rt_system_heap_init(buf, (void*)(buf + sizeof(buf)));
        rt_system_heap_init(rt_heap_begin_get(), rt_heap_end_get());
    #endif

        /* Board underlying hardware initialization */
    #ifdef RT_USING_COMPONENTS_INIT
        rt_components_board_init();
    #endif

    IfxSrc_init(&SRC_GPSR_GPSR0_SR0, IfxSrc_Tos_cpu0, SERVICE_REQUEST_PRIO);
    IfxSrc_enable(&SRC_GPSR_GPSR0_SR0);

    uart_mb = rt_mb_create("uart_mb", 10, RT_IPC_FLAG_FIFO);
}

void rt_hw_console_output(const char *str)
{
    tjrc_asclin0_sendStr((uint8_t*)str);
}


char rt_hw_console_getchar(void)
{
    uint32 dat;
    //等待邮件
    rt_mb_recv(uart_mb, &dat, RT_WAITING_FOREVER);
    //uart_getchar(DEBUG_UART, &dat);
    return (char)dat;
}


int core0_main(void)
{
    extern int rtthread_startup(void);
    enableInterrupts();
    rtthread_startup();
    rt_kprintf("Failed to start rt-thread.\n");
    while(1);
}
#pragma section all restore
