/**
 * @file tjrc_asclin.c
 * @author YYYDS team (1951578@tongji.edu.cn)
 * @brief 
 * @version 0.1
 * @date 2022-03-13
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "tjrc_asclin.h"



/**
 * @brief 定义串口（uart）缓冲队列数组的大小
 */
#define ASC_TX_BUFFER_SIZE 64
#define ASC_RX_BUFFER_SIZE 64
#define ASC1_BUFFER_SIZE 16
#define ASC3_BUFFER_SIZE 16

IfxAsclin_Asc asclin0_Handler;
IfxAsclin_Asc asclin1_Handler;
IfxAsclin_Asc asclin3_Handler;

static uint8_t g_uartTxBuffer[ASC_TX_BUFFER_SIZE + sizeof(Ifx_Fifo) + 8];
static uint8_t g_uartRxBuffer[ASC_RX_BUFFER_SIZE + sizeof(Ifx_Fifo) + 8];
static uint8_t uart1TxBuffer[ASC1_BUFFER_SIZE + sizeof(Ifx_Fifo) + 8];
static uint8_t uart1RxBuffer[ASC1_BUFFER_SIZE + sizeof(Ifx_Fifo) + 8];
static uint8_t uart3TxBuffer[ASC3_BUFFER_SIZE + sizeof(Ifx_Fifo) + 8];
static uint8_t uart3RxBuffer[ASC3_BUFFER_SIZE + sizeof(Ifx_Fifo) + 8];

/**
 * @brief 初始化串口0
 * @return NONE
 */
void tjrc_setAsclin0_uart(void)
{
    /* 步骤1：定义一个配置的结构体，并通过初始化拉取默认的配置到ascConf */
    static IfxAsclin_Asc_Config ascConf;
    IfxAsclin_Asc_initModuleConfig(&ascConf, &MODULE_ASCLIN0);

    /* 步骤2：配置一些基本参数，主要是波特率 */
    ascConf.baudrate.baudrate = 115200;
    ascConf.baudrate.oversampling = IfxAsclin_OversamplingFactor_16;
    ascConf.bitTiming.medianFilter = IfxAsclin_SamplesPerBit_three;
    ascConf.bitTiming.samplePointPosition = IfxAsclin_SamplePointPosition_8;

    /* 步骤3：配置中断的优先级和服务类型 */
    ascConf.interrupt.txPriority = ISR_PRIORITY_ASCLIN0_TX;
    ascConf.interrupt.rxPriority = ISR_PRIORITY_ASCLIN0_RX;
    ascConf.interrupt.erPriority = ISR_PRIORITY_ASCLIN0_ER;
    ascConf.interrupt.typeOfService = IfxSrc_Tos_cpu0;

    /* 步骤4：关联输入输出引脚到串口控制器 */
    const IfxAsclin_Asc_Pins pins =
        {
            .cts = NULL_PTR,
            .rts = NULL_PTR,
            .rtsMode = IfxPort_OutputMode_pushPull,
            .rx = &IfxAsclin0_RXA_P14_1_IN,
            .rxMode = IfxPort_InputMode_pullUp,
            .tx = &IfxAsclin0_TX_P14_0_OUT,
            .txMode = IfxPort_OutputMode_pushPull,
            .pinDriver = IfxPort_PadDriver_cmosAutomotiveSpeed1};
    ascConf.pins = &pins;

    /* 步骤5：配置数据缓冲区 */
    ascConf.txBuffer = g_uartTxBuffer;
    ascConf.txBufferSize = ASC_TX_BUFFER_SIZE;
    ascConf.rxBuffer = g_uartRxBuffer;
    ascConf.rxBufferSize = ASC_RX_BUFFER_SIZE;

    /* 步骤6：输入参数 */
    IfxAsclin_Asc_initModule(&asclin0_Handler, &ascConf);
}

/**
 * @brief 初始化串口1
 * @return NONE
 */
void tjrc_setAsclin1_uart(void)
{
    float baudrate = 115200;
    rt_kprintf("[asclin]set asclin1 as uart interface(br:%d tx:P15.0 rx:15.1)\r\n",baudrate);
    /* 步骤1：定义一个配置的结构体，并通过初始化拉取默认的配置到ascConf */
    static IfxAsclin_Asc_Config ascConf;
    IfxAsclin_Asc_initModuleConfig(&ascConf, &MODULE_ASCLIN1);

    /* 步骤2：配置一些基本参数，主要是波特率 */
    ascConf.baudrate.prescaler = 4;
    ascConf.baudrate.baudrate = (float)baudrate;
    ascConf.baudrate.oversampling = IfxAsclin_OversamplingFactor_16;

    /* 步骤3：配置中断的优先级和服务类型 */
    ascConf.interrupt.txPriority = ISR_PRIORITY_ASCLIN1_TX;
    ascConf.interrupt.rxPriority = ISR_PRIORITY_ASCLIN1_RX;
    ascConf.interrupt.erPriority = ISR_PRIORITY_ASCLIN1_ER;
    ascConf.interrupt.typeOfService = IfxSrc_Tos_cpu0;

    /* 步骤4：关联输入输出引脚到串口控制器 */
    const IfxAsclin_Asc_Pins pins =
        {
            .cts = NULL_PTR,
            .rts = NULL_PTR,
            .rtsMode = IfxPort_OutputMode_pushPull,
            .rx = &IfxAsclin1_RXA_P15_1_IN,
            .rxMode = IfxPort_InputMode_pullUp,
            .tx = &IfxAsclin1_TX_P15_0_OUT,
            .txMode = IfxPort_OutputMode_pushPull,
            .pinDriver = IfxPort_PadDriver_cmosAutomotiveSpeed1};
    ascConf.pins = &pins;

    /* 步骤5：配置数据缓冲区 */
    ascConf.txBuffer = uart1TxBuffer;
    ascConf.txBufferSize = ASC1_BUFFER_SIZE;
    ascConf.rxBuffer = uart1RxBuffer;
    ascConf.rxBufferSize = ASC1_BUFFER_SIZE;

    /* 步骤6：输入参数 */
    IfxAsclin_Asc_initModule(&asclin1_Handler, &ascConf);
}

/**
 * @brief 初始化串口3
 * @return NONE
 */
void tjrc_setAsclin3_uart(void)
{
    float baudrate = 9600;
    rt_kprintf("[asclin]set asclin3 as uart interface(br:%d tx:P15.6 rx:15.7)\r\n",baudrate);
    /* 步骤1：定义一个配置的结构体，并通过初始化拉取默认的配置到ascConf */
    static IfxAsclin_Asc_Config ascConf;
    IfxAsclin_Asc_initModuleConfig(&ascConf, &MODULE_ASCLIN3);

    /* 步骤2：配置一些基本参数，主要是波特率 */
    ascConf.baudrate.prescaler = 4;
    ascConf.baudrate.baudrate = (float)baudrate;
    ascConf.baudrate.oversampling = IfxAsclin_OversamplingFactor_8;

    /* 步骤3：配置中断的优先级和服务类型 */
    ascConf.interrupt.txPriority = ISR_PRIORITY_ASCLIN3_TX;
    ascConf.interrupt.rxPriority = ISR_PRIORITY_ASCLIN3_RX;
    ascConf.interrupt.erPriority = ISR_PRIORITY_ASCLIN3_ER;
    ascConf.interrupt.typeOfService = IfxSrc_Tos_cpu0;

    /* 步骤4：关联输入输出引脚到串口控制器 */
    const IfxAsclin_Asc_Pins pins =
        {
            .cts = NULL_PTR,
            .rts = NULL_PTR,
            .rtsMode = IfxPort_OutputMode_pushPull,
            .rx = &IfxAsclin3_RXA_P15_7_IN,
            .rxMode = IfxPort_InputMode_pullUp,
            .tx = &IfxAsclin3_TX_P15_6_OUT,
            .txMode = IfxPort_OutputMode_pushPull,
            .pinDriver = IfxPort_PadDriver_cmosAutomotiveSpeed1};
    ascConf.pins = &pins;

    /* 步骤5：配置数据缓冲区 */
    ascConf.txBuffer = uart3TxBuffer;
    ascConf.txBufferSize = ASC3_BUFFER_SIZE;
    ascConf.rxBuffer = uart3RxBuffer;
    ascConf.rxBufferSize = ASC3_BUFFER_SIZE;

    /* 步骤6：输入参数 */
    IfxAsclin_Asc_initModule(&asclin3_Handler, &ascConf);
}

/**
 * @brief 串口0发送定长数组
 * @param buff (uint8_t*)字符数组指针
 * @param len (uint32_t)数组长度
 * @return NONE
 */
void tjrc_asclin0_transmit(uint8_t *buff, uint32_t len)
{
    while (len)
    {
        Ifx_SizeT count = 1;
        (void)IfxAsclin_Asc_write(&asclin0_Handler, buff, &count, TIME_INFINITE);
        len--;
        buff++;
    }
}

/**
 * @brief 串口0发送定长数组
 * @param buff (uint8_t*)字符数组指针
 * @return NONE
 */
void tjrc_asclin0_sendStr(uint8_t *buff)
{
    while (*buff)
    {
        Ifx_SizeT count = 1;
        (void)IfxAsclin_Asc_write(&asclin0_Handler, buff, &count, TIME_INFINITE);
        buff++;
    }
}

/**
 * @brief 串口1发送字符串
 * @param buff (uint8_t*)字符串指针
 * @return NONE
 */
void tjrc_asclin1_sendStr(uint8_t *buff)
{
    while (*buff)
    {
        Ifx_SizeT count = 1;
        (void)IfxAsclin_Asc_write(&asclin1_Handler, buff, &count, TIME_INFINITE);
        buff++;
    }
}

/**
 * @brief 串口1发送字符串
 * @param buff (uint8_t*)字符串指针
 * @return NONE
 */
void tjrc_asclin1_transmit(uint8_t *buff, uint32_t len)
{
    while (len)
    {
        Ifx_SizeT count = 1;
        (void)IfxAsclin_Asc_write(&asclin1_Handler, buff, &count, TIME_INFINITE);
        len--;
        buff++;
    }
}

/**
 * @brief 串口3发送定长数组
 * @param buff (uint8_t*)字符数组指针
 * @param len (uint32_t)数组长度
 * @return NONE
 */
void tjrc_asclin3_transmit(uint8_t *buff, uint32_t len)
{
    while (len)
    {
        Ifx_SizeT count = 1;
        (void)IfxAsclin_Asc_write(&asclin3_Handler, buff, &count, TIME_INFINITE);
        len--;
        buff++;
    }
}


/**
 * @brief 串口1接收1个字节数据
 * @param buff (uint8_t*)接受的字符
 * @return (uint8_t)成功读取：0 失败：1
 */
uint8_t tjrc_asclin0_receive(uint8_t *resDat)
{
    if (IfxAsclin_Asc_getReadCount(&asclin0_Handler) > 0)
    {
        *resDat = IfxAsclin_Asc_blockingRead(&asclin0_Handler);
        return 1;
    }
    return 0;
}

/**
 * @brief 串口1接收1个字节数据
 * @param buff (uint8_t*)接受的字符
 * @return (uint8_t)成功读取：0 失败：1
 */
uint8_t tjrc_asclin1_receive(uint8_t *resDat)
{
    if (IfxAsclin_Asc_getReadCount(&asclin1_Handler) > 0)
    {
        *resDat = IfxAsclin_Asc_blockingRead(&asclin1_Handler);
        return 1;
    }
    return 0;
}

/**
 * @brief 串口3接收1个字节数据
 * @param buff (uint8_t*)接受的字符
 * @return (uint8_t)成功读取：0 失败：1
 */
uint8_t tjrc_asclin3_receive(uint8_t *resDat)
{
    if (IfxAsclin_Asc_getReadCount(&asclin3_Handler) > 0)
    {
        *resDat = IfxAsclin_Asc_blockingRead(&asclin3_Handler);
        return 1;
    }
    return 0;
}



