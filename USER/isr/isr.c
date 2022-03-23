/**
 * @file isr.c
 * @author YYYDS team (1951578@tongji.edu.cn)
 * @brief 集中了中断响应函数
 * @version 0.1
 * @date 2022-03-10
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <tjrc_hardware.h>
#include <tjrc_peripherals.h>
#include "rtthread.h"
#include "isr_config.h"
#include "isr.h"


/**
 * @brief QSPI0 RX DMA中断响应函数
 * 
 */
IFX_INTERRUPT(QSPI0_RX_DMA_IRQHandler, 0, ISR_PRIORITY_DMA_CH2)
{
    rt_interrupt_enter();
	extern IfxQspi_SpiMaster qspi0_master_handler;
	IfxQspi_SpiMaster_isrDmaTransmit(&qspi0_master_handler);
    rt_interrupt_leave();
}

/**
 * @brief QSPI0 TX DMA中断响应函数
 * 
 */
IFX_INTERRUPT(QSPI0_TX_DMA_IRQHandler, 0, ISR_PRIORITY_DMA_CH1)
{
    rt_interrupt_enter();
	extern IfxQspi_SpiMaster qspi0_master_handler;
	IfxQspi_SpiMaster_isrDmaReceive(&qspi0_master_handler);
    rt_interrupt_leave();
}

///**
// * @brief QSPI0 RX 中断响应函数
// *
// */
//IFX_INTERRUPT(QSPI0_RX_IRQHandler, 0, ISR_PRIORITY_QSPI0_RX)
//{
//    rt_interrupt_enter();
//    extern IfxQspi_SpiMaster qspi0_master_handler;
//    IfxQspi_SpiMaster_isrReceive(&qspi0_master_handler);
//    rt_interrupt_leave();
//}
//
///**
// * @brief QSPI0 TX 中断响应函数
// *
// */
//IFX_INTERRUPT(QSPI0_TX_IRQHandler, 0, ISR_PRIORITY_QSPI0_TX)
//{
//    rt_interrupt_enter();
//    extern IfxQspi_SpiMaster qspi0_master_handler;
//    IfxQspi_SpiMaster_isrReceive(&qspi0_master_handler);
//    rt_interrupt_leave();
//}

/**
 * @brief QSPI0 ER 中断响应函数
 * 
 */
IFX_INTERRUPT(QSPI0_ER_IRQHandler, 0, ISR_PRIORITY_QSPI0_ER)
{
    rt_interrupt_enter();
	extern IfxQspi_SpiMaster qspi0_master_handler;
    IfxQspi_SpiMaster_isrReceive(&qspi0_master_handler);
    rt_interrupt_leave();
}



/**
 * @brief ASCLIN0 TX 中断响应函数
 * 
 */
IFX_INTERRUPT(uart0_tx_isr, 0, UART0_TX_INT_PRIO)
{
	rt_interrupt_enter();  
    IfxAsclin_Asc_isrTransmit(&uart0_handle);
	rt_interrupt_leave();   
}


/**
 * @brief ASCLIN0 RX 中断响应函数
 * 
 */
IFX_INTERRUPT(uart0_rx_isr, 0, UART0_RX_INT_PRIO)
{
	rt_interrupt_enter();   
	
    extern rt_mailbox_t uart_mb;
	uint8 dat;
	enableInterrupts();
    IfxAsclin_Asc_isrReceive(&uart0_handle);
    uart_getchar(DEBUG_UART, &dat);
    rt_mb_send(uart_mb, dat);           
	
	rt_interrupt_leave();   
}


/**
 * @brief ASCLIN0 ER 中断响应函数
 * 
 */
IFX_INTERRUPT(uart0_er_isr, 0, UART0_ER_INT_PRIO)
{
	rt_interrupt_enter();  
	
	enableInterrupts();
    IfxAsclin_Asc_isrError(&uart0_handle);
	
	rt_interrupt_leave();   
}

/**
 * @brief ASCLIN1 TX 中断响应函数
 *
 */
IFX_INTERRUPT(ASCLIN1_TX_IRQHandler, 0, ISR_PRIORITY_ASCLIN1_TX)
{
    extern IfxAsclin_Asc asclin1_Handler;
    rt_interrupt_enter();

    enableInterrupts();
    IfxAsclin_Asc_isrTransmit(&asclin1_Handler);

    rt_interrupt_leave();
}

/**
 * @brief ASCLIN1 RX 中断响应函数
 *
 */
IFX_INTERRUPT(ASCLIN1_RX_IRQHandler, 0, ISR_PRIORITY_ASCLIN1_RX)
{
    extern IfxAsclin_Asc asclin1_Handler;
    rt_interrupt_enter();

    enableInterrupts();
    IfxAsclin_Asc_isrReceive(&asclin1_Handler);
    rt_interrupt_leave();
}

/**
 * @brief ASCLIN1 ER 中断响应函数
 *
 */
IFX_INTERRUPT(ASCLIN1_ER_IRQHandler, 0, ISR_PRIORITY_ASCLIN1_ER)
{
    extern IfxAsclin_Asc asclin1_Handler;
    rt_interrupt_enter();

    enableInterrupts();
    IfxAsclin_Asc_isrError(&asclin1_Handler);

    rt_interrupt_leave();
}


/**
 * @brief ASCLIN3 TX 中断响应函数
 * 
 */
IFX_INTERRUPT(ASCLIN3_TX_IRQHandler, 0, ISR_PRIORITY_ASCLIN3_TX)
{
    extern IfxAsclin_Asc asclin3_Handler;
	rt_interrupt_enter();  
	
	enableInterrupts();
    IfxAsclin_Asc_isrTransmit(&asclin3_Handler);
	
	rt_interrupt_leave();   
}

/**
 * @brief ASCLIN3 RX 中断响应函数
 * 
 */
IFX_INTERRUPT(ASCLIN3_RX_IRQHandler, 0, ISR_PRIORITY_ASCLIN3_RX)
{
    extern IfxAsclin_Asc asclin3_Handler;
	rt_interrupt_enter();  
	
	enableInterrupts();
    IfxAsclin_Asc_isrReceive(&asclin3_Handler);
    tjrc_mt9v03x_uartCallBack();
	rt_interrupt_leave();   
}

/**
 * @brief ASCLIN3 ER 中断响应函数
 * 
 */
IFX_INTERRUPT(ASCLIN3_ER_IRQHandler, 0, ISR_PRIORITY_ASCLIN3_ER)
{
    extern IfxAsclin_Asc asclin3_Handler;
	rt_interrupt_enter();   
	
	enableInterrupts();
    IfxAsclin_Asc_isrError(&asclin3_Handler);
	
	rt_interrupt_leave();  
}

/**
 * @brief DMA 中断响应函数，处理DMA传输完成事件
 * 
 */
IFX_INTERRUPT(ERU_DMA_IRQHandler, 0, ISR_PRIORITY_ERU_DMA)
{
    rt_interrupt_enter();
    rt_enter_critical();

    tjrc_mt9v03x_dmaCallBack();

    extern rt_sem_t camera_irq_sem;
    rt_sem_release(camera_irq_sem);
    rt_exit_critical();
    rt_interrupt_leave();
}

/**
 * @brief ERU CHANNEL 3&7 中断响应函数，用于处理摄像头的场中断信号
 * 
 */
IFX_INTERRUPT(ERU_CH37_IRQHandler, 0, ISR_PRIORITY_ERU_CH37)
{
    rt_interrupt_enter();
    tjrc_mt9v03x_vSync();
    rt_interrupt_leave();
}

/**
 * @brief ERU CHANNEL 1&5 中断响应函数，用于处理IMU中断信号输入
 *
 */
IFX_INTERRUPT(ERU_CH15_IRQHandler, 0, ISR_PRIORITY_ERU_CH15)
{
    rt_interrupt_enter();

    extern rt_sem_t imu_irq_sem;
    extern uint8_t balance_thread_flag;
    if(balance_thread_flag)
        rt_sem_release(imu_irq_sem);
    rt_interrupt_leave();

}
