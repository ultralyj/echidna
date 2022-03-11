/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ����Ⱥ��824575535
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file       		uart
 * @company	   		�ɶ���ɿƼ����޹�˾
 * @author     		��ɿƼ�(QQ3184284598)
 * @version    		�鿴doc��version�ļ� �汾˵��
 * @Software 		ADS v1.2.2
 * @Target core		TC264D
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2020-3-23
 ********************************************************************************************************************/
 
 
#include "IFXPORT.h"
#include "stdio.h"
#include "ifxAsclin_reg.h"
#include "ifxCpu_Irq.h"
#include "IFXASCLIN_CFG.h"
#include "SysSe/Bsp/Bsp.h"
#include "zf_assert.h"
#include "isr_config.h"
#include "TC264_config.h"
#include "zf_uart.h"



//��������handle����
IfxAsclin_Asc uart0_handle;
IfxAsclin_Asc uart1_handle;
IfxAsclin_Asc uart2_handle;
IfxAsclin_Asc uart3_handle;

//����һ��ascConfig�Ľṹ����������ڴ��ڳ�ʼ��ʱֻ��
static IfxAsclin_Asc_Config uart_config;


//�������ڻ�������
static uint8 uart0_tx_buffer[UART0_TX_BUFFER_SIZE + sizeof(Ifx_Fifo) + 8];
static uint8 uart0_rx_buffer[UART0_RX_BUFFER_SIZE + sizeof(Ifx_Fifo) + 8];

static uint8 uart1_tx_buffer[UART1_TX_BUFFER_SIZE + sizeof(Ifx_Fifo) + 8];
static uint8 uart1_rx_buffer[UART1_RX_BUFFER_SIZE + sizeof(Ifx_Fifo) + 8];

static uint8 uart2_tx_buffer[UART2_TX_BUFFER_SIZE + sizeof(Ifx_Fifo) + 8];
static uint8 uart2_rx_buffer[UART2_RX_BUFFER_SIZE + sizeof(Ifx_Fifo) + 8];

static uint8 uart3_tx_buffer[UART3_TX_BUFFER_SIZE + sizeof(Ifx_Fifo) + 8];
static uint8 uart3_rx_buffer[UART3_RX_BUFFER_SIZE + sizeof(Ifx_Fifo) + 8];







void uart_set_interrupt_priority(UARTN_enum uartn)
{
	switch(uartn)
	{
		case UART_0:
		{
			uart_config.interrupt.txPriority 	= UART0_TX_INT_PRIO;
			uart_config.interrupt.rxPriority 	= UART0_RX_INT_PRIO;
			uart_config.interrupt.erPriority 	= UART0_ER_INT_PRIO;
			uart_config.interrupt.typeOfService = UART0_INT_SERVICE;
		}break;
		default: ZF_ASSERT(FALSE);
	}
}

void uart_set_buffer(UARTN_enum uartn)
{
	switch(uartn)
	{
		case UART_0:
		{
			uart_config.txBuffer 	 = &uart0_tx_buffer;
			uart_config.rxBuffer 	 = &uart0_rx_buffer;
			uart_config.txBufferSize = UART0_TX_BUFFER_SIZE;
			uart_config.rxBufferSize = UART0_RX_BUFFER_SIZE;
		}break;
		case UART_1:
		{
			uart_config.txBuffer 	 = &uart1_tx_buffer;
			uart_config.rxBuffer 	 = &uart1_rx_buffer;
			uart_config.txBufferSize = UART1_TX_BUFFER_SIZE;
			uart_config.rxBufferSize = UART1_RX_BUFFER_SIZE;
		}break;
		case UART_2:
		{
			uart_config.txBuffer 	 = &uart2_tx_buffer;
			uart_config.rxBuffer 	 = &uart2_rx_buffer;
			uart_config.txBufferSize = UART2_TX_BUFFER_SIZE;
			uart_config.rxBufferSize = UART2_RX_BUFFER_SIZE;
		}break;
		case UART_3:
		{
			uart_config.txBuffer 	 = &uart3_tx_buffer;
			uart_config.rxBuffer  	 = &uart3_rx_buffer;
			uart_config.txBufferSize = UART3_TX_BUFFER_SIZE;
			uart_config.rxBufferSize = UART3_RX_BUFFER_SIZE;
		}break;
		default: IFX_ASSERT(IFX_VERBOSE_LEVEL_ERROR, FALSE);
	}
}


IfxAsclin_Asc* uart_get_handle(UARTN_enum uartn)
{
	switch(uartn)
	{
		case UART_0:	 return &uart0_handle;
		case UART_1:	 return &uart1_handle;
		case UART_2:	 return &uart2_handle;
		case UART_3:	 return &uart3_handle;
		default: IFX_ASSERT(IFX_VERBOSE_LEVEL_ERROR, FALSE);
	}

	return NULL;
}

void uart_mux(UARTN_enum uartn, UART_PIN_enum tx_pin, UART_PIN_enum rx_pin, uint32 *set_tx_pin, uint32 *set_rx_pin)
{
	switch(uartn)
	{
		case UART_0:
		{
			if	   (UART0_TX_P14_0 == tx_pin)	*set_tx_pin = (uint32)&IfxAsclin0_TX_P14_0_OUT;
			else if(UART0_TX_P14_1 == tx_pin)	*set_tx_pin = (uint32)&IfxAsclin0_TX_P14_1_OUT;
			else if(UART0_TX_P15_2 == tx_pin)	*set_tx_pin = (uint32)&IfxAsclin0_TX_P15_2_OUT;
			else if(UART0_TX_P15_3 == tx_pin)	*set_tx_pin = (uint32)&IfxAsclin0_TX_P15_3_OUT;
			else IFX_ASSERT(IFX_VERBOSE_LEVEL_ERROR, FALSE);

			if	   (UART0_RX_P14_1 == rx_pin)	*set_rx_pin = (uint32)&IfxAsclin0_RXA_P14_1_IN;
			else if(UART0_RX_P15_3 == rx_pin)	*set_rx_pin = (uint32)&IfxAsclin0_RXB_P15_3_IN;
			else IFX_ASSERT(IFX_VERBOSE_LEVEL_ERROR, FALSE);

		}break;
		case UART_1:
		{
			if	   (UART1_TX_P02_2  == tx_pin)	*set_tx_pin = (uint32)&IfxAsclin1_TX_P02_2_OUT;
			else if(UART1_TX_P11_12 == tx_pin)	*set_tx_pin = (uint32)&IfxAsclin1_TX_P11_12_OUT;
			else if(UART1_TX_P14_10 == tx_pin)	*set_tx_pin = (uint32)&IfxAsclin1_TX_P14_10_OUT;
			else if(UART1_TX_P15_0  == tx_pin)	*set_tx_pin = (uint32)&IfxAsclin1_TX_P15_0_OUT;
			else if(UART1_TX_P15_1  == tx_pin)	*set_tx_pin = (uint32)&IfxAsclin1_TX_P15_1_OUT;
			else if(UART1_TX_P15_4  == tx_pin)	*set_tx_pin = (uint32)&IfxAsclin1_TX_P15_4_OUT;
			else if(UART1_TX_P15_5  == tx_pin)	*set_tx_pin = (uint32)&IfxAsclin1_TX_P15_5_OUT;
			else if(UART1_TX_P20_10 == tx_pin)	*set_tx_pin = (uint32)&IfxAsclin1_TX_P20_10_OUT;
			else if(UART1_TX_P33_12 == tx_pin)	*set_tx_pin = (uint32)&IfxAsclin1_TX_P33_12_OUT;
			else if(UART1_TX_P33_13 == tx_pin)	*set_tx_pin = (uint32)&IfxAsclin1_TX_P33_13_OUT;
			else IFX_ASSERT(IFX_VERBOSE_LEVEL_ERROR, FALSE);

			if	   (UART1_RX_P15_1  == rx_pin)	*set_rx_pin = (uint32)&IfxAsclin1_RXA_P15_1_IN;
			else if(UART1_RX_P15_5  == rx_pin)	*set_rx_pin = (uint32)&IfxAsclin1_RXB_P15_5_IN;
			else if(UART1_RX_P20_9  == rx_pin)	*set_rx_pin = (uint32)&IfxAsclin1_RXC_P20_9_IN;
			else if(UART1_RX_P14_8  == rx_pin)	*set_rx_pin = (uint32)&IfxAsclin1_RXD_P14_8_IN;
			else if(UART1_RX_P11_10 == rx_pin)	*set_rx_pin = (uint32)&IfxAsclin1_RXE_P11_10_IN;
			else if(UART1_RX_P33_13 == rx_pin)	*set_rx_pin = (uint32)&IfxAsclin1_RXF_P33_13_IN;
			else if(UART1_RX_P02_3  == rx_pin)	*set_rx_pin = (uint32)&IfxAsclin1_RXG_P02_3_IN;
			else IFX_ASSERT(IFX_VERBOSE_LEVEL_ERROR, FALSE);

		}break;
		case UART_2:
		{
			if	   (UART2_TX_P02_0 == tx_pin)	*set_tx_pin = (uint32)&IfxAsclin2_TX_P02_0_OUT;
			else if(UART2_TX_P10_5 == tx_pin)	*set_tx_pin = (uint32)&IfxAsclin2_TX_P10_5_OUT;
			else if(UART2_TX_P14_2 == tx_pin)	*set_tx_pin = (uint32)&IfxAsclin2_TX_P14_2_OUT;
			else if(UART2_TX_P14_3 == tx_pin)	*set_tx_pin = (uint32)&IfxAsclin2_TX_P14_3_OUT;
			else if(UART2_TX_P33_8 == tx_pin)	*set_tx_pin = (uint32)&IfxAsclin2_TX_P33_8_OUT;
			else if(UART2_TX_P33_9 == tx_pin)	*set_tx_pin = (uint32)&IfxAsclin2_TX_P33_9_OUT;
			else IFX_ASSERT(IFX_VERBOSE_LEVEL_ERROR, FALSE);

			if	   (UART2_RX_P14_3 == rx_pin)	*set_rx_pin = (uint32)&IfxAsclin2_RXA_P14_3_IN;
			else if(UART2_RX_P02_1 == rx_pin)	*set_rx_pin = (uint32)&IfxAsclin2_RXB_P02_1_IN;
			else if(UART2_RX_P10_6 == rx_pin)	*set_rx_pin = (uint32)&IfxAsclin2_RXD_P10_6_IN;
			else if(UART2_RX_P33_8 == rx_pin)	*set_rx_pin = (uint32)&IfxAsclin2_RXE_P33_8_IN;
			else if(UART2_RX_P02_0 == rx_pin)	*set_rx_pin = (uint32)&IfxAsclin2_RXG_P02_0_IN;

			else IFX_ASSERT(IFX_VERBOSE_LEVEL_ERROR, FALSE);

		}break;
		case UART_3:
		{
			if	   (UART3_TX_P00_0 == tx_pin)	*set_tx_pin = (uint32)&IfxAsclin3_TX_P00_0_OUT;
			else if(UART3_TX_P00_1 == tx_pin)	*set_tx_pin = (uint32)&IfxAsclin3_TX_P00_1_OUT;
			else if(UART3_TX_P15_6 == tx_pin)	*set_tx_pin = (uint32)&IfxAsclin3_TX_P15_6_OUT;
			else if(UART3_TX_P15_7 == tx_pin)	*set_tx_pin = (uint32)&IfxAsclin3_TX_P15_7_OUT;
			else if(UART3_TX_P20_0 == tx_pin)	*set_tx_pin = (uint32)&IfxAsclin3_TX_P20_0_OUT;
			else if(UART3_TX_P20_3 == tx_pin)	*set_tx_pin = (uint32)&IfxAsclin3_TX_P20_3_OUT;
			else if(UART3_TX_P21_7 == tx_pin)	*set_tx_pin = (uint32)&IfxAsclin3_TX_P21_7_OUT;
			else if(UART3_TX_P32_2 == tx_pin)	*set_tx_pin = (uint32)&IfxAsclin3_TX_P32_2_OUT;
			else if(UART3_TX_P32_3 == tx_pin)	*set_tx_pin = (uint32)&IfxAsclin3_TX_P32_3_OUT;
			else IFX_ASSERT(IFX_VERBOSE_LEVEL_ERROR, FALSE);

			if	   (UART3_RX_P15_7 == rx_pin)	*set_rx_pin = (uint32)&IfxAsclin3_RXA_P15_7_IN;
			else if(UART3_RX_P20_3 == rx_pin)	*set_rx_pin = (uint32)&IfxAsclin3_RXC_P20_3_IN;
			else if(UART3_RX_P32_2 == rx_pin)	*set_rx_pin = (uint32)&IfxAsclin3_RXD_P32_2_IN;
			else if(UART3_RX_P00_1 == rx_pin)	*set_rx_pin = (uint32)&IfxAsclin3_RXE_P00_1_IN;
			else if(UART3_RX_P21_6 == rx_pin)	*set_rx_pin = (uint32)&IfxAsclin3_RXF_P21_6_IN;
			else IFX_ASSERT(IFX_VERBOSE_LEVEL_ERROR, FALSE);

		}break;
	}
}



//-------------------------------------------------------------------------------------------------------------------
//  @brief      ���ڳ�ʼ��
//  @param      uartn           ����ģ���(UART_0,UART_1,UART_2,UART_3)
//  @param      baud            ���ڲ�����
//  @param      tx_pin          ���ڷ�������
//  @param      rx_pin          ���ڽ�������
//  @return     uint32          ʵ�ʲ�����
//  Sample usage:               uart_init(UART_0,115200,UART0_TX_P14_0,UART0_RX_P14_1);       // ��ʼ������0 ������115200 ��������ʹ��P14_0 ��������ʹ��P14_1
//-------------------------------------------------------------------------------------------------------------------
void uart_init(UARTN_enum uartn, uint32 baud, UART_PIN_enum tx_pin, UART_PIN_enum rx_pin)
{
	boolean interrupt_state = disableInterrupts();

	volatile Ifx_ASCLIN *moudle = IfxAsclin_getAddress((IfxAsclin_Index)uartn);

	IfxAsclin_Asc_initModuleConfig(&uart_config, moudle); 		//��ʼ�������ýṹ��

	uart_set_buffer(uartn);//���û�����
	uart_set_interrupt_priority(uartn);//�����ж����ȼ�

    uart_config.baudrate.prescaler    = 4;
    uart_config.baudrate.baudrate     = (float32)baud;
    uart_config.baudrate.oversampling = IfxAsclin_OversamplingFactor_8;

    IfxAsclin_Asc_Pins pins;//��������
    pins.cts = NULL;
    pins.rts = NULL;
    uart_mux(uartn, tx_pin, rx_pin, (uint32 *)&pins.tx, (uint32 *)&pins.rx);
    pins.rxMode = IfxPort_InputMode_pullUp;
    pins.txMode = IfxPort_OutputMode_pushPull;
    pins.pinDriver = IfxPort_PadDriver_cmosAutomotiveSpeed1;
    uart_config.pins = &pins;

    IfxAsclin_Asc_initModule(uart_get_handle(uartn), &uart_config);

    restoreInterrupts(interrupt_state);
}




//-------------------------------------------------------------------------------------------------------------------
//  @brief      �����ֽ����
//  @param      uartn           ����ģ���(UART_0,UART_1,UART_2,UART_3)
//  @param      dat             ��Ҫ���͵��ֽ�
//  @return     void        
//  Sample usage:               uart_putchar(UART_0, 0xA5);       // ����0����0xA5
//-------------------------------------------------------------------------------------------------------------------
void uart_putchar(UARTN_enum uartn, uint8 dat)
{
//	IfxAsclin_Asc_blockingWrite(uart_get_handle(uartn),dat);
	Ifx_SizeT count = 1;
	(void)IfxAsclin_Asc_write(uart_get_handle(uartn), &dat, &count, TIME_INFINITE);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ���ڷ�������
//  @param      uartn           ����ģ���(UART_0,UART_1,UART_2,UART_3)
//  @param      *buff           Ҫ���͵������ַ
//  @param      len             ���ͳ���
//  @return     void
//  Sample usage:               uart_putbuff(UART_0,&a[0],5);
//-------------------------------------------------------------------------------------------------------------------
void uart_putbuff(UARTN_enum uartn, uint8 *buff, uint32 len)
{
	while(len)
	{
		uart_putchar(uartn, *buff);
		len--;
		buff++;
	}
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ���ڷ����ַ���
//  @param      uartn           ����ģ���(UART_0,UART_1,UART_2,UART_3)
//  @param      *str            Ҫ���͵��ַ�����ַ
//  @return     void
//  Sample usage:               uart_putstr(UART_0,"i lvoe you");
//-------------------------------------------------------------------------------------------------------------------
void uart_putstr(UARTN_enum uartn, const int8 *str)
{
    while(*str)
    {
        uart_putchar(uartn, *str++);
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��ȡ���ڽ��յ����ݣ�whlie�ȴ���
//  @param      uartn           ����ģ���(UART_0,UART_1,UART_2,UART_3)
//  @param      *dat            �������ݵĵ�ַ
//  @return     void        
//  Sample usage:               uint8 dat; uart_getchar(UART_0,&dat);       // ���մ���0����  ������dat������
//-------------------------------------------------------------------------------------------------------------------
void uart_getchar(UARTN_enum uartn, uint8 *dat)
{
	while(!IfxAsclin_Asc_getReadCount(uart_get_handle(uartn)));
	*dat = IfxAsclin_Asc_blockingRead(uart_get_handle(uartn));
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��ȡ���ڽ��յ����ݣ���ѯ���գ�
//  @param      uartn           ����ģ���(UART_0,UART_1,UART_2,UART_3)
//  @param      *dat            �������ݵĵ�ַ
//  @return     uint8           1�����ճɹ�   0��δ���յ�����
//  Sample usage:               uint8 dat; uart_query(UART_0,&dat);       // ���մ���0����  ������dat������
//-------------------------------------------------------------------------------------------------------------------
uint8 uart_query(UARTN_enum uartn, uint8 *dat)
{
	if(IfxAsclin_Asc_getReadCount(uart_get_handle(uartn)) >0)
	{
		*dat = IfxAsclin_Asc_blockingRead(uart_get_handle(uartn));
		return 1;
	}
    return 0;
}

#if(1 == PRINTF_ENABLE)
//-------------------------------------------------------------------------------------------------------------------
//  @brief      �ض���printf ������
//  @param      ch      ��Ҫ��ӡ���ֽ�
//  @param      stream  ������
//  @note       �˺����ɱ������Դ������printf������
//-------------------------------------------------------------------------------------------------------------------
int fputc(int ch, FILE *stream)
{
    uart_putchar(DEBUG_UART, (char)ch);
    return(ch);
}
#endif

