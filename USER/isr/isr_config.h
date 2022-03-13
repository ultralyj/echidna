
#ifndef _isr_config_h
#define _isr_config_h




//------------�����жϲ�����ض���------------
#define	UART0_INT_SERVICE       IfxSrc_Tos_cpu0
#define UART0_TX_INT_PRIO       70
#define UART0_RX_INT_PRIO       71
#define UART0_ER_INT_PRIO       72


#define ISR_PRIORITY_ASCLIN0_TX 70
#define ISR_PRIORITY_ASCLIN0_RX 71
#define ISR_PRIORITY_ASCLIN0_ER 72

#define ISR_PRIORITY_ASCLIN1_TX 73
#define ISR_PRIORITY_ASCLIN1_RX 74
#define ISR_PRIORITY_ASCLIN1_ER 75

#define ISR_PRIORITY_ASCLIN3_TX 76
#define ISR_PRIORITY_ASCLIN3_RX 78
#define ISR_PRIORITY_ASCLIN3_ER 79

#define ISR_PRIORITY_QSPI0_RX 92
#define ISR_PRIORITY_QSPI0_TX 91
#define ISR_PRIORITY_QSPI0_ER 90

#define ISR_PRIORITY_ERU_CH04   42
#define ISR_PRIORITY_ERU_CH15   43
#define ISR_PRIORITY_ERU_CH26   5   // DMA_ERU PCLK中断 优先级不可动
#define ISR_PRIORITY_ERU_CH37   44

#define ISR_PRIORITY_ERU_DMA    80

#endif
