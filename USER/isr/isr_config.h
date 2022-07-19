/**
 * @file isr_config.h
 * @author YYYDS team (1951578@tongji.edu.cn)
 * @brief 中断优先级配置头文件
 * @version 0.1
 * @date 2022-03-13
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef _isr_config_h
#define _isr_config_h

/**
 * @brief 优先级范围为0-255 数值越大，优先级越高（与主流相反）
 * 
 */


#define ISR_PRIORITY_DMA_CH1    93
#define ISR_PRIORITY_DMA_CH2    94

#define ISR_PRIORITY_ASCLIN0_TX 70
#define ISR_PRIORITY_ASCLIN0_RX 71
#define ISR_PRIORITY_ASCLIN0_ER 72

#define ISR_PRIORITY_ASCLIN1_TX 73
#define ISR_PRIORITY_ASCLIN1_RX 104
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
