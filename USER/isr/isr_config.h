/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ����Ⱥ��824575535
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file       		isr_config
 * @company	   		�ɶ���ɿƼ����޹�˾
 * @author     		��ɿƼ�(QQ3184284598)
 * @version    		�鿴doc��version�ļ� �汾˵��
 * @Software 		ADS v1.2.2
 * @Target core		TC264D
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2020-3-23
 ********************************************************************************************************************/

#ifndef _isr_config_h
#define _isr_config_h


//ISR_PRIORITY�� TC264����255���ж����ȼ��������� 1-255��0���ȼ���ʾ�������жϣ�255Ϊ������ȼ�
//�ر�ע��
//�ж����ȼ���������һ���������ж����ȼ�����������Ϊ��һ����ֵ
//�ر�ע��



//INT_SERVICE��    �궨������ж���˭����Ҳ��Ϊ�����ṩ�ߣ���TC264�У��жϱ��������񣩣������÷�ΧIfxSrc_Tos_cpu0 IfxSrc_Tos_cpu1 IfxSrc_Tos_dma  ��������Ϊ����ֵ



//------------�����жϲ�����ض���------------
#define	UART0_INT_SERVICE       IfxSrc_Tos_cpu0	//���崮��0�жϷ������ͣ����ж�����˭��Ӧ���� 0:CPU0 1:CPU1 3:DMA  ��������Ϊ����ֵ
#define UART0_TX_INT_PRIO       70	//���崮��0�����ж����ȼ� ���ȼ���Χ1-255 Խ�����ȼ�Խ�� ��ƽʱʹ�õĵ�Ƭ����һ��
#define UART0_RX_INT_PRIO       71	//���崮��0�����ж����ȼ� ���ȼ���Χ1-255 Խ�����ȼ�Խ�� ��ƽʱʹ�õĵ�Ƭ����һ��
#define UART0_ER_INT_PRIO       72	//���崮��0�����ж����ȼ� ���ȼ���Χ1-255 Խ�����ȼ�Խ�� ��ƽʱʹ�õĵ�Ƭ����һ��


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

#define ISR_PRIORITY_ERU(n)    (3+n)
#define ISR_PRIORITY_ERU_DMA    20

#endif
