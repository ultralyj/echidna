/**
 * @file tjrc_wireless.h
 * @author YYYDS team (1951578@tongji.edu.cn)
 * @brief 无线串口接收函数的头文件
 * @version 0.1
 * @date 2022-03-13
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef __TJRC_WIRELESS__
#define __TJRC_WIRELESS__

#include "tjrc_hardware.h"
#include "tjrc_peripherals.h"
#include "tjrc_algorithm.h"

void tjrc_wireless_recCallBack(void);
void tjrc_wireless_sendImage(uint8_t* image, uint8_t threshold);


#endif

