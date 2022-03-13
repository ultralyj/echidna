/**
 * @file tjrc_gpt12.h
 * @author YYYDS team (1951578@tongji.edu.cn)
 * @brief GPT12硬件配置层头文件，用于采集编码器数据
 * @version 0.1
 * @date 2022-03-13
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef LIBRARIES_TJRC_LIBRARIES_TJRC_GPT12_H_
#define LIBRARIES_TJRC_LIBRARIES_TJRC_GPT12_H_

#include "IfxGpt12.h"
#include "IfxGpt12_IncrEnc.h"
#include "rtthread.h"
#include "stdint.h"

void tjrc_setGpt12_encoder(void);
int16_t tjrc_gpt12_getT2(void);
int16_t tjrc_gpt12_getT6(void);

#endif /* LIBRARIES_TJRC_LIBRARIES_TJRC_GPT12_H_ */
