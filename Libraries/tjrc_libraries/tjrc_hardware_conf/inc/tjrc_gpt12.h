/*
 * tjrc_gpt12.h
 *
 *  Created on: 2022年2月23日
 *      Author: 11657
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
