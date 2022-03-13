/**
 * @file Peripheral.h
 * @author YYYDS team (1951578@tongji.edu.cn)
 * @brief 
 * @version 0.1
 * @date 2022-03-13
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef __PERIPHERAL_h
#define __PERIPHERAL_h

#include "../../tjrc_hardware_conf/tjrc_hardware.h"
#include "../../tjrc_perpherals/tjrc_peripherals.h"


void receive_data(void);

extern uint8_t steer_direct, run_direct;

#endif

