/*
 * tjrc_threads.h
 *
 *  Created on: 2022年3月12日
 *      Author: 11657
 */

#ifndef LIBRARIES_TJRC_LIBRARIES_TJRC_THREADS_TJRC_THREADS_H_
#define LIBRARIES_TJRC_LIBRARIES_TJRC_THREADS_TJRC_THREADS_H_

#include "rtthread.h"
#include "tjrc_peripherals.h"
#include "tjrc_hardware.h"
#include "tjrc_algorithm.h"
#include "tjrc_systemInit.h"
#include "TC264_config.h"

void tjrc_thread_life_init(void);
void tjrc_thread_key_init(void);
void tjrc_thread_camera_init(void);

void tjrc_thread_balance_init(void);
void tjrc_thread_run_init(void);

#endif /* LIBRARIES_TJRC_LIBRARIES_TJRC_THREADS_TJRC_THREADS_H_ */
