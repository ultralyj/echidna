

#ifndef CODE_SCHEDULING_H_
#define CODE_SCHEDULING_H_

#include "../../tjrc_perpherals/tjrc_peripherals.h"
#include "rtthread.h"
#include "Peripheral.h"
#include "tjrc_kalmam.h"
#include "tjrc_pid_control.h"

extern rt_sem_t Run_sem;

void balance_thread_init(void);
void Run_init(void);

#endif /* CODE_SCHEDULING_H_ */
