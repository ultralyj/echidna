/*
 * ifx_hardware.h
 *
 *  Created on: 2022年2月7日
 *      Author: 11657
 */

#ifndef _IFX_PERIPHERALS_H_
#define _IFX_PERIPHERALS_H_


#include "rtthread.h"
#include "stdint.h"

#include "tjrc_basic.h"
#include "tjrc_spi.h"
#include "tjrc_ccu6.h"
#include "tjrc_gpt12.h"
#include "tjrc_gtm.h"

#if USE_SOFTWATE_I2C
#include "tjrc_softI2c.h"
#else
#include "tjrc_iic.h"
#endif

#include "tjrc_vadc.h"
#include "tjrc_eru.h"


#endif /* LIBRARIES_TJRC_LIBRARIES_IRF_PERIPHERALS_H_ */


