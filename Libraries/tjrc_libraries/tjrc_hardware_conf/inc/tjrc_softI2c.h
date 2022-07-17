/*
 * tjrc_softI2c.h
 *
 *  Created on: 2022年4月26日
 *      Author: 11657
 */

#ifndef LIBRARIES_TJRC_LIBRARIES_TJRC_HARDWARE_CONF_INC_TJRC_SOFTI2C_H_
#define LIBRARIES_TJRC_LIBRARIES_TJRC_HARDWARE_CONF_INC_TJRC_SOFTI2C_H_

#define I2C_SCL_PIN &MODULE_P13, 1
#define I2C_SDA_PIN &MODULE_P13, 2

#include "tjrc_hardware.h"

void simiic_init(void);
void simiic_write_reg(uint8 dev_add, uint8 reg, uint8 dat);
uint8 simiic_read_reg(uint8 dev_add, uint8 reg);
void simiic_read_regs(uint8 dev_add, uint8 reg, uint8 *dat_add, uint8 num);

#endif /* LIBRARIES_TJRC_LIBRARIES_TJRC_HARDWARE_CONF_INC_TJRC_SOFTI2C_H_ */
