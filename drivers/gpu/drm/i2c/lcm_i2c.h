/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifndef __LCM_I2C_H__
#define __LCM_I2C_H__

//#include "lcm_drv.h"

typedef enum {
        LCM_STATUS_OK = 0,
        LCM_STATUS_ERROR,
}LCM_STATUS;

#define LCM_I2C_WRITE  1

typedef struct {
	char cmd;
	char data;
	char padding[130];
} LCM_DATA_T2;


int lcm_i2c_set_data(char type, const LCM_DATA_T2 *t2);
//LCM_STATUS lcm_i2c_set_data(char type, const LCM_DATA_T2 *t2);

#endif

