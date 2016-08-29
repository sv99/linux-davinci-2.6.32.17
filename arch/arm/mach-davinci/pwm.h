/*
 *  DaVinci PWM definitions
 *
 *  Copyright (C) 2013 Linear LLC
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 */

#ifndef _DAVINCI_PWM_H
#define _DAVINCI_PWM_H

#define DAVINCI_PWM_SIZE	0x400
#define DAVINCI_PWM0_BASE	0x01C22000
#define DAVINCI_PWM1_BASE	0x01C22400
#define DAVINCI_PWM2_BASE	0x01C22800
#define DAVINCI_PWM3_BASE	0x01C22C00

#define DAVINCI_PWM_TIMEOUT	(1 * HZ)

#define DAVINCI_PWM_REG_PID     0x00    /* Peripheral ID */
#define DAVINCI_PWM_REG_PCR     0x04    /* Peripheral Control */
#define DAVINCI_PWM_REG_CFG     0x08    /* Configuration */
#define DAVINCI_PWM_REG_START   0x0C    /* Start */
#define DAVINCI_PWM_REG_RPT     0x10    /* Repeat Count */
#define DAVINCI_PWM_REG_PER     0x14    /* Period */
#define DAVINCI_PWM_REG_PH1D    0x18    /* First-Phase Duration */

#endif /* _DAVINCI_PWM_H */
