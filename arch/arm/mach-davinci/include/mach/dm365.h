/*
 * Copyright (C) 2009 Texas Instruments Incorporated
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef __ASM_ARCH_DM365_H
#define __ASM_ARCH_DM665_H

#include <linux/platform_device.h>
#include <mach/hardware.h>
#include <media/davinci/vpfe_capture.h>
#include <mach/emac.h>
#include <mach/asp.h>
#include <mach/keyscan.h>

#define DM365_EMAC_BASE			(0x01D07000)
#define DM365_EMAC_CNTRL_OFFSET		(0x0000)
#define DM365_EMAC_CNTRL_MOD_OFFSET	(0x3000)
#define DM365_EMAC_CNTRL_RAM_OFFSET	(0x1000)
#define DM365_EMAC_MDIO_OFFSET		(0x4000)
#define DM365_EMAC_CNTRL_RAM_SIZE	(0x2000)

#define DM365_RTC_BASE			(0x01C69000)

#define DAVINCI_DM365_VC_BASE		(0x01D0C000)
#define DAVINCI_DMA_VC_TX		2
#define DAVINCI_DMA_VC_RX		3

/* Base of key scan register bank */
#define DM365_KEYSCAN_BASE		(0x01C69400)

void __init dm365_init(void);
void __init dm365_init_asp(struct snd_platform_data *pdata);
void __init dm365_init_vc(struct snd_platform_data *pdata);
void __init dm365_init_rtc(void);
void __init dm365_init_ks(struct davinci_ks_platform_data *pdata);
void dm365_set_vpfe_config(struct vpfe_config *cfg);

struct spi_board_info;
void dm365_init_spi0(unsigned chipselect_mask,
		struct spi_board_info *info, unsigned len);

#endif /* __ASM_ARCH_DM365_H */
