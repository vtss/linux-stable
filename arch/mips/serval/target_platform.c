/*
 *
 * VCore-III platform ressources
 *
 * Copyright (C) 2010 Vitesse Semiconductor Inc.
 * Author: Lars Povlsen (lpovlsen@vitesse.com)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 * VITESSE SEMICONDUCTOR INC SHALL HAVE NO LIABILITY WHATSOEVER OF ANY
 * KIND ARISING OUT OF OR RELATED TO THE PROGRAM OR THE OPEN SOURCE
 * MATERIALS UNDER ANY THEORY OF LIABILITY.
 *
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/serial_8250.h>
#include <linux/uio_driver.h>

#include <asm/mach-serval/hardware.h>
#include <asm/mach-serval/i2c.h>

/* 8250 */
static struct plat_serial8250_port uart8250_vcoreiii_data[] = {
	{
		.mapbase	= 0x70100000,
		.irq		= UART_IRQ,
		.uartclk	= VCOREIII_AHB_CLOCK,
		.iotype		= UPIO_MEM32,
		.flags		= (UPF_BOOT_AUTOCONF | UPF_SKIP_TEST | UPF_IOREMAP),
		.regshift	= 2,
	},
	{ },
};

static struct platform_device uart8250_device = {
	.name			= "serial8250",
	.id			= PLAT8250_DEV_PLATFORM,
	.dev = {
		.platform_data = uart8250_vcoreiii_data,
	},
};

/* Switch UIO */
static struct uio_info switch_platform_data = {
    .name = "vcoreiii_switch",
    .version = "0",
    .irq = INT1_IRQ,    // Alternate IRQ. Application must remap IRQ destination
};

static struct resource switch_resources[] = {
    [0] = {
        .name   = "origin1_2",
        .start  = VTSS_IO_ORIGIN1_OFFSET,
        .end    = VTSS_IO_ORIGIN2_OFFSET + VTSS_IO_ORIGIN2_SIZE - 1,
        .flags  = IORESOURCE_MEM,
    },
};

static struct platform_device switch_device = {
    .name           = "uio_pdrv_genirq",
    .id             = 0,
    .dev = {
        .platform_data  = &switch_platform_data,
    },
    .resource       = switch_resources,
    .num_resources  = ARRAY_SIZE(switch_resources),
};

/* I2C */
static struct vcoreiii_i2c_platform_data i2c_data = {
    .fast_mode = 0,             /* 100 kbit/s */
};

static struct platform_device i2c_device = {
	.name			= "i2c_vcoreiii",
	.id			= -1,
	.dev = {
		.platform_data = &i2c_data,
	},
};

static struct platform_device *target_devices[] __initdata = {
    &uart8250_device,
    &switch_device,
    &i2c_device,
};

static int __init target_device_init(void)
{
    return platform_add_devices(target_devices, ARRAY_SIZE(target_devices)); 
}
module_init(target_device_init);

MODULE_AUTHOR("Lars Povlsen <lpovlsen@vitesse.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Platform drivers for Serval1 platform");
