/*
 *
 * VCore-III MTD platform driver
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
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <mtd/mtd-abi.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi_serval.h>
#include <linux/spi/flash.h>

#include <asm/mach-serval/hardware.h>

static struct spi_serval_platform_data spi_serval_cfg = {
	// .no_spi_delay = 1,
};

static struct platform_device serval_spi = {
        .name             = SPI_SERVAL_PLATDEV_NAME,
        .id               = 0,
        .dev = {
                .platform_data = &spi_serval_cfg,
        },
};

static struct mtd_partition serval_spi_flash_partitions[] = {
	{
		.name =		"redboot",
		.offset = 	0x00000000,
		.size =		0x00040000,
	}, {
		.name =		"config",
		.offset = 	MTDPART_OFS_APPEND,
		.size =		0x00040000,
	}, {
		.name =		"linux",
		.offset = 	0x00800000,
		.size =		0x00200000,
	}, {
		.name =		"rootfs",
		.offset = 	MTDPART_OFS_APPEND,
//		.offset =       0x00800000 + 0x00200000, /* ROOTFS_SPLIT does not like MTDPART_OFS_APPEND */
		.size =		0x00500000,
	}
};

static struct flash_platform_data serval_spi_flash_data = {
	.type = "m25p128",
	.name = "spi_flash",
	.parts = serval_spi_flash_partitions,
	.nr_parts = ARRAY_SIZE(serval_spi_flash_partitions),
        .read_mapped = 1,
        .phys_offset = 0x40000000,
};

static struct spi_board_info serval_spi_board_info[] __initdata = {
	{
		/* the modalias must be the same as spi device driver name */
		.modalias = "m25p80", /* Name of spi_driver for this device */
		.max_speed_hz = 50000000,     /* max spi clock (SCK) speed in HZ */
		.bus_num = 0, /* Framework bus number */
		.chip_select = 0, /* Framework chip select. */
		.platform_data = &serval_spi_flash_data,
		.mode = SPI_MODE_0, /* CPOL=0, CPHA=0 */
	},
};

static int __init vcoreiii_mtd_init(void)
{
	platform_device_register(&serval_spi);

	spi_register_board_info(serval_spi_board_info, ARRAY_SIZE(serval_spi_board_info));

	return 0;
}

module_init(vcoreiii_mtd_init)
