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
#include <linux/spi/spi_vcoreiii.h>
#include <linux/spi/flash.h>
#include <linux/spi/mmc_spi.h>

#include <asm/mach-jaguar2/hardware.h>

static struct spi_vcoreiii_platform_data spi_jaguar2_cfg = {
	// .no_spi_delay = 1,
};

static struct platform_device jaguar2_spi = {
        .name             = SPI_VCOREIII_PLATDEV_NAME,
        .id               = 0,
        .dev = {
                .platform_data = &spi_jaguar2_cfg,
        },
};

static struct mtd_partition jaguar2_spi_flash_partitions[] = {
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

static struct flash_platform_data jaguar2_spi_flash_data = {
	.type = "m25p128",
	.name = "spi_flash",
	.parts = jaguar2_spi_flash_partitions,
	.nr_parts = ARRAY_SIZE(jaguar2_spi_flash_partitions),
        .read_mapped = 1,
        .phys_offset = 0x40000000,
};

/* MMC-SPI driver */
#if defined(CONFIG_MMC_SPI) || defined(CONFIG_MMC_SPI_MODULE)

/* #define MMC_SPI_CARD_DETECT_INT 74 // todo: needs to be verified */

/* static int jaguar2_mmc_spi_init(struct device *dev, */
/*                              irqreturn_t (*detect_int)(int, void *), void *data) */
/* { */
/*   return request_irq(MMC_SPI_CARD_DETECT_INT, detect_int, */
/*                      IRQF_TRIGGER_FALLING, "mmc-spi-detect", data); */
/* } */

/* static void jaguar2_mmc_spi_exit(struct device *dev, void *data) */
/* { */
/*   free_irq(MMC_SPI_CARD_DETECT_INT, data); */
/* } */

static struct mmc_spi_platform_data jaguar2_mmc_spi_pdata = {
  //  .init = jaguar2_mmc_spi_init,
  //  .exit = jaguar2_mmc_spi_exit,
  .detect_delay = 100, /* msecs */
};

static struct jaguar2_spi_chip  mmc_spi_chip_info = {
  .enable_dma = 0,
  .pio_interrupt = 0,
};
#endif


static struct spi_board_info jaguar2_spi_board_info[] __initdata = {
	{
		/* the modalias must be the same as spi device driver name */
		.modalias = "m25p80", /* Name of spi_driver for this device */
		.max_speed_hz = 50000000,     /* max spi clock (SCK) speed in HZ */
		.bus_num = 0, /* Framework bus number */
		.chip_select = 0, /* Framework chip select. */
		.platform_data = &jaguar2_spi_flash_data,
		.mode = SPI_MODE_0, /* CPOL=0, CPHA=0 */
        },

#if defined(CONFIG_MMC_SPI) || defined(CONFIG_MMC_SPI_MODULE)
	{
		.modalias = "mmc_spi",
		.max_speed_hz = 20000000,     /* max spi clock (SCK) speed in HZ */
		.bus_num = 0,
		.chip_select = 1,
		.platform_data = &jaguar2_mmc_spi_pdata,
		.controller_data = &mmc_spi_chip_info,
		.mode = SPI_MODE_0,
	},
#endif

};

static int __init vcoreiii_mtd_init(void)
{
	platform_device_register(&jaguar2_spi);

	spi_register_board_info(jaguar2_spi_board_info, ARRAY_SIZE(jaguar2_spi_board_info));

	return 0;
}

module_init(vcoreiii_mtd_init)
