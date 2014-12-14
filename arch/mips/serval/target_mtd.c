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

#if defined(CONFIG_MMC_SPI) || defined(CONFIG_MMC_SPI_MODULE)
#include <linux/mmc/host.h>
#include <linux/spi/mmc_spi.h>
#endif

#include <asm/mach-serval/hardware.h>

static struct spi_vcoreiii_platform_data spi_serval_cfg = {
	// .no_spi_delay = 1,
};

static struct platform_device serval_spi = {
        .name             = SPI_VCOREIII_PLATDEV_NAME,
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
		.offset = 	0x00880000,
		.size =		0x00200000,
	}, {
		.name =		"rootfs",
		.offset = 	MTDPART_OFS_APPEND,
		.size =		0x00480000,
	}
};

static struct flash_platform_data serval_spi_flash_data = {
	.type = "mx25l12805d",
	.name = "spi_flash",
	.parts = serval_spi_flash_partitions,
	.nr_parts = ARRAY_SIZE(serval_spi_flash_partitions),
        .read_mapped = 1,
        .phys_offset = 0x40000000,
};

/* MMC-SPI driver */
#if defined(CONFIG_MMC_SPI) || defined(CONFIG_MMC_SPI_MODULE)
static int vcoreiii_mmc_spi_init(struct device *dev, 
                                 irqreturn_t (*detect_int)(int, void *), 
                                 void *data)
{
    /* Reserve SD/MMC CS pin */
    vcoreiii_gpio_set_alternate(8, 1); /* SI_nEN1/GPIO_8 */
    return 0;
}

#define VTSS_SPI_MMC_CD (32+32+10)    // SGPIO p10.1 = GPIO 74, 0 => NO Card detect
#define VTSS_SPI_MMC_WP (32+ 0+10)    // SGPIO p10.0 = GPIO 42, 0 => RO is OFF
static struct mmc_spi_platform_data serval_mmc_spi_pdata = {
    .caps          = MMC_CAP_NEEDS_POLL,            /* No IRQ on SGPIO */
    .caps2         = MMC_CAP2_RO_ACTIVE_HIGH,
    .ocr_mask      = MMC_VDD_32_33 | MMC_VDD_33_34, /* default power */
    .detect_delay  = 100, /* msecs */
    .powerup_msecs = 100, /* msecs */
    .flags         = MMC_SPI_USE_CD_GPIO | MMC_SPI_USE_RO_GPIO,
    .cd_gpio	   = VTSS_SPI_MMC_CD,
    .ro_gpio       = VTSS_SPI_MMC_WP,
    .init          = vcoreiii_mmc_spi_init,
};
#endif


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

#if defined(CONFIG_MMC_SPI) || defined(CONFIG_MMC_SPI_MODULE)
	{
		.modalias = "mmc_spi",
		.max_speed_hz = 20000000,     /* max spi clock (SCK) speed in HZ */
		.bus_num = 0,
		.chip_select = 1,
		.platform_data = &serval_mmc_spi_pdata,
		.controller_data = NULL,
		.mode = SPI_MODE_0,
	},
#endif

};

static int __init vcoreiii_mtd_init(void)
{
	platform_device_register(&serval_spi);

	spi_register_board_info(serval_spi_board_info, ARRAY_SIZE(serval_spi_board_info));

	return 0;
}

module_init(vcoreiii_mtd_init)
