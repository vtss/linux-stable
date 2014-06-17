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

#include <asm/mach-vcoreiii/hardware.h>

static struct spi_vcoreiii_platform_data spi_vcoreiii_cfg = {
	// .no_spi_delay = 1,
};

static struct platform_device vcoreiii_spi = {
        .name             = SPI_VCOREIII_PLATDEV_NAME,
        .id               = 0,
        .dev = {
                .platform_data = &spi_vcoreiii_cfg,
        },
};

static struct mtd_partition vcoreiii_spi_flash_partitions[] = {
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
		.size =		0x00500000,
	}
};

static struct flash_platform_data vcoreiii_spi_flash_data = {
	.type = "m25p128",
	.name = "spi_flash",
	.parts = vcoreiii_spi_flash_partitions,
	.nr_parts = ARRAY_SIZE(vcoreiii_spi_flash_partitions),
        .read_mapped = 1,
        .phys_offset = 0x40000000,
};

static struct spi_board_info vcoreiii_spi_board_info[] __initdata = {
	{
		/* the modalias must be the same as spi device driver name */
		.modalias = "m25p80", /* Name of spi_driver for this device */
		.max_speed_hz = 50000000,     /* max spi clock (SCK) speed in HZ */
		.bus_num = 0, /* Framework bus number */
		.chip_select = 0, /* Framework chip select. */
		.platform_data = &vcoreiii_spi_flash_data,
		.mode = SPI_MODE_0, /* CPOL=0, CPHA=0 */
	},
};

#if defined(CONFIG_VTSS_VCOREIII_JAGUAR)
#define NAND_ADDR_BIT_ALE (1 << 13)
#define NAND_ADDR_BIT_CLE (1 << 14)
#else
#define NAND_ADDR_BIT_ALE (1 << 2)
#define NAND_ADDR_BIT_CLE (1 << 3)
#endif

/* hardware specific access to control-lines */
static void vcoreiii_nand_cmd_ctl(struct mtd_info *mtd, int dat,
				  unsigned int ctrl)
{
	struct nand_chip *this = mtd->priv;
	int offset = (int)this->priv;

#if 0
	if(ctrl & NAND_CLE) {
		printk("cmd 0x%02x %s %s\n", 
		       dat, 
		       ctrl & NAND_CLE ? "CLE" : "---",
		       ctrl & NAND_ALE ? "ALE" : "---");
	}
#endif

	if (ctrl & NAND_CTRL_CHANGE) {
		offset = 0;
		if(ctrl & NAND_CLE) offset |= NAND_ADDR_BIT_CLE;
		if(ctrl & NAND_ALE) offset |= NAND_ADDR_BIT_ALE;
		this->priv = (void *)offset;
	}
	if (dat != NAND_CMD_NONE) {
		//printk("ctrl: writeb(0x%02x, %p) - offset 0x%x base %p\n", dat & 0xff, this->IO_ADDR_W + offset, offset, this->IO_ADDR_W);
		writeb(dat, this->IO_ADDR_W + offset);
	}
}

static struct mtd_partition vcoreiii_partition_info[] = {
	[0] = {
		.name	= "rootfs_data",
		.offset	= 0,
		.size	= MTDPART_SIZ_FULL,
	},
};

struct platform_nand_data vcoreiii_nand_platdata = {
	.chip = {
		.nr_chips = 1,
		.chip_offset = 0,
		.nr_partitions = ARRAY_SIZE(vcoreiii_partition_info),
		.partitions = vcoreiii_partition_info,
		.chip_delay = 50,
	},
	.ctrl = {
		.cmd_ctrl = vcoreiii_nand_cmd_ctl,
	},
};

#ifdef CONFIG_VTSS_VCOREIII_JAGUAR
#define VCOREIII_NAND_CS	1 /* CS1 */
#else
#define VCOREIII_NAND_CS	0 /* CS0 */
#endif

static struct resource vcoreiii_nand_resource[] = {
    [0] = {
        .start = 0x50000000 + (VCOREIII_NAND_CS*0x4000000), /* CS1 */
        .end   = 0x50000000 + (VCOREIII_NAND_CS*0x4000000) + NAND_ADDR_BIT_CLE*2,
        .flags = IORESOURCE_MEM,
    },
};

static struct platform_device vcoreiii_nand = {
	.name		= "gen_nand",
	.num_resources	= ARRAY_SIZE(vcoreiii_nand_resource),
	.resource	= vcoreiii_nand_resource,
	.id		= -1,
	.dev		= {
		.platform_data = &vcoreiii_nand_platdata,
	}
};

static int __init vcoreiii_mtd_init(void)
{

#if defined(CONFIG_VTSS_VCOREIII_JAGUAR)
	vcoreiii_gpio_set_alternate(19, 1);
#endif

        /* Slow down NAND CS via waitcc - appx 77ns */
	vcoreiii_io_mask_set(VTSS_ICPU_CFG_PI_MST_PI_MST_CTRL(VCOREIII_NAND_CS),
                             VTSS_M_ICPU_CFG_PI_MST_PI_MST_CTRL_WAITCC,
                             VTSS_F_ICPU_CFG_PI_MST_PI_MST_CTRL_WAITCC(8));

	platform_device_register(&vcoreiii_spi);
	platform_device_register(&vcoreiii_nand);

	spi_register_board_info(vcoreiii_spi_board_info, ARRAY_SIZE(vcoreiii_spi_board_info));

	return 0;
}

module_init(vcoreiii_mtd_init)
