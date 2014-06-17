/*
 * Bitbanging SPI bus driver for VCore-III
 *
 * Copyright (c) 2011 Lars Povlsen
 *
 * based on spi_gpio.c
 *  Copyright (c) 2008 Piotr Skamruk
 *  Copyright (c) 2008 Michael Buesch
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi_bitbang.h>
#include <linux/spi/spi_vcoreiii.h>
#include <asm/mach-vcoreiii/hardware.h>
#include <asm/atomic.h>

struct spi_vcoreiii {
	struct spi_bitbang bitbang;
	struct spi_vcoreiii_platform_data *info;
	u32    bb_cur;
};


static inline struct spi_vcoreiii *spidev_to_sg(struct spi_device *dev)
{
	return spi_master_get_devdata(dev->master);
}

static inline void setsck(struct spi_device *dev, int val)
{
	struct spi_vcoreiii *sp = spidev_to_sg(dev);
	if(val)
		sp->bb_cur |= VTSS_F_ICPU_CFG_SPI_MST_SW_MODE_SW_SPI_SCK;
	else
		sp->bb_cur &= ~VTSS_F_ICPU_CFG_SPI_MST_SW_MODE_SW_SPI_SCK;
	writel(sp->bb_cur, VTSS_ICPU_CFG_SPI_MST_SW_MODE);
}

static inline void setmosi(struct spi_device *dev, int val)
{
	struct spi_vcoreiii *sp = spidev_to_sg(dev);
	sp->bb_cur |= VTSS_F_ICPU_CFG_SPI_MST_SW_MODE_SW_SPI_SDO_OE;
	if(val)
		sp->bb_cur |= VTSS_F_ICPU_CFG_SPI_MST_SW_MODE_SW_SPI_SDO;
	else
		sp->bb_cur &= ~VTSS_F_ICPU_CFG_SPI_MST_SW_MODE_SW_SPI_SDO;
	writel(sp->bb_cur, VTSS_ICPU_CFG_SPI_MST_SW_MODE);
}

static inline u32 getmiso(struct spi_device *dev)
{
	return (readl(VTSS_ICPU_CFG_SPI_MST_SW_MODE) & VTSS_F_ICPU_CFG_SPI_MST_SW_MODE_SW_SPI_SDI) ? 1 : 0;
}

static inline void do_spidelay(struct spi_device *dev, unsigned nsecs)
{
	struct spi_vcoreiii *sp = spidev_to_sg(dev);
	if (!sp->info->no_spi_delay)
		ndelay(nsecs);
}

#define spidelay(nsecs) do {					\
	/* Steal the spi_device pointer from our caller.	\
	 * The bitbang-API should probably get fixed here... */	\
	do_spidelay(spi, nsecs);				\
  } while (0)

#include "spi-bitbang-txrx.h"

static u32 spi_vcoreiii_txrx_mode0(struct spi_device *spi,
			       unsigned nsecs, u32 word, u8 bits)
{
	return bitbang_txrx_be_cpha0(spi, nsecs, 0, 0, word, bits);
}

static u32 spi_vcoreiii_txrx_mode1(struct spi_device *spi,
			       unsigned nsecs, u32 word, u8 bits)
{
	return bitbang_txrx_be_cpha1(spi, nsecs, 0, 0, word, bits);
}

static u32 spi_vcoreiii_txrx_mode2(struct spi_device *spi,
			       unsigned nsecs, u32 word, u8 bits)
{
	return bitbang_txrx_be_cpha0(spi, nsecs, 1, 0, word, bits);
}

static u32 spi_vcoreiii_txrx_mode3(struct spi_device *spi,
			       unsigned nsecs, u32 word, u8 bits)
{
	return bitbang_txrx_be_cpha1(spi, nsecs, 1, 0, word, bits);
}

static void spi_vcoreiii_chipselect(struct spi_device *dev, int on)
{
	struct spi_vcoreiii *sp = spidev_to_sg(dev);
	if(on) {
		sp->bb_cur = 
			VTSS_F_ICPU_CFG_SPI_MST_SW_MODE_SW_SPI_SCK_OE | /* SCK_OE */
			((dev->mode & SPI_CPOL) ? VTSS_F_ICPU_CFG_SPI_MST_SW_MODE_SW_SPI_SCK : 0) | /* SCK */
			VTSS_F_ICPU_CFG_SPI_MST_SW_MODE_SW_PIN_CTRL_MODE ; /* SW Bitbang */
		/* Setup clock in right state before driving CS */
		writel(sp->bb_cur, VTSS_ICPU_CFG_SPI_MST_SW_MODE);
		/* Now enable CS */
		sp->bb_cur |=
			VTSS_F_ICPU_CFG_SPI_MST_SW_MODE_SW_SPI_CS_OE(VTSS_BIT(dev->chip_select)) | /* CS_OE */
			VTSS_F_ICPU_CFG_SPI_MST_SW_MODE_SW_SPI_CS(VTSS_BIT(dev->chip_select));
		writel(sp->bb_cur, VTSS_ICPU_CFG_SPI_MST_SW_MODE);
	} else {
		/* Drive CS low */
		sp->bb_cur &= ~VTSS_M_ICPU_CFG_SPI_MST_SW_MODE_SW_SPI_CS;
		writel(sp->bb_cur, VTSS_ICPU_CFG_SPI_MST_SW_MODE);
		/* Drop everything */
		sp->bb_cur = 0;
		writel(sp->bb_cur, VTSS_ICPU_CFG_SPI_MST_SW_MODE);
	}
}

static int spi_vcoreiii_probe(struct platform_device *pdev)
{
	struct spi_master *master;
	struct spi_vcoreiii *sp;
	int err;

	master = spi_alloc_master(&pdev->dev, sizeof(struct spi_vcoreiii));
	if (!master) {
		printk(KERN_ERR SPI_VCOREIII_PLATDEV_NAME ": Allocation error\n");
		return -ENOMEM;
	}

	sp = spi_master_get_devdata(master);
	platform_set_drvdata(pdev, sp);
	sp->info = pdev->dev.platform_data;

	sp->bitbang.master = spi_master_get(master);
	sp->bitbang.master->bus_num = 0;
	sp->bitbang.master->num_chipselect = 4;
	sp->bitbang.chipselect = spi_vcoreiii_chipselect;
	sp->bitbang.txrx_word[SPI_MODE_0] = spi_vcoreiii_txrx_mode0;
	sp->bitbang.txrx_word[SPI_MODE_1] = spi_vcoreiii_txrx_mode1;
	sp->bitbang.txrx_word[SPI_MODE_2] = spi_vcoreiii_txrx_mode2;
	sp->bitbang.txrx_word[SPI_MODE_3] = spi_vcoreiii_txrx_mode3;

	err = spi_bitbang_start(&sp->bitbang);
	if (err)
		goto err_no_bitbang;

	return 0;

err_no_bitbang:
	spi_master_put(sp->bitbang.master);
	kfree(master);
	printk(KERN_ERR SPI_VCOREIII_PLATDEV_NAME ": Error: %d\n", err);

	return err;
}

static int __exit spi_vcoreiii_remove(struct platform_device *pdev)
{
	struct spi_vcoreiii *sp;
	struct spi_vcoreiii_platform_data *pdata;

	pdata = pdev->dev.platform_data;
	sp = platform_get_drvdata(pdev);

	spi_bitbang_stop(&sp->bitbang);
	spi_master_put(sp->bitbang.master);

	return 0;
}

MODULE_ALIAS("platform:" SPI_VCOREIII_PLATDEV_NAME);
static struct platform_driver spi_vcoreiii_driver = {
	.driver		= {
		.name	= SPI_VCOREIII_PLATDEV_NAME,
		.owner	= THIS_MODULE,
	},
	.probe		= spi_vcoreiii_probe,
	.remove		= __exit_p(spi_vcoreiii_remove),
};

static int __init spi_vcoreiii_init(void)
{
	int err;
	err = platform_driver_register(&spi_vcoreiii_driver);
	if (err)
		printk(KERN_ERR SPI_VCOREIII_PLATDEV_NAME ": register failed: %d\n", err);
	return err;
}
module_init(spi_vcoreiii_init);

static void __exit spi_vcoreiii_exit(void)
{
	platform_driver_unregister(&spi_vcoreiii_driver);
}
module_exit(spi_vcoreiii_exit);

MODULE_AUTHOR("Lars Povlsen <lpovlsen at vitesse.com>");
MODULE_AUTHOR("Lars Povlsen");
MODULE_DESCRIPTION("VCore-III bitbanging SPI driver");
MODULE_LICENSE("GPL v2");
