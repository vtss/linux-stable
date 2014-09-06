/*
 *
 * VCore-III GPIO
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
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/gpio.h>
#include <linux/errno.h>
#include <linux/io.h>

#include <asm/mach-jaguar2/hardware.h>

void vcoreiii_gpio_set_alternate(int gpio, int is_alternate)
{
    if (gpio < 32) {
        u32 mask = VTSS_BIT(gpio);
        if(is_alternate) {
            vcoreiii_io_set(VTSS_DEVCPU_GCB_GPIO_GPIO_ALT(0), mask);
            vcoreiii_io_clr(VTSS_DEVCPU_GCB_GPIO_GPIO_ALT(1), mask);
        } else {
            vcoreiii_io_clr(VTSS_DEVCPU_GCB_GPIO_GPIO_ALT(0), mask);
            vcoreiii_io_clr(VTSS_DEVCPU_GCB_GPIO_GPIO_ALT(1), mask);
        }
    } else {
        u32 mask = VTSS_BIT(gpio - 32);
        if(is_alternate) {
            vcoreiii_io_set(VTSS_DEVCPU_GCB_GPIO_GPIO_ALT1(0), mask);
            vcoreiii_io_clr(VTSS_DEVCPU_GCB_GPIO_GPIO_ALT1(1), mask);
        } else {
            vcoreiii_io_clr(VTSS_DEVCPU_GCB_GPIO_GPIO_ALT1(0), mask);
            vcoreiii_io_clr(VTSS_DEVCPU_GCB_GPIO_GPIO_ALT1(1), mask);
        }
    }
}
EXPORT_SYMBOL(vcoreiii_gpio_set_alternate);

void vcoreiii_gpio_set_input(int gpio, int is_input)
{
    vcoreiii_gpio_set_alternate(gpio, 0);
    if (gpio < 32) {
        u32 mask = VTSS_BIT(gpio);
        if(is_input)
            vcoreiii_io_clr(VTSS_DEVCPU_GCB_GPIO_GPIO_OE, mask);
        else
            vcoreiii_io_set(VTSS_DEVCPU_GCB_GPIO_GPIO_OE, mask);
    } else {
        u32 mask = VTSS_BIT(gpio - 32);
        if(is_input)
            vcoreiii_io_clr(VTSS_DEVCPU_GCB_GPIO_GPIO_OE1, mask);
        else
            vcoreiii_io_set(VTSS_DEVCPU_GCB_GPIO_GPIO_OE1, mask);
    }
}
EXPORT_SYMBOL(vcoreiii_gpio_set_input);

static DEFINE_SPINLOCK(jag2_gpio_lock);

static int jag2_gpio_get(struct gpio_chip *chip, unsigned int offset)
{
    if (offset < 32) {
        return !!(readl(VTSS_DEVCPU_GCB_GPIO_GPIO_IN) & (1 << offset));
    } else {
        return !!(readl(VTSS_DEVCPU_GCB_GPIO_GPIO_IN1) & (1 << (offset - 32)));
    }
}

static void jag2_gpio_set(struct gpio_chip *chip, unsigned int offset, int value)
{
    unsigned long flags;
    spin_lock_irqsave(&jag2_gpio_lock, flags);
    if (offset < 32) {
        u32 mask = VTSS_BIT(offset);
        if(value)
            vcoreiii_io_set(VTSS_DEVCPU_GCB_GPIO_GPIO_OUT, mask);
        else
            vcoreiii_io_clr(VTSS_DEVCPU_GCB_GPIO_GPIO_OUT, mask);
    } else {
        u32 mask = VTSS_BIT(offset - 32);
        if(value)
            vcoreiii_io_set(VTSS_DEVCPU_GCB_GPIO_GPIO_OUT1, mask);
        else
            vcoreiii_io_clr(VTSS_DEVCPU_GCB_GPIO_GPIO_OUT1, mask);
    }
    mmiowb();
    spin_unlock_irqrestore(&jag2_gpio_lock, flags);
}

static int jag2_gpio_dir_in(struct gpio_chip *chip, unsigned int offset)
{
    spin_lock_irq(&jag2_gpio_lock);
    vcoreiii_io_clr(VTSS_DEVCPU_GCB_GPIO_GPIO_OE, 1 << offset);
    mmiowb();
    spin_unlock_irq(&jag2_gpio_lock);
    return 0;
}

static int jag2_gpio_dir_out(struct gpio_chip *chip, unsigned int offset, int value)
{
    spin_lock_irq(&jag2_gpio_lock);
    if (offset < 32) {
        u32 mask = VTSS_BIT(offset);
        if(value)
            vcoreiii_io_set(VTSS_DEVCPU_GCB_GPIO_GPIO_OUT, mask);
        else
            vcoreiii_io_clr(VTSS_DEVCPU_GCB_GPIO_GPIO_OUT, mask);
        vcoreiii_io_set(VTSS_DEVCPU_GCB_GPIO_GPIO_OE, mask);
    } else {
        u32 mask = VTSS_BIT(offset - 32);
        if(value)
            vcoreiii_io_set(VTSS_DEVCPU_GCB_GPIO_GPIO_OUT1, mask);
        else
            vcoreiii_io_clr(VTSS_DEVCPU_GCB_GPIO_GPIO_OUT1, mask);
        vcoreiii_io_set(VTSS_DEVCPU_GCB_GPIO_GPIO_OE1, mask);
    }
    mmiowb();
    spin_unlock_irq(&jag2_gpio_lock);
    return 0;
}

static struct gpio_chip jag2_gpio_chip = {
    .get = jag2_gpio_get,
    .set = jag2_gpio_set,
    .direction_input = jag2_gpio_dir_in,
    .direction_output = jag2_gpio_dir_out,
    .label = "gpio",
    .base = 0,
    .ngpio = 64,
    .exported = 1,
};

static int jag2_sgpio_get(struct gpio_chip *chip, unsigned int offset)
{
     int ix = offset / 32;
     offset %= 32;
     return !!(readl(VTSS_DEVCPU_GCB_SIO_CTRL_SIO_INPUT_DATA(ix,offset)));
}

static void jag2_sgpio_set(struct gpio_chip *chip, unsigned int offset, int value)
{
}

static int jag2_sgpio_dir_in(struct gpio_chip *chip, unsigned int offset)
{
    return 0;
}

static int jag2_sgpio_dir_out(struct gpio_chip *chip, unsigned int offset, int value)
{
    return -EFAULT;
}

static struct gpio_chip jag2_sgpio_chip = {
    .get = jag2_sgpio_get,
    .set = jag2_sgpio_set,
    .direction_input = jag2_sgpio_dir_in,
    .direction_output = jag2_sgpio_dir_out,
    .label = "sgpio",
    .base = 64,
    .ngpio = 4*32,             /* Bitwidth 4 */
    .exported = 1,
};

int __init jag2_gpio_init(void)
{
    int rc;
    int bit_count = 2;    /* ... */
    int port;

    if((rc = gpiochip_add(&jag2_gpio_chip)) != 0)
        return rc;
    printk(KERN_WARNING "Jaguar2 registered %d GPIOs\n", jag2_gpio_chip.ngpio);

    /* Intialize SGPIO */
    writel(0xFFF0FFFF, VTSS_DEVCPU_GCB_SIO_CTRL_SIO_PORT_ENA(0)); /* Enable [31:24] and [15:0] */
    writel(VTSS_F_DEVCPU_GCB_SIO_CTRL_SIO_CFG_SIO_BMODE_0(2) |
	   VTSS_F_DEVCPU_GCB_SIO_CTRL_SIO_CFG_SIO_BMODE_1(1) |
	   VTSS_F_DEVCPU_GCB_SIO_CTRL_SIO_CFG_SIO_BURST_GAP(0x1F) |
	   VTSS_F_DEVCPU_GCB_SIO_CTRL_SIO_CFG_SIO_PORT_WIDTH(bit_count - 1) |
	   VTSS_M_DEVCPU_GCB_SIO_CTRL_SIO_CFG_SIO_AUTO_REPEAT, 
	   VTSS_DEVCPU_GCB_SIO_CTRL_SIO_CFG(0));
	   
    /* Setup the serial IO clock frequency - 12.5MHz (0x14) */
    writel(0x14, VTSS_DEVCPU_GCB_SIO_CTRL_SIO_CLOCK(0));

    /* Reset all SGPIO ports */
    for (port = 0; port < 32; port++)
        writel(0, VTSS_DEVCPU_GCB_SIO_CTRL_SIO_PORT_CFG(0, port));

    if((rc = gpiochip_add(&jag2_sgpio_chip)) != 0)
	    return rc;
    printk(KERN_WARNING "Jaguar2 registered %d SGPIOs\n", jag2_sgpio_chip.ngpio);

    return 0;
}

int gpio_to_irq(unsigned gpio) /* Dummy */
{
    return -1;
}
