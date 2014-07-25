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

static DEFINE_SPINLOCK(srvl_gpio_lock);

static int srvl_gpio_get(struct gpio_chip *chip, unsigned int offset)
{
    if (offset < 32) {
        return !!(readl(VTSS_DEVCPU_GCB_GPIO_GPIO_IN) & (1 << offset));
    } else {
        return !!(readl(VTSS_DEVCPU_GCB_GPIO_GPIO_IN1) & (1 << (offset - 32)));
    }
}

static void srvl_gpio_set(struct gpio_chip *chip, unsigned int offset, int value)
{
    unsigned long flags;
    spin_lock_irqsave(&srvl_gpio_lock, flags);
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
    spin_unlock_irqrestore(&srvl_gpio_lock, flags);
}

static int srvl_gpio_dir_in(struct gpio_chip *chip, unsigned int offset)
{
    spin_lock_irq(&srvl_gpio_lock);
    vcoreiii_io_clr(VTSS_DEVCPU_GCB_GPIO_GPIO_OE, 1 << offset);
    mmiowb();
    spin_unlock_irq(&srvl_gpio_lock);
    return 0;
}

static int srvl_gpio_dir_out(struct gpio_chip *chip, unsigned int offset, int value)
{
    spin_lock_irq(&srvl_gpio_lock);
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
    spin_unlock_irq(&srvl_gpio_lock);
    return 0;
}

static struct gpio_chip srvl_gpio_chip = {
    .get = srvl_gpio_get,
    .set = srvl_gpio_set,
    .direction_input = srvl_gpio_dir_in,
    .direction_output = srvl_gpio_dir_out,
    .label = "gpio",
    .base = 0,
    .ngpio = 64,
    .exported = 1,
};

int __init srvl_gpio_init(void)
{
    int rc;

    if((rc = gpiochip_add(&srvl_gpio_chip)) != 0)
        return rc;
    printk(KERN_WARNING "Jaguar2 registered %d GPIOs\n", srvl_gpio_chip.ngpio);

    return 0;
}

int gpio_to_irq(unsigned gpio) /* Dummy */
{
    return -1;
}
