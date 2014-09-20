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

#include <asm/mach-serval/hardware.h>

void vcoreiii_gpio_set_alternate(int gpio, int is_alternate)
{
    u32 mask = VTSS_BIT(gpio);
    if(is_alternate) {
        vcoreiii_io_set(VTSS_DEVCPU_GCB_GPIO_GPIO_ALT(0), mask);
        vcoreiii_io_clr(VTSS_DEVCPU_GCB_GPIO_GPIO_ALT(1), mask);
    } else {
        vcoreiii_io_clr(VTSS_DEVCPU_GCB_GPIO_GPIO_ALT(0), mask);
        vcoreiii_io_clr(VTSS_DEVCPU_GCB_GPIO_GPIO_ALT(1), mask);
    }
}
EXPORT_SYMBOL(vcoreiii_gpio_set_alternate);

void vcoreiii_gpio_set_input(int gpio, int is_input)
{
    vcoreiii_gpio_set_alternate(gpio, 0);
    if(is_input)
        vcoreiii_io_clr(VTSS_DEVCPU_GCB_GPIO_GPIO_OE, VTSS_BIT(gpio));
    else
        vcoreiii_io_set(VTSS_DEVCPU_GCB_GPIO_GPIO_OE, VTSS_BIT(gpio));
}
EXPORT_SYMBOL(vcoreiii_gpio_set_input);

static DEFINE_SPINLOCK(srvl_gpio_lock);

static int srvl_gpio_get(struct gpio_chip *chip, unsigned int offset)
{
    return !!(readl(VTSS_DEVCPU_GCB_GPIO_GPIO_IN) & (1 << offset));
}

static void srvl_gpio_set(struct gpio_chip *chip, unsigned int offset, int value)
{
    unsigned long flags;
    spin_lock_irqsave(&srvl_gpio_lock, flags);
    if(value)
        vcoreiii_io_set(VTSS_DEVCPU_GCB_GPIO_GPIO_OUT, 1 << offset);
    else
        vcoreiii_io_clr(VTSS_DEVCPU_GCB_GPIO_GPIO_OUT, 1 << offset);
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
    if(value)
        vcoreiii_io_set(VTSS_DEVCPU_GCB_GPIO_GPIO_OUT, 1 << offset);
    else
        vcoreiii_io_clr(VTSS_DEVCPU_GCB_GPIO_GPIO_OUT, 1 << offset);
    vcoreiii_io_set(VTSS_DEVCPU_GCB_GPIO_GPIO_OE, 1 << offset);
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
    .ngpio = 32,
};

static int srvl_sgpio_get(struct gpio_chip *chip, unsigned int offset)
{
     int ix   = offset / 32;
     int port = offset % 32;
     return !!(readl(VTSS_DEVCPU_GCB_SIO_CTRL_SIO_INPUT_DATA(ix)) & (1 << port));
}

static void srvl_sgpio_set(struct gpio_chip *chip, unsigned int offset, int value)
{
     int ix   = offset / 32;
     int port = offset % 32;
     u32 val  = readl(VTSS_DEVCPU_GCB_SIO_CTRL_SIO_PORT_CONFIG(port));
     u32 mask = (0x7 << (ix*3));
     val = (val & ~mask) | (value << (ix*3));
     writel(val, VTSS_DEVCPU_GCB_SIO_CTRL_SIO_PORT_CONFIG(port));
}

static int srvl_sgpio_dir_in(struct gpio_chip *chip, unsigned int offset)
{
    return 0;
}

static int srvl_sgpio_dir_out(struct gpio_chip *chip, unsigned int offset, int value)
{
    return 0;
}

static struct gpio_chip srvl_sgpio_chip = {
    .get = srvl_sgpio_get,
    .set = srvl_sgpio_set,
    .direction_input = srvl_sgpio_dir_in,
    .direction_output = srvl_sgpio_dir_out,
    .label = "sgpio",
    .base = 32,
    .ngpio = 32+32,
};

int __init srvl_gpio_init(void)
{
    int rc;
    //int bit_count = (srvl_sgpio_chip.ngpio / 32); /* Serval1ref */
    //int port;

    if((rc = gpiochip_add(&srvl_gpio_chip)) != 0)
        return rc;
    printk(KERN_WARNING "Serval registered %d GPIOs\n", srvl_gpio_chip.ngpio);

    if((rc = gpiochip_add(&srvl_sgpio_chip)) != 0)
	    return rc;
    printk(KERN_WARNING "Serval registered %d SGPIOs\n", srvl_sgpio_chip.ngpio);
    return 0;
}

int gpio_to_irq(unsigned gpio) /* Dummy */
{
    return -1;
}

static int __init vcoreiii_gpio_reserve(void)
{
    /* Standard GPIO's */
    vcoreiii_gpio_set_alternate(22, 0); /* VCORE_CFG0-3 */
    vcoreiii_gpio_set_alternate(23, 0);
    vcoreiii_gpio_set_alternate(24, 0);
    vcoreiii_gpio_set_alternate(25, 0);
    /* Standard SGPIO's */
    //gpio_request(22, "cfg0");
    //gpio_request(23, "cfg1");
    //gpio_request(24, "cfg2");
    //gpio_request(25, "cfg3");
    (void) gpio_request_one(32+ 0+ 7, GPIOF_DIR_IN|GPIOF_EXPORT, "sgpio_pushbutton");
    return 0;
}
late_initcall(vcoreiii_gpio_reserve);
