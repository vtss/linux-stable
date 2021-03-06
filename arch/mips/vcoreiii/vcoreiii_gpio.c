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

#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

#include <asm/mach-vcoreiii/hardware.h>

struct proc_dir_entry *g_button_dir;

/* NB - this is really board specific ... */
#if defined(CONFIG_VTSS_VCOREIII_JAGUAR)
#define VCOREIII_BTN_DEFAULT	13
#elif defined(CONFIG_VTSS_VCOREIII_LUTON26)
#define VCOREIII_BTN_DEFAULT	12
#else
#error Please provide GPIO line number here!
#endif

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

#if defined(CONFIG_VTSS_VCOREIII_JAGUAR_DUAL)
void vcoreiii_gpio_set_alternate_slv(int gpio, int is_alternate)
{
    u32 mask = VTSS_BIT(gpio);
    if(is_alternate) {
        vcoreiii_io_set_slv(VTSS_DEVCPU_GCB_GPIO_GPIO_ALT(0), mask);
        vcoreiii_io_clr_slv(VTSS_DEVCPU_GCB_GPIO_GPIO_ALT(1), mask);
    } else {
        vcoreiii_io_clr_slv(VTSS_DEVCPU_GCB_GPIO_GPIO_ALT(0), mask);
        vcoreiii_io_clr_slv(VTSS_DEVCPU_GCB_GPIO_GPIO_ALT(1), mask);
    }
}
EXPORT_SYMBOL(vcoreiii_gpio_set_alternate_slv);

void vcoreiii_gpio_set_input_slv(int gpio, int is_input)
{
    vcoreiii_gpio_set_alternate_slv(gpio, 0);
    if(is_input)
        vcoreiii_io_clr_slv(VTSS_DEVCPU_GCB_GPIO_GPIO_OE, VTSS_BIT(gpio));
    else
        vcoreiii_io_set_slv(VTSS_DEVCPU_GCB_GPIO_GPIO_OE, VTSS_BIT(gpio));
}
EXPORT_SYMBOL(vcoreiii_gpio_set_input_slv);
#endif

static ssize_t
gpio_proc_read(struct file *file, char __user *buf,
               size_t count, loff_t *ppos)
{
    u32 status = readl(VTSS_DEVCPU_GCB_GPIO_GPIO_IN);
    unsigned char mybuf[2];
    mybuf[0] = (status & VTSS_BIT(VCOREIII_BTN_DEFAULT)) ? '1' : '0';
    mybuf[1] = '\n';
    return simple_read_from_buffer(mybuf, count, ppos, mybuf, sizeof(mybuf));
}

static const struct file_operations button_proc_fops = {
	.read		= gpio_proc_read,
	.llseek		= default_llseek,
};

static int __init vcoreiii_gpio_init(void)
{
    struct proc_dir_entry *entry;
    vcoreiii_gpio_set_input(VCOREIII_BTN_DEFAULT, 1);
    g_button_dir = proc_mkdir("button", NULL);
    entry = proc_create("default", S_IRUGO, g_button_dir, &button_proc_fops);
    if (entry) {
        proc_set_size(entry, 2);
    }
    return 0;
}

static void __exit vcoreiii_gpio_exit(void)
{
    remove_proc_entry("default", g_button_dir);
    remove_proc_entry("button", NULL);
}

module_init(vcoreiii_gpio_init);
module_exit(vcoreiii_gpio_exit);

MODULE_AUTHOR("Lars Povlsen <lpovlsen@vitesse.com>");
MODULE_DESCRIPTION("VCore-III GPIO button driver");
MODULE_LICENSE("GPL");
