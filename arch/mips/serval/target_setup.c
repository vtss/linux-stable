/*
 *
 * VCore-III System initialization
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
#include <linux/irq.h>
#include <linux/ioport.h>	/* for struct resource */
#include <linux/gpio.h>

#include <asm/time.h>
#include <asm/idle.h>
#include <asm/reboot.h>

#include <asm/mach-serval/hardware.h>
#include <linux/delay.h>
#include <linux/export.h>

/* No other usable initialization hook than this ...  */
extern void (*late_time_init)(void);

void __iomem *map_base_switch;
void __iomem *map_base_cpu;

EXPORT_SYMBOL(map_base_switch);
EXPORT_SYMBOL(map_base_cpu);

static struct resource vcoreiii_reserved_resources[] = {
        {
		.name   = "Switch registers",
		.start  = VTSS_IO_ORIGIN1_OFFSET,
		.end    = VTSS_IO_ORIGIN1_OFFSET + VTSS_IO_ORIGIN1_SIZE - 1,
		.flags  = IORESOURCE_MEM
        },
        {
		.name   = "CPU Registers",
		.start  = VTSS_IO_ORIGIN2_OFFSET,
		.end    = VTSS_IO_ORIGIN2_OFFSET + VTSS_IO_ORIGIN2_SIZE - 1,
		.flags  = IORESOURCE_MEM
        },
};

unsigned int __cpuinit get_c0_compare_int(void)
{
	return TIMER_IRQ;
}

void __init plat_time_init(void)
{
	mips_hpt_frequency = VCOREIII_AHB_CLOCK;
}

const char *get_system_type(void)
{
	return "Vitesse VCore-III Serval";
}

static void vcoreiii_machine_restart(char *command)
{
        writel(VTSS_F_DEVCPU_GCB_CHIP_REGS_SOFT_RST_SOFT_CHIP_RST,
               VTSS_DEVCPU_GCB_CHIP_REGS_SOFT_RST);

        while (1)
                cpu_wait();
}

u32 vcoreiii_reg_to_phys(void __iomem *addr)
{
    u32 offset;
    if(addr > map_base_switch && (offset = (addr - map_base_switch)) < VTSS_IO_ORIGIN1_SIZE) {
        return VTSS_IO_ORIGIN1_OFFSET + offset;
    } else if (addr > map_base_cpu && (offset = (addr - map_base_cpu)) < VTSS_IO_ORIGIN2_SIZE) {
        return VTSS_IO_ORIGIN2_OFFSET + offset;
    } else {
        printk("Invalid register address, %p reg1 %p reg2 %p\n",
               addr, map_base_switch, map_base_cpu);
        BUG();
        return -1;
    }
}
EXPORT_SYMBOL(vcoreiii_reg_to_phys);


/* Not only time init but that's what the hook it's called through is named */
static void __init vcoreiii_late_time_init(void)
{
	extern void __init vcoreiii_irq_init(void);
        extern void __init srvl_gpio_init(void);

	map_base_switch = ioremap(vcoreiii_reserved_resources[0].start,
				  vcoreiii_reserved_resources[0].end -
				  vcoreiii_reserved_resources[0].start + 1);
	if(!map_base_switch)
                printk(KERN_ERR "Unable to ioremap switch register space\n");

	map_base_cpu = ioremap(vcoreiii_reserved_resources[1].start,
			       vcoreiii_reserved_resources[1].end -
			       vcoreiii_reserved_resources[1].start + 1);	
	if(!map_base_cpu)
                printk(KERN_ERR "Unable to ioremap cpu register space\n");

	/* Hook chained IRQs */
	vcoreiii_irq_init();

        /* Register GPIO's */
        srvl_gpio_init();

        /* Standard GPIO's */
        vcoreiii_gpio_set_alternate(22, 0); /* VCORE_CFG0-3 */
        vcoreiii_gpio_set_alternate(23, 0);
        vcoreiii_gpio_set_alternate(24, 0);
        vcoreiii_gpio_set_alternate(25, 0);
        gpio_request(22, "cfg0");
        gpio_request(23, "cfg1");
        gpio_request(24, "cfg2");
        gpio_request(25, "cfg3");
        /* Standard SGPIO's */
#if 0
        gpio_request(32+ 0+ 7, "pushbutton");
        gpio_request(32+ 0+10, "sd_wp");
        gpio_request(32+32+10, "sd_cd");
#endif
}

void __init plat_mem_setup(void)
{
	int i;

	/* Callbacks for halt, restart */
	_machine_restart = vcoreiii_machine_restart;
	_machine_halt    = NULL;

	/* Set up initialization hooks */
	late_time_init = vcoreiii_late_time_init;

        for (i = 0; i < ARRAY_SIZE(vcoreiii_reserved_resources); i++)
                request_resource(&ioport_resource, vcoreiii_reserved_resources + i);
}

