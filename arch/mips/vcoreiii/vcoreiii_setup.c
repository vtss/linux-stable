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

#include <asm/time.h>
#include <asm/reboot.h>

#include <asm/mach-vcoreiii/hardware.h>
#include <linux/delay.h>
#include <linux/export.h>

/* No other usable initialization hook than this ...  */
extern void (*late_time_init)(void);

void __iomem *map_base_switch;
void __iomem *map_base_cpu;
void __iomem *map_base_slv_switch;

EXPORT_SYMBOL(map_base_switch);
EXPORT_SYMBOL(map_base_cpu);
EXPORT_SYMBOL(map_base_slv_switch);

#define SLAVE_CS 3              /* Slave switch on CS3 by default */

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
#if defined(CONFIG_VTSS_VCOREIII_JAGUAR_DUAL)
        {
		.name   = "Switch registers - Slave unit",
		.start  = VTSS_IO_PI_REGION(SLAVE_CS), /* CS3 */
		.end    = VTSS_IO_PI_REGION(SLAVE_CS) + VTSS_IO_ORIGIN1_SIZE - 1,
		.flags  = IORESOURCE_MEM
        },
#undef VTSS_IO_PI_REGION
#endif  /* CONFIG_VTSS_VCOREIII_JAGUAR_DUAL */
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
	return "Vitesse VCore-III";
}

static void vcoreiii_machine_restart(char *command)
{
#if 0
	/* This is a *CPU*-only reset */
	writel(VTSS_F_ICPU_CFG_CPU_SYSTEM_CTRL_RESET_CORE_RST_FORCE,
	       VTSS_ICPU_CFG_CPU_SYSTEM_CTRL_RESET);
#endif
        writel(VTSS_F_DEVCPU_GCB_DEVCPU_RST_REGS_SOFT_CHIP_RST_SOFT_CHIP_RST,
               VTSS_DEVCPU_GCB_DEVCPU_RST_REGS_SOFT_CHIP_RST);

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

#if defined(CONFIG_VTSS_VCOREIII_JAGUAR_DUAL)
static volatile void __iomem *master2slave_addr(volatile void __iomem *addr)
{
    volatile void __iomem *slvaddr = (volatile void __iomem *)
        (((u32)addr) - ((u32)map_base_switch) + ((u32)map_base_slv_switch));
    return slvaddr;
}

static void slv_direct_writel(u32 val, volatile void __iomem *addr)
{
    //printk("slv_write(%p) = 0x%0x\n", addr, val);
    writel(val, master2slave_addr(addr));
}

static u32 slv_direct_readl(volatile void __iomem *addr)
{
    u32 r = readl(master2slave_addr(addr));
    //printk("slv_read(%p) = 0x%0x\n", addr, r);
    return r;
}

static int slv_region1_accessible(u32 addr)
{
    return
        (addr > (u32) map_base_switch) &&
        (addr < (((u32)map_base_switch) + VTSS_IO_ORIGIN1_SIZE));
}

static u32 slv_region2_offset(volatile u32 __iomem *addr)
{
    return 0x70000000 + (((u32)addr) - ((u32)map_base_cpu));
}

static u32 slv_indirect_readl(volatile u32 __iomem *addr)
{
    u32 val, ctl;
    slv_direct_writel(slv_region2_offset(addr), VTSS_DEVCPU_GCB_VCORE_ACCESS_VA_ADDR);
    val = slv_direct_readl(VTSS_DEVCPU_GCB_VCORE_ACCESS_VA_DATA);
    do {
        ctl = slv_direct_readl(VTSS_DEVCPU_GCB_VCORE_ACCESS_VA_CTRL);
    } while(ctl & VTSS_F_DEVCPU_GCB_VCORE_ACCESS_VA_CTRL_VA_BUSY);
    if(ctl & VTSS_F_DEVCPU_GCB_VCORE_ACCESS_VA_CTRL_VA_ERR) {
        printk(KERN_ERR ": Slave Read error on address %p, ctl = 0x%08x\n", addr, ctl);
    }
    val = slv_direct_readl(VTSS_DEVCPU_GCB_VCORE_ACCESS_VA_DATA_INERT);
    //printk("Rd(%p) = 0x%08x\n", addr, val);
    return val;
}

static void slv_indirect_writel(u32 val, volatile u32 __iomem *addr)
{
    u32 ctl;
    //printk("Wr(%p) = 0x%08x\n", addr, val);
    slv_direct_writel(slv_region2_offset(addr), VTSS_DEVCPU_GCB_VCORE_ACCESS_VA_ADDR);
    slv_direct_writel(val, VTSS_DEVCPU_GCB_VCORE_ACCESS_VA_DATA);
    do {
        ctl = slv_direct_readl(VTSS_DEVCPU_GCB_VCORE_ACCESS_VA_CTRL);
    } while(ctl & VTSS_F_DEVCPU_GCB_VCORE_ACCESS_VA_CTRL_VA_BUSY);
    if(ctl & VTSS_F_DEVCPU_GCB_VCORE_ACCESS_VA_CTRL_VA_ERR) {
        printk(KERN_ERR "Write error on address %p, ctl = 0x%08x\n", addr, ctl);
    }
}

void slv_writel(u32 val, volatile u32 __iomem *addr)
{
    if(slv_region1_accessible((u32)addr))
        slv_direct_writel(val, addr);
    else
        slv_indirect_writel(val, addr);
}
EXPORT_SYMBOL(slv_writel);

u32 slv_readl(volatile u32 __iomem *addr)
{
    if(slv_region1_accessible((u32)addr))
        return slv_direct_readl(addr);
    return slv_indirect_readl(addr);
}
EXPORT_SYMBOL(slv_readl);
#endif	/* CONFIG_VTSS_VCOREIII_JAGUAR_DUAL */

/* Not only time init but that's what the hook it's called through is named */
static void __init vcoreiii_late_time_init(void)
{
	extern void __init vcoreiii_irq_init(void);

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

#if defined(CONFIG_VTSS_VCOREIII_JAGUAR_DUAL)
	map_base_slv_switch = ioremap(vcoreiii_reserved_resources[2].start,
                                      vcoreiii_reserved_resources[2].end -
                                      vcoreiii_reserved_resources[2].start + 1);
	if(!map_base_slv_switch)
            printk(KERN_ERR "Unable to ioremap slave switch register space\n");

        /* GPIO 18 + cs is used for Slave switch */
	vcoreiii_gpio_set_alternate(18 + SLAVE_CS, 1);

        /* Configure Slow PI */
	vcoreiii_io_mask_set(VTSS_ICPU_CFG_PI_MST_PI_MST_CFG, 
                             VTSS_M_ICPU_CFG_PI_MST_PI_MST_CFG_CLK_DIV,
                             VTSS_F_ICPU_CFG_PI_MST_PI_MST_CFG_CLK_DIV(0x1f));

        /* Enable the PI interface */
	vcoreiii_io_set(VTSS_ICPU_CFG_CPU_SYSTEM_CTRL_GENERAL_CTRL,  
			VTSS_F_ICPU_CFG_CPU_SYSTEM_CTRL_GENERAL_CTRL_IF_MASTER_PI_ENA);

        /* Reset slave device */
        slv_writel(VTSS_F_DEVCPU_GCB_DEVCPU_RST_REGS_SOFT_CHIP_RST_SOFT_CHIP_RST,
                   VTSS_DEVCPU_GCB_DEVCPU_RST_REGS_SOFT_CHIP_RST);

        /* Small nap during reset */
        udelay(10);

        /* Initialize slave device for '16-bit ndone' mode (again) */
        slv_writel(0x18181818, VTSS_DEVCPU_PI_PI_PI_MODE);
        slv_writel(0x18181818, VTSS_DEVCPU_PI_PI_PI_MODE);

	vcoreiii_io_set(VTSS_ICPU_CFG_PI_MST_PI_MST_CTRL(SLAVE_CS), 0x00C200B3); /* Slv CS settings */

#endif	/* CONFIG_VTSS_VCOREIII_JAGUAR_DUAL */

	/* Enable the PI interface */
	vcoreiii_io_set(VTSS_ICPU_CFG_CPU_SYSTEM_CTRL_GENERAL_CTRL,  
			VTSS_F_ICPU_CFG_CPU_SYSTEM_CTRL_GENERAL_CTRL_IF_MASTER_PI_ENA);

        /* Configure Fast PI */
	vcoreiii_io_mask_set(VTSS_ICPU_CFG_PI_MST_PI_MST_CFG, 
                             VTSS_M_ICPU_CFG_PI_MST_PI_MST_CFG_CLK_DIV,
                             VTSS_F_ICPU_CFG_PI_MST_PI_MST_CFG_CLK_DIV(4));

	/* Hook chained IRQs */
	vcoreiii_irq_init();
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

