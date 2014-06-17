/*
 *
 * Hardware access header file.
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

#ifndef __VCOREIII_HARDWARE_H
#define __VCOREIII_HARDWARE_H

/* We use virtual memory to map the registers */
#define VTSS_IO_OFFSET1(offset) (map_base_switch + offset)
#define VTSS_IO_OFFSET2(offset) (map_base_cpu + offset)

/* We just want the addresses from VTSS_IOREG() */
#define VTSS_IOREG(t,o)	((void __iomem *)VTSS_IOADDR(t,o))

#define VTSS_IO_PI_REGION(x) (0x50000000 + (0x4000000*x))

#if defined(CONFIG_VTSS_VCOREIII_JAGUAR)
#include <vtss_jaguar_core_regs.h>
#define VTSS_DEVCPU_PI_PI_PI_MODE            VTSS_IOREG(VTSS_TO_DEVCPU_PI,0x3)
#elif defined(CONFIG_VTSS_VCOREIII_LUTON26)
#include <vtss_luton26_core_regs.h>
#else
#error Invalid sub-architecture type
#endif

#define VCOREIII_CPU_CLOCK 416666666              /* 416MHz */
#define VCOREIII_AHB_CLOCK (VCOREIII_CPU_CLOCK/2) /* 208MHz - half of the CPU freq */

#ifndef __ASSEMBLER__

extern void __iomem *map_base_switch;
extern void __iomem *map_base_cpu;
#if defined(CONFIG_VTSS_VCOREIII_JAGUAR_DUAL)
extern void __iomem *map_base_slv_switch;
#endif  /* CONFIG_VTSS_VCOREIII_JAGUAR_DUAL */

static inline void vcoreiii_io_clr(volatile void __iomem *addr, u32 mask)
{
	writel(readl(addr) & ~mask, addr);
}

static inline void vcoreiii_io_set(volatile void __iomem *addr, u32 mask)
{
	writel(readl(addr) | mask, addr);
}

static inline void vcoreiii_io_mask_set(volatile void __iomem *addr, u32 clr_mask, u32 set_mask)
{
    u32 val = readl(addr);
    val &= ~clr_mask;
    val |= set_mask;
    writel(val, addr);
}

#if defined(CONFIG_VTSS_VCOREIII_JAGUAR_DUAL)
void slv_writel(u32 val, volatile u32 __iomem *addr);
u32  slv_readl(volatile u32 __iomem *addr);

static inline void vcoreiii_io_clr_slv(volatile void __iomem *addr, u32 mask)
{
    slv_writel(slv_readl(addr) & ~mask, addr);
}

static inline void vcoreiii_io_set_slv(volatile void __iomem *addr, u32 mask)
{
    slv_writel(slv_readl(addr) | mask, addr);
}

static inline void vcoreiii_io_mask_set_slv(volatile void __iomem *addr, u32 clr_mask, u32 set_mask)
{
    u32 val = slv_readl(addr);
    val &= ~clr_mask;
    val |= set_mask;
    slv_writel(val, addr);
}

#endif	/* CONFIG_VTSS_VCOREIII_JAGUAR_DUAL */

u32 vcoreiii_reg_to_phys(void __iomem *addr);

void vcoreiii_gpio_set_alternate(int gpio, int is_alternate);
void vcoreiii_gpio_set_input(int gpio, int is_input);
#if defined(CONFIG_VTSS_VCOREIII_JAGUAR_DUAL)
void vcoreiii_gpio_set_alternate_slv(int gpio, int is_alternate);
void vcoreiii_gpio_set_input_slv(int gpio, int is_input);
#endif

#endif // __ASSEMBLER__

#endif /* __VCOREIII_HARDWARE_H */
