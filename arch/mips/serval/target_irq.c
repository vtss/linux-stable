/*
 *
 * VCore-III IRQ Handling
 *
 * Copyright (C) 2010 Vitesse Semiconductor Inc.
 * Author: Lars Povlsen (lpovlsen@vitesse.com)
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
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
#include <linux/interrupt.h>

#include <asm/irq_cpu.h>
#include <asm/irq_regs.h>
#include <asm/mach-serval/irq.h>
#include <asm/mach-serval/hardware.h>

static void ack_irq_ioc(struct irq_data *data)
{
    writel(1 << (data->irq - ICPU_IRQ0_BASE), VTSS_ICPU_CFG_INTR_INTR_STICKY); /* Ack sticky */
}

static void mask_irq_ioc(struct irq_data *data)
{
    writel(1 << (data->irq - ICPU_IRQ0_BASE), VTSS_ICPU_CFG_INTR_INTR_ENA_CLR); /* Disable */
}

static void mask_ack_irq_ioc(struct irq_data *data)
{
    writel(1 << (data->irq - ICPU_IRQ0_BASE), VTSS_ICPU_CFG_INTR_INTR_ENA_CLR); /* Disable */
    writel(1 << (data->irq - ICPU_IRQ0_BASE), VTSS_ICPU_CFG_INTR_INTR_STICKY); /* Ack sticky */
}

static void unmask_irq_ioc(struct irq_data *data)
{
    writel(1 << (data->irq - ICPU_IRQ0_BASE), VTSS_ICPU_CFG_INTR_INTR_ENA_SET); /* Enable */
}

static struct irq_chip vcoreiii_irq_ioc = {
    .name = "icpu",
    .irq_ack = ack_irq_ioc,
    .irq_mask = mask_irq_ioc,
    .irq_mask_ack = mask_ack_irq_ioc,
    .irq_unmask = unmask_irq_ioc,
};

void __init vcoreiii_irq_init(void)
{
    u32 i;

    writel(0, VTSS_ICPU_CFG_INTR_INTR_ENA); /* Mask all */
    writel(0xffffffff, VTSS_ICPU_CFG_INTR_INTR_STICKY); /* Ack pending */

    for (i = ICPU_IRQ0_BASE; i <= PCIE_IRQ ; i++)
        irq_set_chip_and_handler(i, &vcoreiii_irq_ioc, handle_level_irq);
    irq_set_chained_handler(INT0_IRQ, handle_simple_irq);
}

static inline int clz(unsigned long x)
{
	__asm__ (
	"	.set	push					\n"
	"	.set	mips32					\n"
	"	clz	%0, %1					\n"
	"	.set	pop					\n"
	: "=r" (x)
	: "r" (x));

	return x;
}

static inline unsigned int irq_ffs(unsigned int pending)
{
    return -clz(pending) + 31;
}

asmlinkage void plat_irq_dispatch(void)
{
    unsigned int pending = read_c0_cause() & read_c0_status() & ST0_IM;

    if (pending & STATUSF_IP7)                      /* cpu timer */
        do_IRQ(TIMER_IRQ);
    else if (pending & STATUSF_IP2) {               /* vcoreiii pic */
        int irq = readl(VTSS_ICPU_CFG_INTR_INTR_IDENT);
        if (unlikely(irq == 0)) {
            spurious_interrupt();
            return;
        }
        irq = ICPU_IRQ0_BASE + irq_ffs(irq);
        do_IRQ(irq);
    } else if (pending & STATUSF_IP0)               /* user line 0 */
        do_IRQ(MIPS_SOFTINT0_IRQ);
    else if (pending & STATUSF_IP1)                 /* user line 1 */
        do_IRQ(MIPS_SOFTINT1_IRQ);
    else
        spurious_interrupt();
}

void __init arch_init_irq(void)
{
    mips_cpu_irq_init();
}
