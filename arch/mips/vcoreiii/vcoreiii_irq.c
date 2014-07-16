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
#include <asm/mach-vcoreiii/irq.h>
#include <asm/mach-vcoreiii/hardware.h>

static void ack_irq_ioc(struct irq_data *data)
{
	writel(1 << (data->irq - ICPU_IRQ0_BASE), VTSS_ICPU_CFG_INTR_INTR); /* Ack sticky */
}

static void mask_irq_ioc(struct irq_data *data)
{
	writel(1 << (data->irq - ICPU_IRQ0_BASE), VTSS_ICPU_CFG_INTR_INTR_ENA_CLR); /* Disable */
}

static void mask_ack_irq_ioc(struct irq_data *data)
{
	writel(1 << (data->irq - ICPU_IRQ0_BASE), VTSS_ICPU_CFG_INTR_INTR_ENA_CLR); /* Disable */
	writel(1 << (data->irq - ICPU_IRQ0_BASE), VTSS_ICPU_CFG_INTR_INTR); /* Ack sticky */
}

static void unmask_irq_ioc(struct irq_data *data)
{
	writel(1 << (data->irq - ICPU_IRQ0_BASE), VTSS_ICPU_CFG_INTR_INTR_ENA_SET); /* Enable */
}

static struct irq_chip vcoreiii_irq_ioc = {
	.name = "vc3_pri",
	.irq_ack = ack_irq_ioc,
	.irq_mask = mask_irq_ioc,
	.irq_mask_ack = mask_ack_irq_ioc,
	.irq_unmask = unmask_irq_ioc,
};

#if defined(CONFIG_VTSS_VCOREIII_JAGUAR_DUAL)
static void slv_ack_irq_ioc(unsigned int irq)
{
    //printk("%s: %d\n", __FUNCTION__, irq - SLV_IRQ_BASE);
    slv_writel(1 << (irq - SLV_IRQ_BASE), VTSS_ICPU_CFG_INTR_INTR); /* Ack sticky */
    writel(1 << (EXT1_IRQ - ICPU_IRQ0_BASE), VTSS_ICPU_CFG_INTR_INTR); /* Ack master ext1 sticky */
}

static void slv_mask_irq_ioc(unsigned int irq)
{
    //printk("%s: %d\n", __FUNCTION__, irq - SLV_IRQ_BASE);
    slv_writel(1 << (irq - SLV_IRQ_BASE), VTSS_ICPU_CFG_INTR_INTR_ENA_CLR); /* Disable */
}

static void slv_mask_ack_irq_ioc(unsigned int irq)
{
    //printk("%s: %d\n", __FUNCTION__, irq - SLV_IRQ_BASE);
    slv_writel(1 << (irq - SLV_IRQ_BASE), VTSS_ICPU_CFG_INTR_INTR_ENA_CLR); /* Disable */
    slv_writel(1 << (irq - SLV_IRQ_BASE), VTSS_ICPU_CFG_INTR_INTR); /* Ack sticky */
    writel(1 << (EXT1_IRQ - ICPU_IRQ0_BASE), VTSS_ICPU_CFG_INTR_INTR); /* Ack master ext1 sticky */
}

static void slv_unmask_irq_ioc(unsigned int irq)
{
    slv_writel(1 << (irq - SLV_IRQ_BASE), VTSS_ICPU_CFG_INTR_INTR_ENA_SET); /* Enable */
}

static struct irq_chip vcoreiii_slvirq_ioc = {
    .name = "vc3_sec",
    .ack = slv_ack_irq_ioc,
    .mask = slv_mask_irq_ioc,
    .mask_ack = slv_mask_ack_irq_ioc,
    .unmask = slv_unmask_irq_ioc,
};
#endif	/* CONFIG_VTSS_VCOREIII_JAGUAR_DUAL */

void __init vcoreiii_irq_init(void)
{
	u32 i;

	writel(0, VTSS_ICPU_CFG_INTR_INTR_ENA); /* Mask all */
	writel(0xffffffff, VTSS_ICPU_CFG_INTR_INTR); /* Ack pending */

	for (i = ICPU_IRQ0_BASE; i <= MIIM1_INTR_IRQ ; i++)
		irq_set_chip_and_handler(i, &vcoreiii_irq_ioc, handle_level_irq);
	irq_set_chained_handler(INT0_IRQ, handle_simple_irq);
#if defined(CONFIG_VTSS_VCOREIII_JAGUAR_DUAL)
        /* EXT1-master (input) is connected to EXT0-slave (output) */
        /* Slave EXT0 is output, active low */
	slv_writel(VTSS_F_ICPU_CFG_INTR_EXT_IRQ0_INTR_CFG_EXT_IRQ0_INTR_DRV | 
                   VTSS_F_ICPU_CFG_INTR_EXT_IRQ0_INTR_CFG_EXT_IRQ0_INTR_DIR,
                   VTSS_ICPU_CFG_INTR_EXT_IRQ0_INTR_CFG);
        /* Master EXT1 is input, active low */
	writel(0, VTSS_ICPU_CFG_INTR_EXT_IRQ1_INTR_CFG);
        
        /* Route EXT_IRQ1 (input) and all internal interrupts to EXT_IRQ0 (output). */

#define JR_INTR_CFG_EXTERNAL(intr) slv_writel(VTSS_F_ICPU_CFG_INTR_##intr##_INTR_CFG_##intr##_INTR_SEL(2), VTSS_ICPU_CFG_INTR_##intr##_INTR_CFG) /* External, active low input, to EXT_IRQ0 */

#define JR_INTR_CFG_INTERNAL(intr) slv_writel(VTSS_F_ICPU_CFG_INTR_##intr##_INTR_CFG_##intr##_INTR_SEL(2), VTSS_ICPU_CFG_INTR_##intr##_INTR_CFG) /* Internal, to EXT_IRQ0 */

        /* EXT_IRQ1 is an input on the secondary chip. Route it to EXT_IRQ0. */
        JR_INTR_CFG_EXTERNAL(EXT_IRQ1);
        
        /* Remaining interrupts are also routed to EXT_IRQ0. */
        JR_INTR_CFG_INTERNAL(SW0);
        JR_INTR_CFG_INTERNAL(SW1);
        JR_INTR_CFG_INTERNAL(MIIM1);
        JR_INTR_CFG_INTERNAL(MIIM0);
        JR_INTR_CFG_INTERNAL(PI_SD0);
        JR_INTR_CFG_INTERNAL(PI_SD1);
        JR_INTR_CFG_INTERNAL(UART);
        JR_INTR_CFG_INTERNAL(TIMER0);
        JR_INTR_CFG_INTERNAL(TIMER1);
        JR_INTR_CFG_INTERNAL(TIMER2);
        JR_INTR_CFG_INTERNAL(FDMA);
        JR_INTR_CFG_INTERNAL(TWI);
        JR_INTR_CFG_INTERNAL(GPIO);
        JR_INTR_CFG_INTERNAL(SGPIO);
        JR_INTR_CFG_INTERNAL(DEV_ALL);
        JR_INTR_CFG_INTERNAL(BLK_ANA);
        JR_INTR_CFG_INTERNAL(XTR_RDY0);
        JR_INTR_CFG_INTERNAL(XTR_RDY1);
        JR_INTR_CFG_INTERNAL(XTR_RDY2);
        JR_INTR_CFG_INTERNAL(XTR_RDY3);
        JR_INTR_CFG_INTERNAL(INJ_RDY0);
        JR_INTR_CFG_INTERNAL(INJ_RDY1);
        JR_INTR_CFG_INTERNAL(INJ_RDY2);
        JR_INTR_CFG_INTERNAL(INJ_RDY3);
        JR_INTR_CFG_INTERNAL(INJ_RDY4);
        JR_INTR_CFG_INTERNAL(INTEGRITY);
        JR_INTR_CFG_INTERNAL(PTP_SYNC);
        
        /* GPIO Alternate functions EXT1(master) <-> EXT0(slave) */
        vcoreiii_gpio_set_alternate(7, 1); /* Master GPIO7 = EXT1 */
        vcoreiii_gpio_set_alternate_slv(6, 1); /* Slave GPIO6 = EXT0 */
        
        /* Enable EXT0 to drive it */
        slv_writel(VTSS_F_ICPU_CFG_INTR_EXT_IRQ0_ENA_EXT_IRQ0_ENA, 
                   VTSS_ICPU_CFG_INTR_EXT_IRQ0_ENA);

	slv_writel(0, VTSS_ICPU_CFG_INTR_INTR_ENA); /* Mask all */
	slv_writel(0xffffffff, VTSS_ICPU_CFG_INTR_INTR); /* Ack pending */

	for (i = SLV_IRQ_BASE; i <= SLV_MIIM1_INTR_IRQ ; i++)
            set_irq_chip_and_handler(i, &vcoreiii_slvirq_ioc, handle_level_irq);
	set_irq_chained_handler(EXT1_IRQ, handle_simple_irq);
#endif	/* CONFIG_VTSS_VCOREIII_JAGUAR_DUAL */

        // Enable VCoreIII INT routing to CPU IRQ0/1 (IP2/IP3).
        writel(VTSS_F_ICPU_CFG_INTR_ICPU_IRQ0_ENA_ICPU_IRQ0_ENA, VTSS_ICPU_CFG_INTR_ICPU_IRQ0_ENA);
        writel(VTSS_F_ICPU_CFG_INTR_ICPU_IRQ1_ENA_ICPU_IRQ1_ENA, VTSS_ICPU_CFG_INTR_ICPU_IRQ1_ENA);
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
                int irq = readl(VTSS_ICPU_CFG_INTR_ICPU_IRQ0_IDENT);
                if (unlikely(irq == 0)) {
                        spurious_interrupt();
                        return;
                }
		irq = ICPU_IRQ0_BASE + irq_ffs(irq);
#if defined(CONFIG_VTSS_VCOREIII_JAGUAR_DUAL)
                if(irq == EXT1_IRQ) {
                    irq = slv_readl(VTSS_ICPU_CFG_INTR_EXT_IRQ0_IDENT); /* *slave* EXT0 */
                    if (unlikely(irq == 0)) {
                        ack_irq_ioc(EXT1_IRQ); /* Ack the cascaded (master) IRQ */
                        spurious_interrupt();
                        return;
                    }
                    irq = SLV_IRQ_BASE + irq_ffs(irq);
                }
#endif  /* CONFIG_VTSS_VCOREIII_JAGUAR_DUAL */
                do_IRQ(irq);
        } else if (pending & STATUSF_IP3) {             /* vcoreiii UIO */
            int irq = readl(VTSS_ICPU_CFG_INTR_ICPU_IRQ1_IDENT); // UIO-controlled IRQ's only
            if (unlikely(irq == 0)) {
                spurious_interrupt();
                return;
            }
            do_IRQ(INT1_IRQ);
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
        // Change INT1_IRQ handler to avoid eoi (unmask)
        // This IRQ is exposed via UIO
        irq_set_handler(INT1_IRQ, handle_level_irq);
}
