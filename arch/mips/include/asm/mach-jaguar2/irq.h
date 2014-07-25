/*
 *
 * IRQ definitions
 *
 * Copyright (C) 2012 Vitesse Semiconductor Inc.
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

#ifndef __JAGUAR2_IRQ_H
#define __JAGUAR2_IRQ_H

#define NR_IRQS 64

/*
 * CPU core Interrupt Numbers
 */
#define MIPS_CPU_IRQ_BASE       0
#define MIPS_CPU_IRQ(x)         (MIPS_CPU_IRQ_BASE + (x))
#define MIPS_SOFTINT0_IRQ       MIPS_CPU_IRQ(0)
#define MIPS_SOFTINT1_IRQ       MIPS_CPU_IRQ(1)
#define INT0_IRQ                MIPS_CPU_IRQ(2)
#define INT1_IRQ                MIPS_CPU_IRQ(3)
#define INT2_IRQ                MIPS_CPU_IRQ(4)
#define INT3_IRQ                MIPS_CPU_IRQ(5)
#define INT4_IRQ                MIPS_CPU_IRQ(6)
#define TIMER_IRQ               MIPS_CPU_IRQ(7)

/*
 * ICPU IRQ0 Interrupt Numbers
 */
#define ICPU_IRQ0_BASE          (TIMER_IRQ + 1) /* 8 */
#define ICPU_IRQ0(x)            (ICPU_IRQ0_BASE + (x))

#define DEV_ALL_IRQ       ICPU_IRQ0(0)
#define EXT0_IRQ          ICPU_IRQ0(1)
#define EXT1_IRQ          ICPU_IRQ0(2)
#define TIMER0_IRQ        ICPU_IRQ0(3)
#define TIMER1_IRQ        ICPU_IRQ0(4)
#define TIMER2_IRQ        ICPU_IRQ0(5)
#define UART_IRQ          ICPU_IRQ0(6)
#define UART2_IRQ         ICPU_IRQ0(7)
#define TWI_IRQ           ICPU_IRQ0(8)
#define TWI2_IRQ          ICPU_IRQ0(9)
#define SIMC_IRQ          ICPU_IRQ0(10)
#define SW0_IRQ           ICPU_IRQ0(11)
#define SW1_IRQ           ICPU_IRQ0(12)
#define SGPIO_IRQ         ICPU_IRQ0(13)
#define SGPIO1_IRQ        ICPU_IRQ0(14)
#define SGPIO2_IRQ        ICPU_IRQ0(15)
#define GPIO_IRQ          ICPU_IRQ0(16)
#define MIIM0_INTR_IRQ    ICPU_IRQ0(17)
#define MIIM1_INTR_IRQ    ICPU_IRQ0(18)
#define MIIM2_INTR_IRQ    ICPU_IRQ0(19)
#define FDMA_IRQ          ICPU_IRQ0(20)
#define ANA_IRQ           ICPU_IRQ0(21)
#define PTP_RDY_IRQ       ICPU_IRQ0(22)
#define PTP_SYNC_IRQ      ICPU_IRQ0(23)
#define INTEGRITY_IRQ     ICPU_IRQ0(24)
#define XTR_RDY_IRQ       ICPU_IRQ0(25)
#define INJ_RDY_IRQ       ICPU_IRQ0(26)
#define PCIE_IRQ          ICPU_IRQ0(27)
#define OAM_VOP_IRQ       ICPU_IRQ0(28)

#include_next <irq.h>

#endif /* __JAGUAR2_IRQ_H */
