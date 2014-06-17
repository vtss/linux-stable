#ifndef _VTSS_SERVAL_REGS_UART_H_
#define _VTSS_SERVAL_REGS_UART_H_

/*
 *
 * VCore-III Register Definitions
 *
 * Copyright (C) 2012 Vitesse Semiconductor Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#include "vtss_serval_regs_common.h"

#define VTSS_UART_UART_RBR_THR(target)       VTSS_IOREG(target,0x0)
#define  VTSS_F_UART_UART_RBR_THR_RBR_THR(x)  VTSS_ENCODE_BITFIELD(x,0,8)
#define  VTSS_M_UART_UART_RBR_THR_RBR_THR     VTSS_ENCODE_BITMASK(0,8)
#define  VTSS_X_UART_UART_RBR_THR_RBR_THR(x)  VTSS_EXTRACT_BITFIELD(x,0,8)

#define VTSS_UART_UART_IER(target)           VTSS_IOREG(target,0x1)
#define  VTSS_F_UART_UART_IER_PTIME           VTSS_BIT(7)
#define  VTSS_F_UART_UART_IER_EDSSI           VTSS_BIT(3)
#define  VTSS_F_UART_UART_IER_ELSI            VTSS_BIT(2)
#define  VTSS_F_UART_UART_IER_ETBEI           VTSS_BIT(1)
#define  VTSS_F_UART_UART_IER_ERBFI           VTSS_BIT(0)

#define VTSS_UART_UART_IIR_FCR(target)       VTSS_IOREG(target,0x2)
#define  VTSS_F_UART_UART_IIR_FCR_FIFOSE_RT(x)  VTSS_ENCODE_BITFIELD(x,6,2)
#define  VTSS_M_UART_UART_IIR_FCR_FIFOSE_RT     VTSS_ENCODE_BITMASK(6,2)
#define  VTSS_X_UART_UART_IIR_FCR_FIFOSE_RT(x)  VTSS_EXTRACT_BITFIELD(x,6,2)
#define  VTSS_F_UART_UART_IIR_FCR_TET(x)      VTSS_ENCODE_BITFIELD(x,4,2)
#define  VTSS_M_UART_UART_IIR_FCR_TET         VTSS_ENCODE_BITMASK(4,2)
#define  VTSS_X_UART_UART_IIR_FCR_TET(x)      VTSS_EXTRACT_BITFIELD(x,4,2)
#define  VTSS_F_UART_UART_IIR_FCR_XFIFOR      VTSS_BIT(2)
#define  VTSS_F_UART_UART_IIR_FCR_RFIFOR      VTSS_BIT(1)
#define  VTSS_F_UART_UART_IIR_FCR_FIFOE       VTSS_BIT(0)

#define VTSS_UART_UART_LCR(target)           VTSS_IOREG(target,0x3)
#define  VTSS_F_UART_UART_LCR_DLAB            VTSS_BIT(7)
#define  VTSS_F_UART_UART_LCR_BC              VTSS_BIT(6)
#define  VTSS_F_UART_UART_LCR_EPS             VTSS_BIT(4)
#define  VTSS_F_UART_UART_LCR_PEN             VTSS_BIT(3)
#define  VTSS_F_UART_UART_LCR_STOP            VTSS_BIT(2)
#define  VTSS_F_UART_UART_LCR_DLS(x)          VTSS_ENCODE_BITFIELD(x,0,2)
#define  VTSS_M_UART_UART_LCR_DLS             VTSS_ENCODE_BITMASK(0,2)
#define  VTSS_X_UART_UART_LCR_DLS(x)          VTSS_EXTRACT_BITFIELD(x,0,2)

#define VTSS_UART_UART_MCR(target)           VTSS_IOREG(target,0x4)
#define  VTSS_F_UART_UART_MCR_AFCE            VTSS_BIT(5)
#define  VTSS_F_UART_UART_MCR_LB              VTSS_BIT(4)
#define  VTSS_F_UART_UART_MCR_RTS             VTSS_BIT(1)

#define VTSS_UART_UART_LSR(target)           VTSS_IOREG(target,0x5)
#define  VTSS_F_UART_UART_LSR_RFE             VTSS_BIT(7)
#define  VTSS_F_UART_UART_LSR_TEMT            VTSS_BIT(6)
#define  VTSS_F_UART_UART_LSR_THRE            VTSS_BIT(5)
#define  VTSS_F_UART_UART_LSR_BI              VTSS_BIT(4)
#define  VTSS_F_UART_UART_LSR_FE              VTSS_BIT(3)
#define  VTSS_F_UART_UART_LSR_PE              VTSS_BIT(2)
#define  VTSS_F_UART_UART_LSR_OE              VTSS_BIT(1)
#define  VTSS_F_UART_UART_LSR_DR              VTSS_BIT(0)

#define VTSS_UART_UART_MSR(target)           VTSS_IOREG(target,0x6)
#define  VTSS_F_UART_UART_MSR_CTS             VTSS_BIT(4)
#define  VTSS_F_UART_UART_MSR_DCTS            VTSS_BIT(0)

#define VTSS_UART_UART_SCR(target)           VTSS_IOREG(target,0x7)
#define  VTSS_F_UART_UART_SCR_SCR(x)          VTSS_ENCODE_BITFIELD(x,0,8)
#define  VTSS_M_UART_UART_SCR_SCR             VTSS_ENCODE_BITMASK(0,8)
#define  VTSS_X_UART_UART_SCR_SCR(x)          VTSS_EXTRACT_BITFIELD(x,0,8)

#define VTSS_UART_UART_USR(target)           VTSS_IOREG(target,0x1f)
#define  VTSS_F_UART_UART_USR_BUSY            VTSS_BIT(0)


#endif /* _VTSS_SERVAL_REGS_UART_H_ */
