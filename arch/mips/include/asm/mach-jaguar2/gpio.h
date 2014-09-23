/*
 *
 * IRQ definitions
 *
 * Copyright (C) 2014 Vitesse Semiconductor Inc.
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

#ifndef __JAGUAR2_GPIO_H
#define __JAGUAR2_GPIO_H

#define ARCH_NR_GPIOS   512
#include <asm-generic/gpio.h>

/* Offset by 64 GPIO's, 3 controllers */
#define VCOREIII_SGPIO(cnt, port, bit) (64 + (cnt * 128) + (bit * 32) + (port))

#include_next <gpio.h>

#endif /* __JAGUAR2_GPIO_H */
