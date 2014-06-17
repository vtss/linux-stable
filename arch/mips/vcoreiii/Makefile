#
# Makefile for the VCore-III reference boards
#

obj-y := vcoreiii_setup.o vcoreiii_irq.o vcoreiii_platform.o vcoreiii_prom.o vcoreiii_gpio.o vcoreiii_mtd.o

obj-$(CONFIG_MTD_PHYSMAP)	+= vcoreiii_mtd.o

EXTRA_CFLAGS += -Werror
