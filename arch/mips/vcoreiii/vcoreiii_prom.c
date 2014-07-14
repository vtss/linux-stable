/*
 *
 * VCore-III PROM functionality
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
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/serial_reg.h>
#include <linux/ioport.h>
#include <asm/io.h>

#include <asm/bootinfo.h>

/* This struct is used by Redboot to pass arguments to the kernel */
typedef struct
{
	char *name;
	char *val;
} t_env_var;

struct parmblock {
	t_env_var memsize;
	t_env_var modetty0;
	t_env_var ethaddr;
	t_env_var env_end;
	char *argv[2];
	char text[0];
};

static const t_env_var * prom_env;

char * __init prom_getcmdline(void)
{
	return &(arcs_cmdline[0]);
}

static const char __init *prom_getenv(char *name)
{
	if(prom_env) {
		const t_env_var * p;
		for (p = prom_env; p->name != NULL; p++)
			if(strcmp(name, p->name) == 0)
				return p->val;
	}

	return NULL;
}

void __init prom_init(void)
{
    /* Sanity check for defunct bootloader */
    if(fw_arg0 < 10 && 
       (fw_arg1 & 0xFFF00000) == 0x80000000) {
        unsigned int prom_argc = fw_arg0;
        const char ** prom_argv = (const char **) fw_arg1;
        const struct parmblock * const pb = (struct parmblock *) fw_arg2;
	
        prom_env = &pb->memsize;
        
        if(prom_argc > 1 && strlen(prom_argv[1]) > 0)
            /* ignore all built-in args if any f/w args given */
            strcpy(arcs_cmdline, prom_argv[1]);
    } else {
        printk(KERN_NOTICE "Invalid kernel arglist - use RedBoot \"exec\" command to boot kernel.\n");
        printk(KERN_NOTICE "Using predefined kernel options.\n");
    }

    /* Add some random defaults... */
    if ((strstr(arcs_cmdline, "console=")) == NULL)
        strcat(arcs_cmdline, " console=ttyS0,115200");
    if ((strstr(arcs_cmdline, "init=")) == NULL)
      strcat(arcs_cmdline, " init=/etc/preinit");
    if ((strstr(arcs_cmdline, "root=")) == NULL)
        strcat(arcs_cmdline, " root=/dev/mtdblock4");
    if ((strstr(arcs_cmdline, "mem=")) == NULL) {
        const char *memopt = prom_getenv("memsize");
        unsigned long memsize;
        if(memopt && (memsize = simple_strtol(memopt, NULL, 16))) {
            /* Add directly as memory region */
            add_memory_region(0x00000000, memsize, BOOT_MEM_RAM);
        } else {
            /* Reasonable default */
            strcat(arcs_cmdline, " mem=128M");
        }
    }
}

void __init prom_free_prom_memory(void)
{
}

#ifdef CONFIG_EARLY_PRINTK
void prom_putchar(char c)
{
}
#endif
