/*
 *
 * VCore-III I2C platform data structure
 *
 * Copyright (C) 2011 Vitesse Semiconductor Inc.
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/slab.h>

#if defined(CONFIG_VTSS_VCOREIII_MK1)
#include <asm/mach-vcoreiii/hardware.h>
#include <asm/mach-vcoreiii/i2c.h>
#elif defined(CONFIG_VTSS_VCOREIII_SERVAL1)
#include <asm/mach-serval/hardware.h>
#include <asm/mach-serval/i2c.h>
#elif defined(CONFIG_VTSS_VCOREIII_JAGUAR2)
#include <asm/mach-jaguar2/hardware.h>
#include <asm/mach-jaguar2/i2c.h>
#else
#error Invalid architecture type
#endif



#define DEBUG_I2C(x...) do { if(debug) printk(KERN_DEBUG x); } while(0)
#define DEBUG_I2C_L(l, x...) do { if(debug >= l) printk(KERN_DEBUG x); } while(0)

#define MOD_NAME     "i2c_vcoreiii"

#define VCOREIII_XFER_TIMEOUT       (HZ/2) // 0.5sec
#define VCOREIII_TX_FIFO_FULL_LEVEL 8
#define VCOREIII_TX_FIFO_THRESHOLD  6


// Macro for accessing registers - Used for being able to see when we do registers accesses
/* Single-instance macros */
#if !defined(CONFIG_VTSS_VCOREIII_JAGUAR2)
/* Only one instance available */
#define VTSS_WR(data, address) writel(data, address)
#define VTSS_RD(address) readl(address)
#else
/* Use the default instance for now */
#define VTSS_WR(data,address) writel(data, address(VTSS_TO_TWI))
#define VTSS_RD(address) readl(address(VTSS_TO_TWI))
#endif

#define VTSS_WRS(data, address) writel(data, address)

// On JR2, VTSS_F_xxx() macros for single-bit-fields have been
// replaced by VTSS_M_xxx() macros. The VTSS_F_xxx() macros
// all take a parameter.
#if !defined(VTSS_M_TWI_TWI_ENABLE_STATUS_BUSY)
#define VTSS_M_TWI_TWI_ENABLE_STATUS_BUSY VTSS_F_TWI_TWI_ENABLE_STATUS_BUSY
#endif
#if !defined(VTSS_M_TWI_TWI_STAT_TFNF)
#define VTSS_M_TWI_TWI_STAT_TFNF VTSS_F_TWI_TWI_STAT_TFNF
#endif
#if !defined(VTSS_M_TWI_TWI_DATA_CMD_CMD)
#define VTSS_M_TWI_TWI_DATA_CMD_CMD VTSS_F_TWI_TWI_DATA_CMD_CMD
#endif
#if !defined(VTSS_M_TWI_TWI_INTR_STAT_TX_ABRT)
#define VTSS_M_TWI_TWI_INTR_STAT_TX_ABRT VTSS_F_TWI_TWI_INTR_STAT_TX_ABRT
#endif
#if !defined(VTSS_M_TWI_TWI_INTR_MASK_M_RX_FULL)
#define VTSS_M_TWI_TWI_INTR_MASK_M_RX_FULL VTSS_F_TWI_TWI_INTR_MASK_M_RX_FULL
#endif
#if !defined(VTSS_M_TWI_TWI_STAT_RFNE)
#define VTSS_M_TWI_TWI_STAT_RFNE VTSS_F_TWI_TWI_STAT_RFNE
#endif
#if !defined(VTSS_M_TWI_TWI_INTR_MASK_M_TX_EMPTY)
#define VTSS_M_TWI_TWI_INTR_MASK_M_TX_EMPTY VTSS_F_TWI_TWI_INTR_MASK_M_TX_EMPTY
#endif
#if !defined(VTSS_M_TWI_TWI_INTR_STAT_STOP_DET)
#define VTSS_M_TWI_TWI_INTR_STAT_STOP_DET VTSS_F_TWI_TWI_INTR_STAT_STOP_DET
#endif
#if !defined(VTSS_M_TWI_TWI_CTRL_ENABLE)
#define VTSS_M_TWI_TWI_CTRL_ENABLE VTSS_F_TWI_TWI_CTRL_ENABLE
#endif
#if !defined(VTSS_M_ICPU_CFG_TWI_DELAY_TWI_CONFIG_TWI_DELAY_ENABLE)
#define VTSS_M_ICPU_CFG_TWI_DELAY_TWI_CONFIG_TWI_DELAY_ENABLE VTSS_F_ICPU_CFG_TWI_DELAY_TWI_CONFIG_TWI_DELAY_ENABLE
#endif
#if !defined(VTSS_M_TWI_TWI_INTR_MASK_M_TX_ABRT)
#define VTSS_M_TWI_TWI_INTR_MASK_M_TX_ABRT VTSS_F_TWI_TWI_INTR_MASK_M_TX_ABRT
#endif
#if !defined(VTSS_M_TWI_TWI_INTR_STAT_RX_DONE)
#define VTSS_M_TWI_TWI_INTR_STAT_RX_DONE VTSS_F_TWI_TWI_INTR_STAT_RX_DONE
#endif
#if !defined(VTSS_M_TWI_TWI_INTR_STAT_RX_OVER)
#define VTSS_M_TWI_TWI_INTR_STAT_RX_OVER VTSS_F_TWI_TWI_INTR_STAT_RX_OVER
#endif
#if !defined(VTSS_M_TWI_TWI_INTR_STAT_TX_OVER)
#define VTSS_M_TWI_TWI_INTR_STAT_TX_OVER VTSS_F_TWI_TWI_INTR_STAT_TX_OVER
#endif
#if !defined(VTSS_M_TWI_TWI_TX_ABRT_SOURCE_ABRT_TXDATA_NOACK)
#define VTSS_M_TWI_TWI_TX_ABRT_SOURCE_ABRT_TXDATA_NOACK VTSS_F_TWI_TWI_TX_ABRT_SOURCE_ABRT_TXDATA_NOACK
#endif
#if !defined(VTSS_M_TWI_TWI_TX_ABRT_SOURCE_ARB_LOST)
#define VTSS_M_TWI_TWI_TX_ABRT_SOURCE_ARB_LOST VTSS_F_TWI_TWI_TX_ABRT_SOURCE_ARB_LOST
#endif
#if !defined(VTSS_M_TWI_TWI_INTR_STAT_RX_FULL)
#define VTSS_M_TWI_TWI_INTR_STAT_RX_FULL VTSS_F_TWI_TWI_INTR_STAT_RX_FULL
#endif
#if !defined(VTSS_M_TWI_TWI_INTR_STAT_TX_EMPTY)
#define VTSS_M_TWI_TWI_INTR_STAT_TX_EMPTY VTSS_F_TWI_TWI_INTR_STAT_TX_EMPTY
#endif
#if !defined(VTSS_M_TWI_TWI_CFG_MASTER_ENA)
#define VTSS_M_TWI_TWI_CFG_MASTER_ENA VTSS_F_TWI_TWI_CFG_MASTER_ENA
#endif
#if !defined(VTSS_M_TWI_TWI_CFG_SLAVE_DIS)
#define VTSS_M_TWI_TWI_CFG_SLAVE_DIS VTSS_F_TWI_TWI_CFG_SLAVE_DIS
#endif
#if !defined(VTSS_M_TWI_TWI_CFG_RESTART_ENA)
#define VTSS_M_TWI_TWI_CFG_RESTART_ENA VTSS_F_TWI_TWI_CFG_RESTART_ENA
#endif


struct vcoreiii_twi_iface {
    struct device      *dev;
    int                irq;
    u8                 *buf;
    size_t             buf_len;
    struct completion  cmd_complete;
    int                cmd_err;
    spinlock_t         lock;
    struct i2c_adapter adap;
};

static int debug;

enum {
    I2C_CMPLT_ABORT = (1 << 0),
};

static void i2c_reset(struct i2c_adapter *adap)
{
    VTSS_WR(0x0, VTSS_TWI_TWI_CTRL); /* Leave it disabled */
}

static void stuff_tx(struct vcoreiii_twi_iface *dev)
{
    unsigned long flags;
    int lev;
    spin_lock_irqsave(&dev->lock, flags);

    /* write to slave */
    while(dev->buf_len && (lev = VTSS_RD(VTSS_TWI_TWI_TXFLR)) < VCOREIII_TX_FIFO_FULL_LEVEL) {
        DEBUG_I2C_L(2,"PUT 0x%x, %d bytes left, level %d\n", *(dev->buf), dev->buf_len, lev);
        VTSS_WR(*(dev->buf), VTSS_TWI_TWI_DATA_CMD);
        dev->buf++;
        dev->buf_len--;
    }
    if(dev->buf_len == 0) {
        VTSS_WR(0, VTSS_TWI_TWI_TX_TL); /* when empty, call me */
    }
    spin_unlock_irqrestore(&dev->lock, flags);
}

static void stuff_rx(struct vcoreiii_twi_iface *dev)
{
    unsigned long flags;
    int lev;
    size_t cnt = dev->buf_len;
    
    spin_lock_irqsave(&dev->lock, flags);

    /* read from slave */
    while ((lev = VTSS_RD(VTSS_TWI_TWI_STAT) & VTSS_M_TWI_TWI_STAT_TFNF)) {
        if (cnt > 0) {
            DEBUG_I2C("GET 1 byte, %d bytes left, level %d\n", cnt, lev);
            VTSS_WR(VTSS_M_TWI_TWI_DATA_CMD_CMD, VTSS_TWI_TWI_DATA_CMD);
            cnt--;                
        }
        if (cnt == 0)
            break;
    }

    spin_unlock_irqrestore(&dev->lock, flags);
}

static int wait_for_tx_buffer(int level, int timeout)
{
    unsigned long status;

    while((status = VTSS_RD(VTSS_TWI_TWI_TXFLR) > level) && timeout > 0) {
        DEBUG_I2C("TX wait level %ld - want %d\n", status, level);
        msleep(1);
        timeout--;
    }

    return status <= level;
}

static int do_xfer(struct i2c_adapter *adap, struct i2c_msg *msg)
{
    struct vcoreiii_twi_iface *dev = i2c_get_adapdata(adap);
    int r;

    if (msg->len == 0)
        return -EINVAL;

    /* Safety - wait upto 5 msecs for TX buffer to drain */
    wait_for_tx_buffer(0, 5); 

    // disable controller to write TAR
    VTSS_WR(0x0, VTSS_TWI_TWI_CTRL);

    // set target address
    VTSS_WR(msg->addr, VTSS_TWI_TWI_TAR);

    // enable controller
    VTSS_WR(VTSS_M_TWI_TWI_CTRL_ENABLE, VTSS_TWI_TWI_CTRL);

    dev->buf = msg->buf;
    dev->buf_len = msg->len;
    dev->cmd_err = 0;
    init_completion(&dev->cmd_complete);

    if (msg->flags & I2C_M_RD) {
        // read command - data = 0x00 (don't care)
        //VTSS_WR(VTSS_M_TWI_TWI_DATA_CMD_CMD, VTSS_TWI_TWI_DATA_CMD);
        while (dev->buf_len > 0) {
            stuff_rx(dev);
            VTSS_WR(VTSS_M_TWI_TWI_INTR_MASK_M_RX_FULL, VTSS_TWI_TWI_INTR_MASK); // enable rx fifo interrupt
        }
    } else {
        // write command - stuff data into fifo
        VTSS_WR(VCOREIII_TX_FIFO_THRESHOLD, VTSS_TWI_TWI_TX_TL); /* less than 3/4 full, call me */
        stuff_tx(dev);
        VTSS_WR(VTSS_M_TWI_TWI_INTR_MASK_M_TX_ABRT|
               VTSS_M_TWI_TWI_INTR_STAT_TX_OVER|
               VTSS_M_TWI_TWI_INTR_MASK_M_TX_EMPTY, 
	       VTSS_TWI_TWI_INTR_MASK); // enable tx fifo interrupt
    }

    r = wait_for_completion_interruptible_timeout(&dev->cmd_complete, VCOREIII_XFER_TIMEOUT);
    dev->buf_len = 0;
    if (r < 0)
        return r;

    /* no error */
    if (likely(r != 0 && !dev->cmd_err))
        return msg->len;

    /* Something bad happened */
    i2c_reset(adap);

    if (r == 0) {
        DEBUG_I2C("controller timed out\n");
        return -ETIMEDOUT;
    }

    /* Something else... */
    return -EIO;
}

static int vcoreiii_xfer(struct i2c_adapter *adap, struct i2c_msg *pmsg, int num)
{
    int i, rc = 0;

    DEBUG_I2C_L(2, "%s: vcoreiii_xfer - processing %d messages:\n", MOD_NAME, num);

    for(i = 0; i < num; i++, pmsg++) {
	DEBUG_I2C("MSG #%d: %sing %d byte%s %s 0x%02x flags %0x\n", i,
		  pmsg->flags & I2C_M_RD ? "read" : "writ",
		  pmsg->len, pmsg->len == 1 ? "" : "s",
		  pmsg->flags & I2C_M_RD ? "from" : "to", pmsg->addr, pmsg->flags);
        if((rc = do_xfer(adap, pmsg)) < 0)
            return rc;
    }
    return num;
}

static unsigned int vcoreiii_func(struct i2c_adapter *adapter)
{
    return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static irqreturn_t vcoreiii_twi_interrupt_entry(int irq, void *dev_id)
{
    struct vcoreiii_twi_iface *dev = dev_id;
    unsigned long status;
    int count = 0;

    while((status = VTSS_RD(VTSS_TWI_TWI_INTR_STAT))) { // figure out what interrupt we got
        DEBUG_I2C_L(2, "IRQ stat 0x%lx\n", status);
        if (count++ == 100) {
            dev_err(dev->dev, "Too much work in one IRQ - stat 0x%lx\n", status);
            break;
        }
	if(status & VTSS_M_TWI_TWI_INTR_STAT_TX_ABRT) {
            status &= ~VTSS_M_TWI_TWI_INTR_STAT_TX_ABRT; /* clear tx_abrt */
            DEBUG_I2C(" TX abrt src 0x%x\n", VTSS_RD(VTSS_TWI_TWI_TX_ABRT_SOURCE));
            VTSS_WR(0, VTSS_TWI_TWI_INTR_MASK);
            dev->cmd_err |= I2C_CMPLT_ABORT;
            complete(&dev->cmd_complete);
        }

        if(status & VTSS_M_TWI_TWI_INTR_STAT_RX_FULL) {
            status &= ~VTSS_M_TWI_TWI_INTR_MASK_M_RX_FULL; /* Clear rx_full */
            /* Drain Rx FIFO */
            while (VTSS_RD(VTSS_TWI_TWI_STAT) & VTSS_M_TWI_TWI_STAT_RFNE) {
                /* while data in fifo */
                (*dev->buf) = VTSS_RD(VTSS_TWI_TWI_DATA_CMD);
                DEBUG_I2C("READ %x, %d bytes left\n", *(dev->buf), dev->buf_len);
                dev->buf++;
                if (--dev->buf_len == 0) {
                    VTSS_WR(0, VTSS_TWI_TWI_INTR_MASK);
                    DEBUG_I2C("Read complete\n");
                    complete(&dev->cmd_complete);
                    break;
                }
            }            
        }

        if(status & VTSS_M_TWI_TWI_INTR_STAT_TX_EMPTY) {
            status &= ~VTSS_M_TWI_TWI_INTR_MASK_M_TX_EMPTY; /* clear tx_empty */
            if (dev->buf_len) {
                stuff_tx(dev);
            } else {
                u32 lev = VTSS_RD(VTSS_TWI_TWI_TXFLR);
                if(lev == 0) {
                    VTSS_WR(0, VTSS_TWI_TWI_INTR_MASK);
                    DEBUG_I2C("Empty and done - status 0x%lx\n", status);
                    complete(&dev->cmd_complete);
                } else {
                    DEBUG_I2C("Empty and not done - status 0x%lx, level %d\n", status, lev);
                }
            }
        }

        if (status) {
            dev_err(dev->dev, "Unexpected status 0x%lx\n", status);
        }
        VTSS_RD(VTSS_TWI_TWI_CLR_INTR); // clear the interrupt(s)
    }
    return count ? IRQ_HANDLED : IRQ_NONE;
}

static struct i2c_algorithm i2c_vcoreiii_algo = {
    .master_xfer    = vcoreiii_xfer,
    .functionality  = vcoreiii_func,
};

static int i2c_vcoreiii_hwinit(const struct vcoreiii_i2c_platform_data *pdata)
{
    unsigned long clk_freq = VCOREIII_AHB_CLOCK, reg_val;

    reg_val = (5 * clk_freq / 1000000) - 8;  // datasheet 6.17.1.5
    VTSS_WR(reg_val, VTSS_TWI_TWI_SS_SCL_HCNT);

    reg_val = (5 * clk_freq / 1000000) - 1;  // datasheet 6.17.1.6
    VTSS_WR(reg_val, VTSS_TWI_TWI_SS_SCL_LCNT);

    reg_val = VTSS_F_ICPU_CFG_TWI_DELAY_TWI_CONFIG_TWI_CNT_RELOAD((unsigned int)(0.3 * clk_freq / 1000000) - 1) | VTSS_M_ICPU_CFG_TWI_DELAY_TWI_CONFIG_TWI_DELAY_ENABLE;  // datasheet 6.17
    VTSS_WRS(reg_val, VTSS_ICPU_CFG_TWI_DELAY_TWI_CONFIG);

    reg_val = (1.1 * clk_freq / 1000000) - 1;  // datasheet 6.17.1.7
    VTSS_WR(reg_val, VTSS_TWI_TWI_FS_SCL_HCNT);

    reg_val = (1.4 * clk_freq / 1000000) - 1;  // datasheet 6.17.1.8
    VTSS_WR(reg_val, VTSS_TWI_TWI_FS_SCL_LCNT);

    reg_val =
            VTSS_M_TWI_TWI_CFG_MASTER_ENA |
            VTSS_F_TWI_TWI_CFG_SPEED(pdata->fast_mode ? 2 : 1) | /* 400 or 100 kbit/s */
            VTSS_M_TWI_TWI_CFG_RESTART_ENA |
            VTSS_M_TWI_TWI_CFG_SLAVE_DIS;
    VTSS_WR(reg_val, VTSS_TWI_TWI_CFG);

    reg_val = (0.25 * clk_freq / 1000000);  // datasheet 6.17.1.30
    VTSS_WR(reg_val, VTSS_TWI_TWI_SDA_SETUP);

    VTSS_WR(0, VTSS_TWI_TWI_RX_TL); /* (n+1) => one byte of data */
    VTSS_WR(0x0, VTSS_TWI_TWI_INTR_MASK); // mask all until we're ready
    VTSS_WR(VTSS_M_TWI_TWI_CTRL_ENABLE, VTSS_TWI_TWI_CTRL);

    return 0;
}

static struct vcoreiii_i2c_platform_data i2c_data_default = {
    .fast_mode = 0,             /* 100 kbit/s */
};

static int i2c_vcoreiii_probe(struct platform_device *pdev)
{
    struct vcoreiii_i2c_platform_data *pdata = pdev->dev.platform_data;
    struct vcoreiii_twi_iface *iface;
    struct i2c_adapter *adapter;
    int rc=1;

    DEBUG_I2C_L(2, "%s: i2c_vcoreiii_probe, pdata %p\n", MOD_NAME, pdata);
    if(!pdata)
        pdata = &i2c_data_default;

#if defined(CONFIG_VTSS_VCOREIII_JAGUAR)
    vcoreiii_gpio_set_alternate(14, 1); /* TWI_SCL */
    vcoreiii_gpio_set_alternate(15, 1); /* TWI_SDA */
#elif defined(CONFIG_VTSS_VCOREIII_LUTON26)
    vcoreiii_gpio_set_alternate(5, 1); /* TWI_SCL */
    vcoreiii_gpio_set_alternate(6, 1); /* TWI_SDA */
#elif defined(CONFIG_VTSS_VCOREIII_JAGUAR2)
    vcoreiii_gpio_set_alternate(14, 1); /* TWI_SCL */
    vcoreiii_gpio_set_alternate(15, 1); /* TWI_SDA */
#else
#error Unsupported platform!
#endif

    i2c_vcoreiii_hwinit(pdata);

    iface = kzalloc(sizeof(struct vcoreiii_twi_iface), GFP_KERNEL);
    if(iface == NULL) {
        dev_err(&pdev->dev, "can't allocate interface\n");
        rc = -ENOMEM;
        goto fail0;
    }

    iface->dev = get_device(&pdev->dev);
    iface->irq = TWI_IRQ;
    spin_lock_init(&iface->lock);
    platform_set_drvdata(pdev, iface);

    adapter = &iface->adap;
    i2c_set_adapdata(adapter, iface);
    adapter->owner = THIS_MODULE;
    adapter->class = I2C_CLASS_HWMON;
    snprintf(adapter->name, sizeof(adapter->name), MOD_NAME);
    adapter->algo = &i2c_vcoreiii_algo;
    adapter->dev.parent = &pdev->dev;

    rc = request_irq(iface->irq, vcoreiii_twi_interrupt_entry, 0, pdev->name, iface);
    if (rc) {
	dev_err(&pdev->dev, "Can't get IRQ %d !\n", iface->irq);
	rc = -ENODEV;
	goto fail1;
    }

    rc = i2c_add_numbered_adapter(adapter);
    if(rc) {
        dev_err(&pdev->dev, "Adapter %s registration failed\n", adapter->name);
        goto fail2;
    }

    dev_info(&pdev->dev, "i2c bus driver on IRQ %d\n", iface->irq);
    return 0;

fail2:
    free_irq(iface->irq, iface);
fail1:
    platform_set_drvdata(pdev, NULL);
    kfree(adapter);
fail0:
    return 0;
}

static int i2c_vcoreiii_remove(struct platform_device *pdev)
{
    struct vcoreiii_twi_iface *iface = platform_get_drvdata(pdev);

    dev_info(&pdev->dev, "Driver remove (releasing IRQ %d)\n", iface->irq);

    platform_set_drvdata(pdev, NULL);

    i2c_del_adapter(&(iface->adap));
    free_irq(iface->irq, iface);
    kfree(iface);

    return 0;
}

static struct platform_driver i2c_vcoreiii_driver = {
	.probe		= i2c_vcoreiii_probe,
	.remove		= i2c_vcoreiii_remove,
	.driver		= {
		.name	= MOD_NAME,
		.owner	= THIS_MODULE,
	},
};

static int __init i2c_vcoreiii_init(void)
{
#ifdef DEBUG
    debug++;
#endif
    return platform_driver_register(&i2c_vcoreiii_driver);
}
subsys_initcall(i2c_vcoreiii_init);

static void __exit i2c_vcoreiii_exit(void)
{
    platform_driver_unregister(&i2c_vcoreiii_driver);
}
module_exit(i2c_vcoreiii_exit);

module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "Enable debug");

/* work with hotplug and coldplug */
MODULE_ALIAS("platform:i2c_vcoreiii");
MODULE_AUTHOR("Lars Povlsen <lpovlsen at vitesse.com>");
MODULE_DESCRIPTION("Vitesse VCore-III I2C bus adapter");
MODULE_LICENSE("GPL");
