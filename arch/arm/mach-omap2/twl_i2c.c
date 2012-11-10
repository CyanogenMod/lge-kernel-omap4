/* arch/arm/mach-omap2/twl-i2c.c
 *
 * Copyright (C) 2012 LGE Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/pm_runtime.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/err.h>

#include <plat/omap_device.h>
#include <plat/omap_hwmod.h>
#include <plat/clock.h>

#define MAX_OMAP_I2C_HWMOD_NAME_LEN	16

/* I2C chip address */
#define TWL6030_CHIP_PM			0x48
#define TWL6030_BASEADD_PM_MASTER       0x001F

#define I2C_STANDARD_MODE	100
#define I2C_FAST_MODE		400
#define I2C_HIGH_SPEED_MODE	3400

#define I2C_REV			0x04
#define I2C_SYSS		0x90
#define I2C_IRQSTATUS		0x28
#define I2C_IRQENABLE_SET	0x2c
#define I2C_WE			0x34
#define I2C_BUF			0x94
#define I2C_DATA		0x9c
#define I2C_CNT			0x98
#define I2C_CON			0xa4
#define I2C_SA			0xac
#define I2C_PSC			0xb0
#define I2C_SCLL		0xb4
#define I2C_SCLH		0xb8
#define I2C_OA			0xa8

/* I2C_SYSS register */
#define I2C_RDONE	(1<<0)

/* I2C_IRQSTATUS register */
#define I2C_BB		(1<<12)
#define I2C_XRDY	(1<<4)
#define I2C_RRDY	(1<<3)
#define I2C_ARDY	(1<<2)
#define I2C_NACK	(1<<1)
#define I2C_AL		(1<<0)

/* I2C_IRQENABLE_SET register */
#define I2C_XRDY_IE	(1<<4)
#define I2C_RRDY_IE	(1<<3)
#define I2C_ARDY_IE	(1<<2)
#define I2C_NACK_IE	(1<<1)
#define I2C_AL_IE	(1<<0)

/* I2C_CON register */
#define I2C_EN			(1<<15)
#define I2C_OPMODE_HIGHSPEED	(1<<12)
#define I2C_MST			(1<<10)
#define I2C_TRX			(1<<9)
#define I2C_STP			(1<<1)
#define I2C_STT			(1<<0)

struct twl_i2c_dev {
	void __iomem	*base;	/* virtual */
	int		irq;
	int		rev;
	struct device	*dev;
};

static struct twl_i2c_dev *i2c_dev = NULL;

static unsigned short i2c_opmode = 0;

static inline void omap4_i2c_reg_write(int reg, u16 val)
{
	__raw_writew(val, i2c_dev->base + reg);
}

static inline u16 omap4_i2c_reg_read(int reg)
{
	return __raw_readw(i2c_dev->base + reg);
}

static inline void i2c_wait_for_bus_busy(void)
{
	int i;

	/* clear whatever is pending */
	omap4_i2c_reg_write(I2C_IRQSTATUS, 0xffff);

	for (i = 5000; i > 0; i--) {
		if ((omap4_i2c_reg_read(I2C_IRQSTATUS) & I2C_BB) == 0)
			break;
		omap4_i2c_reg_write(I2C_IRQSTATUS, 0xffff);
	}

	if (i == 0) {
		pr_err("twl-i2c: %s: timeout error!\n", __func__);
		return;
	}

	omap4_i2c_reg_write(I2C_IRQSTATUS, 0xffff);
}

static inline void flush_i2c_fifo(void)
{
	unsigned short status;

	/* NOTE: if you try and read data when it's not there or ready
	 *       you get a bus error
	 */
	while (1) {
		status = omap4_i2c_reg_read(I2C_IRQSTATUS);

		switch(status) {
			case I2C_RRDY:
				omap4_i2c_reg_read(I2C_DATA);
				omap4_i2c_reg_write(I2C_IRQSTATUS, I2C_RRDY);
				udelay(1000);
				break;
			default:
				return;
		}
	}
}

static void i2c_reset_bus(int speed)
{
	int i;
	u16 psc = 0, scll = 0, sclh = 0;
	u16 fsscll = 0, fssclh = 0, hsscll = 0, hssclh = 0;
	unsigned long fclk_rate = 12000000;
	unsigned long internal_clk = 0;
	struct clk *fclk;

	if (i2c_dev->rev >= 0x3c) {
		/*
		 * Enabling all wakup sources to stop I2C freezing on
		 * WFI instruction.
		 * REVISIT: Some wkup sources might not be needed.
		 */
		omap4_i2c_reg_write(I2C_WE, 0x636f);
	}

	/* make sure the bus is disabled */
	omap4_i2c_reg_write(I2C_CON, 0);

	/* reset the bus */
	omap4_i2c_reg_write(I2C_CON, I2C_EN);

	for (i = 10; i > 0; i--) {
		if ((omap4_i2c_reg_read(I2C_SYSS) & I2C_RDONE) != 0)
			break;
		else
			udelay(1000);
	}

	if (i == 0) {
		pr_err("twl-i2c: %s: timeout error on soft-reset\n", __func__);
		return;
	}

	/* disable the bus again and set up some internals */
	omap4_i2c_reg_write(I2C_CON, 0);

	i2c_opmode = 0;

	/* set up the clock */
	if (speed > 400)
		internal_clk = 19200;
	else if (speed > 100)
		internal_clk = 9600;
	else
		internal_clk = 4000;
	fclk = clk_get(i2c_dev->dev, "fck");
	fclk_rate = clk_get_rate(fclk) / 1000;
	clk_put(fclk);
	pr_debug("twl-i2c: fclk_rate %lu\n", fclk_rate);

	/* Compute prescaler divisor */
	psc = fclk_rate / internal_clk;
	psc = psc - 1;

	/* If configured for High speed */
	if (speed > 400) {
		unsigned long scl;

		/* For first phase of HS mode */
		scl = internal_clk / 400;
		fsscll = scl - (scl / 3) - 7;
		fssclh = (scl / 3) - 5;

		/* For second phase of HS mode */
		scl = fclk_rate / speed;
		hsscll = scl - (scl / 3) - 7;
		hssclh = (scl / 3) - 5;

		i2c_opmode = 1;
	} else if (speed > 100) {
		unsigned long scl;

		/* Fast mode */
		scl = internal_clk / speed;
		fsscll = scl - (scl / 3) - 7;
		fssclh = (scl / 3) - 5;
	} else {
		/* Standard mode */
		fsscll = internal_clk / (speed * 2) - 7;
		fssclh = internal_clk / (speed * 2) - 5;
	}
	scll = (hsscll << 8) | fsscll;
	sclh = (hssclh << 8) | fssclh;

	omap4_i2c_reg_write(I2C_PSC, psc);
	omap4_i2c_reg_write(I2C_SCLL, scll);
	omap4_i2c_reg_write(I2C_SCLH, sclh);

	/* enable the bus */
	omap4_i2c_reg_write(I2C_CON, I2C_EN); /* enable, operation mode */

	/* have to enable interrupt or OMAP I2C module doesn't work */
	omap4_i2c_reg_write(I2C_IRQENABLE_SET,
		I2C_XRDY_IE | I2C_RRDY_IE | I2C_ARDY_IE | I2C_NACK_IE | I2C_AL_IE);
	udelay(1000);

	flush_i2c_fifo();
	omap4_i2c_reg_write(I2C_IRQSTATUS, 0xffff);
	omap4_i2c_reg_write(I2C_CNT, 0);
}

static int i2c_transmit(unsigned char address,
		const void *buf, size_t count)
{
	int err = 0;
	int cnt = 0;
	const unsigned char *ptr = buf;
	unsigned short status;

	pr_debug("twl-i2c: %s: address 0x%02x, buf %p, count %d\n", __func__,
			address, buf, count);

	i2c_wait_for_bus_busy();

	omap4_i2c_reg_write(I2C_SA, address);
	omap4_i2c_reg_write(I2C_CNT, count);

	/* clear FIFO buffer */
	omap4_i2c_reg_write(I2C_BUF, (1 << 14) | (1 << 6));

	omap4_i2c_reg_write(I2C_CON,
		I2C_EN | I2C_MST | I2C_TRX | I2C_STP | I2C_STT | i2c_opmode);

	while (1) {
		status = omap4_i2c_reg_read(I2C_IRQSTATUS);
		if (status & I2C_NACK) {
			pr_err("twl-i2c: %s: nack error\n", __func__);
			err = -1;
			goto transmit_out;
		}
		if (status & I2C_AL) {
			pr_err("twl-i2c: %s: al error\n", __func__);
			err = -1;
			goto transmit_out;
		}
		if (status & I2C_ARDY) {
			break;
		}
		if (status & I2C_XRDY) {
			omap4_i2c_reg_write(I2C_DATA, *ptr);
			ptr++;
			cnt++;
		}
		omap4_i2c_reg_write(I2C_IRQSTATUS, status);

		/* FIXME:
		 * timeout?
		 */
	}
	
	err = cnt;

transmit_out:
	flush_i2c_fifo();
	omap4_i2c_reg_write(I2C_IRQSTATUS, 0xffff);
	omap4_i2c_reg_write(I2C_CNT, 0);

	return err;
}

static int i2c_write_reg(unsigned char address,
		unsigned char val, unsigned char reg)
{
	u8 buf[2];

	buf[0] = reg;
	buf[1] = val;

	return i2c_transmit(address, buf, 2);
}

int twl6030_pm_i2c_write_u8(unsigned char val, unsigned char reg)
{
	int err;
	pm_runtime_get_sync(i2c_dev->dev);
	
	disable_irq_nosync(i2c_dev->irq);
	i2c_reset_bus(I2C_STANDARD_MODE);
	err = i2c_write_reg(TWL6030_CHIP_PM, val, TWL6030_BASEADD_PM_MASTER + reg);	
	enable_irq(i2c_dev->irq);

	pm_runtime_put_sync(i2c_dev->dev);

	return err;
}
EXPORT_SYMBOL(twl6030_pm_i2c_write_u8);

static int __init twl_i2c_init(void)
{
	int l;
	int err;
	struct omap_hwmod *oh;
	struct omap_device *od;
	struct omap_hwmod_ocp_if *os;
	char oh_name[MAX_OMAP_I2C_HWMOD_NAME_LEN];

	l = snprintf(oh_name, MAX_OMAP_I2C_HWMOD_NAME_LEN, "i2c1");
	if (l > MAX_OMAP_I2C_HWMOD_NAME_LEN)
		pr_warning("twl-i2c: %s: string buffer overflow in I2C1 device setup\n", __func__);
	oh = omap_hwmod_lookup(oh_name);
	if (!oh) {
		pr_err("twl-i2c: %s: could not look up %s\n", __func__, oh_name);
		return -EEXIST;
	}
	od = oh->od;

	i2c_dev = kzalloc(sizeof(struct twl_i2c_dev), GFP_KERNEL);
	if (!i2c_dev) {
		pr_err("twl-i2c: %s: failed to allocate memory\n", __func__);
		return -ENOMEM;
	}
	i2c_dev->dev = &od->pdev.dev;

	/* irq */
	pr_info("twl-i2c: name %s, irq %d\n", (oh->mpu_irqs)->name,
			(oh->mpu_irqs)->irq);
	i2c_dev->irq = (oh->mpu_irqs)->irq;

	/* physical address */
	os = oh->slaves[0];
	pr_info("twl-i2c: name %s, pa 0x%08x-0x%08x\n",
			(os->addr)->name,
			(os->addr)->pa_start,
			(os->addr)->pa_end);

	/* io address mapping */
	i2c_dev->base = ioremap((os->addr)->pa_start,
		(os->addr)->pa_end - (os->addr)->pa_start + 1);
	if (!i2c_dev->base) {
		pr_err("twl-i2c: %s: failed to remap io address\n", __func__);
		err = -ENOMEM;
		goto err_ioremap;
	}
	pr_info("twl-i2c: va 0x%08x\n", (unsigned int)i2c_dev->base);

	/* already enabled pm_runtime on i2c-omap driver */
	pm_runtime_get_sync(i2c_dev->dev);
	i2c_dev->rev = omap4_i2c_reg_read(I2C_REV) & 0xff;
	pr_info("twl-i2c: rev 0x%02x\n", i2c_dev->rev);
	pm_runtime_put_sync(i2c_dev->dev);

	return 0;

err_ioremap:
	kfree(i2c_dev);
	return err;
}

late_initcall(twl_i2c_init);
