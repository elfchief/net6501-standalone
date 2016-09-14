/*
 * Generic driver for ioport-mapped GPIO controllers.
 *
 * Originally from gpio-generic, which only handles plain mmap'd io.
 * Original credits for that:
 *     Copyright 2008 MontaVista Software, Inc.
 *     Copyright 2008,2010 Anton Vorontsov <cbouatmailru@gmail.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 * The simplest form of a GPIO controller that the driver supports is
 * just a single "data" register, where GPIO state can be read and/or
 * written.
 *
 * A bit more sophisticated controllers with a pair of set/clear-bit 
 * registers affecting the data register and the output pins are also
 * supported.
 *
 * The driver supports big-endian notation, just register the device
 * with the -be suffix.
 *
 * 8, 16, 32, and 64-bit registers are (eventually) supported, and
 * the number of GPIOs is determined by the width of the registers.
 *
 * This module could probably be somewhat trivially merged with 
 * gpio-generic if that was desirable.
 */

// FIXME: Probably don't need all these
#include <linux/init.h>
#include <linux/err.h>
#include <linux/bug.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/compiler.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/log2.h>
#include <linux/ioport.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/mod_devicetable.h>
#include "basic_ioport_gpio.h"

static void pgpio_write8(void __iomem *reg, unsigned long data)
{
    iowrite8(data, reg);
}

static unsigned long pgpio_read8(void __iomem *reg)
{
    return ioread8(reg);
}

static void pgpio_write16(void __iomem *reg, unsigned long data)
{
    iowrite16(data, reg);
}

static unsigned long pgpio_read16(void __iomem *reg)
{
    return ioread16(reg);
}

static void pgpio_write32(void __iomem *reg, unsigned long data)
{
    iowrite32(data, reg);
}

static unsigned long pgpio_read32(void __iomem *reg)
{
    return ioread32(reg);
}

// #if BITS_PER_LONG >= 64
#if 0
static void pgpio_write64(void __iomem *reg, unsigned long data)
{
    iowrite64(data, reg);
}

static unsigned long pgpio_read64(void __iomem *reg)
{
    return ioread64(reg);
}
#endif /* BITS_PER_LONG >= 64 */

static void pgpio_write16be(void __iomem *reg, unsigned long data)
{
    iowrite16be(data, reg);
}

static unsigned long pgpio_read16be(void __iomem *reg)
{
    return ioread16be(reg);
}

static void pgpio_write32be(void __iomem *reg, unsigned long data)
{
    iowrite32be(data, reg);
}

static unsigned long pgpio_read32be(void __iomem *reg)
{
    return ioread32be(reg);
}

static unsigned long pgpio_pin2mask(struct pgpio_chip *pgc, unsigned int pin)
{
    return 1 << pin;
}

static unsigned long pgpio_pin2mask_be(struct pgpio_chip *pgc,
                       unsigned int pin)
{
    return 1 << (pgc->bits - 1 - pin);
}

static int pgpio_get(struct gpio_chip *gc, unsigned int gpio)
{
    struct pgpio_chip *pgc = to_pgpio_chip(gc);

    return pgc->read_reg(pgc->reg_dat) & pgc->pin2mask(pgc, gpio);
}

static void pgpio_set(struct gpio_chip *gc, unsigned int gpio, int val)
{
    struct pgpio_chip *pgc = to_pgpio_chip(gc);
    unsigned long mask = pgc->pin2mask(pgc, gpio);
    unsigned long flags;

    spin_lock_irqsave(&pgc->lock, flags);

    if (val)
        pgc->data |= mask;
    else
        pgc->data &= ~mask;

    pgc->write_reg(pgc->reg_dat, pgc->data);

    spin_unlock_irqrestore(&pgc->lock, flags);
}

static void pgpio_set_with_clear(struct gpio_chip *gc, unsigned int gpio,
                 int val)
{
    struct pgpio_chip *pgc = to_pgpio_chip(gc);
    unsigned long mask = pgc->pin2mask(pgc, gpio);

    if (val)
        pgc->write_reg(pgc->reg_set, mask);
    else
        pgc->write_reg(pgc->reg_clr, mask);
}

static void pgpio_set_set(struct gpio_chip *gc, unsigned int gpio, int val)
{
    struct pgpio_chip *pgc = to_pgpio_chip(gc);
    unsigned long mask = pgc->pin2mask(pgc, gpio);
    unsigned long flags;

    spin_lock_irqsave(&pgc->lock, flags);

    if (val)
        pgc->data |= mask;
    else
        pgc->data &= ~mask;

    pgc->write_reg(pgc->reg_set, pgc->data);

    spin_unlock_irqrestore(&pgc->lock, flags);
}

static int pgpio_simple_dir_in(struct gpio_chip *gc, unsigned int gpio)
{
    return 0;
}

static int pgpio_simple_dir_out(struct gpio_chip *gc, unsigned int gpio,
                int val)
{
    gc->set(gc, gpio, val);

    return 0;
}

static int pgpio_dir_in(struct gpio_chip *gc, unsigned int gpio)
{
    struct pgpio_chip *pgc = to_pgpio_chip(gc);
    unsigned long flags;

    spin_lock_irqsave(&pgc->lock, flags);

    pgc->dir &= ~pgc->pin2mask(pgc, gpio);
    pgc->write_reg(pgc->reg_dir, pgc->dir);

    spin_unlock_irqrestore(&pgc->lock, flags);

    return 0;
}

static int pgpio_dir_out(struct gpio_chip *gc, unsigned int gpio, int val)
{
    struct pgpio_chip *pgc = to_pgpio_chip(gc);
    unsigned long flags;

    gc->set(gc, gpio, val);

    spin_lock_irqsave(&pgc->lock, flags);

    pgc->dir |= pgc->pin2mask(pgc, gpio);
    pgc->write_reg(pgc->reg_dir, pgc->dir);

    spin_unlock_irqrestore(&pgc->lock, flags);

    return 0;
}

static int pgpio_dir_in_inv(struct gpio_chip *gc, unsigned int gpio)
{
    struct pgpio_chip *pgc = to_pgpio_chip(gc);
    unsigned long flags;

    spin_lock_irqsave(&pgc->lock, flags);

    pgc->dir |= pgc->pin2mask(pgc, gpio);
    pgc->write_reg(pgc->reg_dir, pgc->dir);

    spin_unlock_irqrestore(&pgc->lock, flags);

    return 0;
}

static int pgpio_dir_out_inv(struct gpio_chip *gc, unsigned int gpio, int val)
{
    struct pgpio_chip *pgc = to_pgpio_chip(gc);
    unsigned long flags;

    gc->set(gc, gpio, val);

    spin_lock_irqsave(&pgc->lock, flags);

    pgc->dir &= ~pgc->pin2mask(pgc, gpio);
    pgc->write_reg(pgc->reg_dir, pgc->dir);

    spin_unlock_irqrestore(&pgc->lock, flags);

    return 0;
}

static int pgpio_setup_accessors(struct device *dev,
                 struct pgpio_chip *pgc,
                 bool bit_be,
                 bool byte_be)
{

    switch (pgc->bits) {
    case 8:
        pgc->read_reg   = pgpio_read8;
        pgc->write_reg  = pgpio_write8;
        break;
    case 16:
        if (byte_be) {
            pgc->read_reg   = pgpio_read16be;
            pgc->write_reg  = pgpio_write16be;
        } else {
            pgc->read_reg   = pgpio_read16;
            pgc->write_reg  = pgpio_write16;
        }
        break;
    case 32:
        if (byte_be) {
            pgc->read_reg   = pgpio_read32be;
            pgc->write_reg  = pgpio_write32be;
        } else {
            pgc->read_reg   = pgpio_read32;
            pgc->write_reg  = pgpio_write32;
        }
        break;
// #if BITS_PER_LONG >= 64
#if 0
    case 64:
        if (byte_be) {
            dev_err(dev,
                "64 bit big endian byte order unsupported\n");
            return -EINVAL;
        } else {
            pgc->read_reg   = pgpio_read64;
            pgc->write_reg  = pgpio_write64;
        }
        break;
#endif /* BITS_PER_LONG >= 64 */
    default:
        dev_err(dev, "unsupported data width %u bits\n", pgc->bits);
        return -EINVAL;
    }

    pgc->pin2mask = bit_be ? pgpio_pin2mask_be : pgpio_pin2mask;

    return 0;
}

/*
 * Create the device and allocate the resources.  For setting GPIO's there are
 * three supported configurations:
 *
 *  - single input/output register resource (named "dat").
 *  - set/clear pair (named "set" and "clr").
 *  - single output register resource and single input resource ("set" and
 *  dat").
 *
 * For the single output register, this drives a 1 by setting a bit and a zero
 * by clearing a bit.  For the set clr pair, this drives a 1 by setting a bit
 * in the set register and clears it by setting a bit in the clear register.
 * The configuration is detected by which resources are present.
 *
 * For setting the GPIO direction, there are three supported configurations:
 *
 *  - simple bidirection GPIO that requires no configuration.
 *  - an output direction register (named "dirout") where a 1 bit
 *  indicates the GPIO is an output.
 *  - an input direction register (named "dirin") where a 1 bit indicates
 *  the GPIO is an input.
 */
static int pgpio_setup_io(struct pgpio_chip *pgc,
              void __iomem *dat,
              void __iomem *set,
              void __iomem *clr)
{

    pgc->reg_dat = dat;
    if (!pgc->reg_dat)
        return -EINVAL;

    if (set && clr) {
        pgc->reg_set = set;
        pgc->reg_clr = clr;
        pgc->gc.set = pgpio_set_with_clear;
    } else if (set && !clr) {
        pgc->reg_set = set;
        pgc->gc.set = pgpio_set_set;
    } else {
        pgc->gc.set = pgpio_set;
    }

    pgc->gc.get = pgpio_get;

    return 0;
}

static int pgpio_setup_direction(struct pgpio_chip *pgc,
                 void __iomem *dirout,
                 void __iomem *dirin)
{
    if (dirout && dirin) {
        return -EINVAL;
    } else if (dirout) {
        pgc->reg_dir = dirout;
        pgc->gc.direction_output = pgpio_dir_out;
        pgc->gc.direction_input = pgpio_dir_in;
    } else if (dirin) {
        pgc->reg_dir = dirin;
        pgc->gc.direction_output = pgpio_dir_out_inv;
        pgc->gc.direction_input = pgpio_dir_in_inv;
    } else {
        pgc->gc.direction_output = pgpio_simple_dir_out;
        pgc->gc.direction_input = pgpio_simple_dir_in;
    }

    return 0;
}

int pgpio_remove(struct pgpio_chip *pgc)
{
    return gpiochip_remove(&pgc->gc);
}
EXPORT_SYMBOL_GPL(pgpio_remove);

int pgpio_init(struct pgpio_chip *pgc, struct device *dev,
           unsigned long sz, void __iomem *dat, void __iomem *set,
           void __iomem *clr, void __iomem *dirout, void __iomem *dirin,
           unsigned long flags)
{
    int ret;

    if (!is_power_of_2(sz))
        return -EINVAL;

    pgc->bits = sz * 8;
    if (pgc->bits > BITS_PER_LONG)
        return -EINVAL;

    spin_lock_init(&pgc->lock);
    pgc->gc.dev = dev;
    pgc->gc.label = dev_name(dev);
    pgc->gc.base = -1;
    pgc->gc.ngpio = pgc->bits;

    ret = pgpio_setup_io(pgc, dat, set, clr);
    if (ret)
        return ret;

    ret = pgpio_setup_accessors(dev, pgc, flags & PGPIOF_BIG_ENDIAN,
                    flags & PGPIOF_BIG_ENDIAN_BYTE_ORDER);
    if (ret)
        return ret;

    ret = pgpio_setup_direction(pgc, dirout, dirin);
    if (ret)
        return ret;

    pgc->data = pgc->read_reg(pgc->reg_dat);
    if (pgc->gc.set == pgpio_set_set &&
            !(flags & PGPIOF_UNREADABLE_REG_SET))
        pgc->data = pgc->read_reg(pgc->reg_set);
    if (pgc->reg_dir && !(flags & PGPIOF_UNREADABLE_REG_DIR))
        pgc->dir = pgc->read_reg(pgc->reg_dir);

    return ret;
}
EXPORT_SYMBOL_GPL(pgpio_init);

#ifdef CONFIG_GPIO_GENERIC_PLATFORM_IOPORT

static void __iomem *pgpio_map(struct platform_device *pdev,
                    struct resource *base,
                    const char *name,
                    int *err)
{
    struct device *dev = &pdev->dev;
    struct resource *r;
    resource_size_t sz, offset;
    void __iomem *ret;

    *err = 0;

    r = platform_get_resource_byname(pdev, IORESOURCE_REG, name);
    if (!r)
        return NULL;

    sz = resource_size(r);
    offset = r->start;

    if (!devm_request_region(dev, base->start + offset, sz, r->name)) {
        *err = -EBUSY;
        return NULL;
    }

    ret = ioport_map(base->start + offset, sz);
    if (!ret) {
        *err = -ENOMEM;
        return NULL;
    }

    return ret;
}

static int pgpio_pdev_probe(struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct resource *r;
    void __iomem *dat;
    void __iomem *set;
    void __iomem *clr;
    void __iomem *dirout;
    void __iomem *dirin;
    unsigned long flags = 0;
    int err;
    struct pgpio_chip *pgc;
    struct pgpio_pdata *pdata = dev_get_platdata(dev);

    if (!pdata || pdata->ngpio <= 0) {
        return -EINVAL;
    }

    r = platform_get_resource_byname(pdev, IORESOURCE_IO, "gpio");
    if (!r)
        return -EINVAL;

    dat = pgpio_map(pdev, r, "dat", &err);
    if (!dat)
        return err;

    set = pgpio_map(pdev, r, "set", &err);
    if (err)
        return err;

    clr = pgpio_map(pdev, r, "clr", &err);
    if (err)
        return err;

    dirout = pgpio_map(pdev, r, "dirout", &err);
    if (err)
        return err;

    dirin = pgpio_map(pdev, r, "dirin", &err);
    if (err)
        return err;

    if (!strcmp(platform_get_device_id(pdev)->name, "basic-ioport-gpio-be"))
        flags |= PGPIOF_BIG_ENDIAN;

    pgc = devm_kzalloc(&pdev->dev, sizeof(*pgc), GFP_KERNEL);
    if (!pgc)
        return -ENOMEM;

    err = pgpio_init(pgc, dev, pdata->ngpio / 8, dat, set, clr, dirout, dirin, flags);
    if (err)
        return err;

    pgc->gc.base = pdata->base;

    platform_set_drvdata(pdev, pgc);

    return gpiochip_add(&pgc->gc);
}

static int pgpio_pdev_remove(struct platform_device *pdev)
{
    struct pgpio_chip *pgc = platform_get_drvdata(pdev);

    return pgpio_remove(pgc);
}

static const struct platform_device_id pgpio_id_table[] = {
    { "basic-ioport-gpio", },
    { "basic-ioport-gpio-be", },
    {},
};
MODULE_DEVICE_TABLE(platform, pgpio_id_table);

static struct platform_driver pgpio_driver = {
    .driver = {
        .name = "basic-ioport-gpio",
    },
    .id_table = pgpio_id_table,
    .probe    = pgpio_pdev_probe,
    .remove   = pgpio_pdev_remove,
};

module_platform_driver(pgpio_driver);

#endif /* CONFIG_GPIO_GENERIC_PLATFORM_IOPORT */

MODULE_DESCRIPTION("Driver for basic ioport-mapped GPIO controllers");
MODULE_AUTHOR("J. Grizzard <jg-github@lupine.org>");
MODULE_LICENSE("GPL");
