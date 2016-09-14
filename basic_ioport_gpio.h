/*
 * Basic ioport-mapped GPIO controllers. HEAVILY based on basic_mmio_gpio.h
 *
 * Original Copyright 2008 MontaVista Software, Inc.
 * Original Copyright 2008,2010 Anton Vorontsov <cbouatmailru@gmail.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef __BASIC_PORTIO_GPIO_H
#define __BASIC_PORTIO_GPIO_H

#include <linux/gpio.h>
#include <linux/types.h>
#include <linux/compiler.h>
#include <linux/spinlock_types.h>

struct pgpio_pdata {
        int base;
        int ngpio;
};

struct device;

struct pgpio_chip {
        struct gpio_chip gc;

        unsigned long (*read_reg)(void __iomem *reg);
        void (*write_reg)(void __iomem *reg, unsigned long data);

        void __iomem *reg_dat;
        void __iomem *reg_set;
        void __iomem *reg_clr;
        void __iomem *reg_dir;

        /* Number of bits (GPIOs): <register width> * 8. */
        int bits;

        /*
         * Some GPIO controllers work with the big-endian bits notation,
         * e.g. in a 8-bits register, GPIO7 is the least significant bit.
         */
        unsigned long (*pin2mask)(struct pgpio_chip *pgc, unsigned int pin);

        /*
         * Used to lock pgpio_chip->data. Also, this is needed to keep
         * shadowed and real data registers writes together.
         */
        spinlock_t lock;

        /* Shadowed data register to clear/set bits safely. */
        unsigned long data;

        /* Shadowed direction registers to clear/set direction safely. */
        unsigned long dir;
};

static inline struct pgpio_chip *to_pgpio_chip(struct gpio_chip *gc)
{
        return container_of(gc, struct pgpio_chip, gc);
}

int pgpio_remove(struct pgpio_chip *bgc);
int pgpio_init(struct pgpio_chip *bgc, struct device *dev,
               unsigned long sz, void __iomem *dat, void __iomem *set,
               void __iomem *clr, void __iomem *dirout, void __iomem *dirin,
               unsigned long flags);

#define PGPIOF_BIG_ENDIAN               BIT(0)
#define PGPIOF_UNREADABLE_REG_SET       BIT(1) /* reg_set is unreadable */
#define PGPIOF_UNREADABLE_REG_DIR       BIT(2) /* reg_dir is unreadable */
#define PGPIOF_BIG_ENDIAN_BYTE_ORDER    BIT(3)

#endif /* __BASIC_IOPORT_GPIO_H */
