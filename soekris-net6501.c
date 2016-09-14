/*
 * System Specific setup for Soekris net6501
 * This means setup of GPIO lines and LEDs on net6501 boards.
 * 
 * FIXME: Do we have a way to add the HPET-force-enable to this, too?
 * 
 * Based on original work by Chris Wilson <chris+soekris@aptivate.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/string.h>
#include <linux/module.h>
#include <linux/leds.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include "basic_ioport_gpio.h"

#define BIOS_REGION_BASE        0x000f0000
#define BIOS_REGION_SIZE        0x00010000

#define BOARD_STRING            "net6501"
#define NET6501_GPIO_DEVICE     "basic-ioport-gpio"
#define NET6501_LEDS_DEVICE     "leds-net6501"

#define NET6501_GPIO_BASE       0x0680   /* Base address */
#define NET6501_GPIO_INPUT_OFF  0x00     /* Input register */
#define NET6501_GPIO_OUTPUT_OFF 0x04     /* Output register */
#define NET6501_GPIO_RESET_OFF  0x08     /* Reset output */
#define NET6501_GPIO_SET_OFF    0x0c     /* Set output*/
#define NET6501_GPIO_DIR_OFF    0x10     /* Direction register offset (set for output) */
#define NET6501_GPIO_LEN        20
#define NET6501_NGPIO           16

#define NET6501_LEDS_BASE       0x069C   /* Start of LEDS */
#define NET6501_LEDS_ERROR_OFF  0x00     /* Error LED */
#define NET6501_LEDS_READY_OFF  0x01     /* Ready LED */
#define NET6501_LEDS_LEN        2

static void net6501_release(struct device *);


#if 0    // Not actually using these resources in the LED driver right now
static struct platform_device net6501_leds_device = {
    .name           = NET6501_LEDS_DEVICE,
    .id             = -1,
    .resource       = net6501_gpio_resources,
    .num_resources  = ARRAY_SIZE(net6501_gpio_resources),
};

static struct resource net6501_leds_resources[] = {
    {
        .name    = "leds",
        .start    = NET6501_LEDS_BASE,
        .end    = NET6501_LEDS_BASE + NET6501_LEDS_LEN - 1,
        .flags    = IORESOURCE_IO,
    },
    {
        .name    = "error",
        .start    = NET6501_LEDS_ERROR_OFF,
        .end    = 0,
        .flags    = IORESOURCE_REG,
    },
    {
        .name    = "ready",
        .start    = NET6501_LEDS_READY_OFF,
        .end     = 0,
        .flags    = IORESOURCE_REG,
    }
};
#endif

static struct resource net6501_gpio_resources[] = {
    {
        .name   = "gpio",
        .start  = NET6501_GPIO_BASE,
        .end    = NET6501_GPIO_BASE + NET6501_GPIO_LEN - 1,
        .flags  = IORESOURCE_IO,
    },
    {
        .name   = "dat",
        .start  = NET6501_GPIO_INPUT_OFF,
        .end    = NET6501_GPIO_INPUT_OFF + (NET6501_NGPIO / 8) - 1,
        .flags  = IORESOURCE_REG,
    },
    {
        .name   = "set",
        .start  = NET6501_GPIO_SET_OFF,
        .end    = NET6501_GPIO_SET_OFF + (NET6501_NGPIO / 8) - 1,
        .flags  = IORESOURCE_REG,
    },
    {
        .name   = "clr",
        .start  = NET6501_GPIO_RESET_OFF,
        .end    = NET6501_GPIO_RESET_OFF + (NET6501_NGPIO / 8) - 1,
        .flags  = IORESOURCE_REG,
    },
    {
        .name   = "dirout",
        .start  = NET6501_GPIO_DIR_OFF,
        .end    = NET6501_GPIO_DIR_OFF + (NET6501_NGPIO / 8) - 1,
        .flags  = IORESOURCE_REG,
    },
};

static struct pgpio_pdata net6501_gpio_pdata = {
    .base = -1,
    .ngpio = NET6501_NGPIO,
};

static struct platform_device net6501_gpio_device = {
    .name           = NET6501_GPIO_DEVICE,
    .id             = -1,
    .resource       = net6501_gpio_resources,
    .num_resources  = ARRAY_SIZE(net6501_gpio_resources),
    .dev            = {
        .platform_data  = &net6501_gpio_pdata,
        .release    = &net6501_release,
    },
};

static struct platform_device *net6501_devices[] = {
    &net6501_gpio_device,
};


static struct platform_device *leds_device;
static void __init register_net6501(void)
{
    platform_add_devices(net6501_devices, ARRAY_SIZE(net6501_devices));
    leds_device = platform_device_register_simple(NET6501_LEDS_DEVICE, -1, NULL, 0);
    printk(KERN_INFO "%s: Soekris net6501 board platform devices created\n", KBUILD_MODNAME);
}

/*
 * Search for the board string in the BIOS region. This is sloppy, but
 * I'm not sure of a better way.
 */
static bool __init net6501_present(void)
{
    unsigned char *rombase, *i;
    bool found = false;

    rombase = ioremap(BIOS_REGION_BASE, BIOS_REGION_SIZE - 1);
    if (!rombase) {
        printk(KERN_ERR "%s: failed to get rombase\n", KBUILD_MODNAME);
        return found;
    }

    for (i = rombase; i <= (rombase + BIOS_REGION_SIZE - 7); i++) {
        if (!memcmp(i, BOARD_STRING, 7)) {
            printk(KERN_INFO "%s: system recognized as soekris net6501\n", KBUILD_MODNAME);
            found = true;
            break;
        }
    }

    iounmap(rombase);
    return found;
}

static int __init net6501_init(void)
{
    if (!net6501_present())
        return 0;

    register_net6501();

    return 0;
}

static void net6501_exit(void)
{
    int i;

    for (i = 0; i < ARRAY_SIZE(net6501_devices); i++) {
        platform_device_unregister(net6501_devices[i]);
    }

    platform_device_unregister(leds_device);
}

/* I don't -think- we need anything here */
static void net6501_release(struct device *dev)
{
    return;
}

module_init(net6501_init);
module_exit(net6501_exit);

MODULE_AUTHOR("J. Grizzard <jg-github@lupine.org>");
MODULE_DESCRIPTION("Soekris net6501 platform device setup");
MODULE_LICENSE("GPL");
