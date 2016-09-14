/*
 * LED devices for the Soekris Net6501
 * http://www.mail-archive.com/soekris-tech@lists.soekris.com/msg06738.html
 *
 * Copyright 2012 Chris Wilson <chris+soekris@aptivate.org>
 * based on leds-hp6xx.c by Kristoffer Ericson <kristoffer.ericson@gmail.com>
 * parts based on gdrom.c by Adrian McMenamin <adrian@mcmen.demon.co.uk>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/leds.h>

/*
 * These magic numbers come from a post by Soren Kristensen (the board
 * designer?) on the soekris mailing list:
 * http://www.mail-archive.com/soekris-tech@lists.soekris.com/msg06738.html
 */ 
#define LED_ERROR 0x069C
#define LED_READY 0x069D
#define REGION_START 0x069C
#define REGION_LEN 2

#define DRIVER_NAME "leds-net6501"

static void net6501_led_error_set(struct led_classdev *led_cdev,
	enum led_brightness value)
{
	u8 old_value, new_value;

	old_value = inb(LED_ERROR);
	new_value = (old_value & ~1) | (value ? 1 : 0);
	outb(new_value, LED_ERROR);
}

static void net6501_led_ready_set(struct led_classdev *led_cdev,
	enum led_brightness value)
{
	u8 old_value, new_value;

	old_value = inb(LED_READY);
	new_value = (old_value & ~1) | (value ? 1 : 0);
	outb(new_value, LED_READY);
}

static struct led_classdev net6501_led_error = {
	.name			= "net6501:red:error",
	.default_trigger	= "timer",
	.brightness_set		= net6501_led_error_set,
	.flags			= LED_CORE_SUSPENDRESUME,
};

static struct led_classdev net6501_led_ready = {
	.name			= "net6501:green:ready",
	.default_trigger	= NULL,
	.brightness_set		= net6501_led_ready_set,
	.flags			= LED_CORE_SUSPENDRESUME,
};

static int net6501_leds_probe(struct platform_device *pdev)
{
	int ret;

	if (!request_region(REGION_START, REGION_LEN, DRIVER_NAME))
	{
		printk(KERN_WARNING DRIVER_NAME ": I/O region 0x%x-0x%x "
			"is busy\n", REGION_START,
			REGION_START + REGION_LEN - 1);
		return -EBUSY;
	}

	ret = led_classdev_register(&pdev->dev, &net6501_led_error);
	if (ret < 0)
	{
		printk(KERN_WARNING DRIVER_NAME ": failed to register "
			"error LED device: error %d\n", ret);
		goto release_registered_region;
	}

	ret = led_classdev_register(&pdev->dev, &net6501_led_ready);
	if (ret < 0)
	{
		printk(KERN_WARNING DRIVER_NAME ": failed to register "
			"ready LED device: error %d\n", ret);
		goto unregister_led_error;
	}

	return 0;

unregister_led_error:
	led_classdev_unregister(&net6501_led_error);

release_registered_region:
	release_region(REGION_START, REGION_LEN);

	return ret;
}

static int net6501_leds_remove(struct platform_device *pdev)
{
	led_classdev_unregister(&net6501_led_ready);
	led_classdev_unregister(&net6501_led_error);
	release_region(REGION_START, REGION_LEN);
	return 0;
}

static struct platform_driver net6501_leds_driver = {
	.probe		= net6501_leds_probe,
	.remove		= net6501_leds_remove,
	.driver		= {
		.name		= DRIVER_NAME,
		.owner		= THIS_MODULE,
	},
};

module_platform_driver(net6501_leds_driver);

MODULE_AUTHOR("Chris Wilson <chris+soekris@aptivate.org>");
MODULE_DESCRIPTION("Soekris Net6051 LED driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRIVER_NAME);
