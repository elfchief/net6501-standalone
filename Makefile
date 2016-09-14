# Comment/uncomment the following line to disable/enable debugging
#DEBUG = y

# Add your debugging flag (or not) to EXTRA_CFLAGS
ifeq ($(DEBUG),y)
  DEBFLAGS = -O -g -DSHORT_DEBUG # "-O" is needed to expand inlines
else
  DEBFLAGS = -O2
endif

EXTRA_CFLAGS += $(DEBFLAGS)
EXTRA_CFLAGS += -I..
EXTRA_CFLAGS += -DCONFIG_GPIO_GENERIC_PLATFORM_IOPORT

ifneq ($(KERNELRELEASE),)
# call from kernel build system

obj-m   := soekris-net6501.o gpio-ioport.o leds-net6501.o

else

KERNELDIR ?= /lib/modules/$(shell uname -r)/build
PWD       := $(shell pwd)
 
default:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules

endif

load:
	/sbin/insmod ./soekris-net6501.ko &&\
	/sbin/insmod ./leds-net6501.ko &&\
	/sbin/insmod ./gpio-ioport.ko

unload:
	/sbin/rmmod gpio-ioport &&\
	/sbin/rmmod leds-net6501 &&\
	/sbin/rmmod soekris-net6501

clean:
	rm -rf *.o *~ core .depend .*.cmd *.ko *.mod.c .tmp_versions Module.symvers modules.order

depend .depend dep:
	$(CC) $(EXTRA_CFLAGS) -M *.c > .depend


ifeq (.depend,$(wildcard .depend))
include .depend
endif
