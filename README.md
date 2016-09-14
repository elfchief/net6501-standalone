# net6501-standalone
Drivers for Soekris net6501 LEDs and GPIO lines, standalone version

For all the specific details, please see the [main repository](https://github.com/elfchief/net6501-centos7-kernel). This repo has a "I just wanna build the modules and don't care about patching a kernel tree" setup that will hopefully make it easy for existing systems to build and load. I hope.

Basic usage:

```
# make
make -C /lib/modules/3.10.0-327.28.2.2.el7.wwp_soekris.x86_64/build M=/tmp/net6501 modules
make[1]: Entering directory `/usr/src/kernels/3.10.0-327.28.2.2.el7.wwp_soekris.x86_64'
  CC [M]  /tmp/net6501/soekris-net6501.o
  CC [M]  /tmp/net6501/gpio-ioport.o
  CC [M]  /tmp/net6501/leds-net6501.o
  Building modules, stage 2.
  MODPOST 3 modules
  CC      /tmp/net6501/gpio-ioport.mod.o
  LD [M]  /tmp/net6501/gpio-ioport.ko
  CC      /tmp/net6501/leds-net6501.mod.o
  LD [M]  /tmp/net6501/leds-net6501.ko
  CC      /tmp/net6501/soekris-net6501.mod.o
  LD [M]  /tmp/net6501/soekris-net6501.ko
make[1]: Leaving directory `/usr/src/kernels/3.10.0-327.28.2.2.el7.wwp_soekris.x86_64'

# make load
Loading net6501 kernel modules...
All modules loaded successfully.

# make unload
Unloading net6501 kernel modules...
All modules unloaded successfully.
```
