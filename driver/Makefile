obj-m := fl2000drm.o
fl2000drm-y := fl2000_drv.o fl2000_hw.o fl2000_ite.o

KDIR ?= /lib/modules/$(shell uname -r)/build

all:
	$(MAKE) -C $(KDIR) M=$(CURDIR) modules

clean:
	$(MAKE) -C $(KDIR) M=$(CURDIR) clean

install: all
	$(MAKE) -C $(KDIR) M=$(CURDIR) modules_install
	depmod -a
