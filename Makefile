ifeq ($(KERNELRELEASE), )

KERNELDIR := /lib/modules/$(shell uname -r)/build

PWD :=$(shell pwd)
default:
	$(MAKE) -C $(KERNELDIR)  M=$(PWD) 
clean:
	rm -rf .tmp_versions Module.symvers *.mod.c *.o *.ko .*.cmd Module.markers modules.order
load:
	modprobe usbserial
	insmod ch34x.ko
unload:
	rmmod ch34x
install:
	cp ch34x.ko /lib/modules/$(shell uname -r)/kernel/drivers/usb/serial
	depmod
else
	obj-m := ch34x.o
endif
