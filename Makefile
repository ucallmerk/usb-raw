obj-m := pic32-usb.o

KERNEL_DIR = /lib/modules/$(shell uname -r)/build/
PWD = $(shell pwd)
# ccflags-y := -ggdb
all:
	make -C ${KERNEL_DIR} M=${PWD} modules
clean:
	rm -rf *.o *.ko *.s *.symvers *.mod *.mod.* *.order
