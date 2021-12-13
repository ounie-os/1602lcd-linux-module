obj-m := lcd1602.o
KERNELDIR ?= /mnt/disk2/build_desktop_for_arm/output/build/linux-4.19.94-ti-r34
PWD := $(shell pwd)

all:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules

.PHONY: clean

clean:
	rm -rf *.o *.ko *.mod.c Module.symvers modules.order .*cmd .tmp_versions