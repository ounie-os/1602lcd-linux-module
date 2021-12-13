PWD := $(shell pwd)

export CROSS_COMPILE := /opt/gcc-linaro-5.4.1-2017.05-x86_64_arm-linux-gnueabihf/bin/arm-linux-gnueabihf-
export ARCH := arm
export CC := $(CROSS_COMPILE)gcc

all:
	$(MAKE) -C $(PWD)/module

app:
	$(MAKE) -C $(PWD)/app
	
.PHONY: clean appclean app

clean: appclean
	$(MAKE) -C $(PWD)/module clean

appclean:
	$(MAKE) -C $(PWD)/app clean