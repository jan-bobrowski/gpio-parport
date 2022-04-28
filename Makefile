NAME = gpio-parport

obj-m := $(NAME).o

KDIR ?= /lib/modules/$(shell uname -r)/build

modules modules_install clean:
	make -C $(KDIR) M=$(PWD) $@
