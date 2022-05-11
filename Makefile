NAME = gpio-parport

obj-m := $(NAME).o

KVERSION ?= $(shell uname -r)
KDIR ?= /lib/modules/$(KVERSION)/build

#cflags := -DDEBUG
cflags := -DGPIO_LINE_DIRECTION_OUT=0 -DGPIO_LINE_DIRECTION_IN=1

modules:
	make -C $(KDIR) M=$(PWD) $(and $(cflags),CFLAGS_MODULE="$(cflags)") $@

modules_install clean:
	make -C $(KDIR) M=$(PWD) $@
