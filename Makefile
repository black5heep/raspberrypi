obj-m += touch.o
touch-objs := rasp_touch_dev.o rasp_touch_driver.o
KDIR := /usr/src/linux-headers-4.9.41-v7+
all:
	make -C $(KDIR) M=$(PWD) modules
clean:
	rm -f *.ko *.o *.mod.o *.mod.c *.symvers modul*