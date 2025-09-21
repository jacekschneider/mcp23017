obj-m += pinctrl-mcp23017_i2c.o

ccflags-y += -std=gnu11

all:
		make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules
clean:
		make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean