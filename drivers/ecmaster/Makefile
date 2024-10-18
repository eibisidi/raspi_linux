DRV = ecmaster
SOEM_ROOT = SOEM-master
EXPBOT_ROOT = /home/gk/EXPBOT2
obj-m += $(DRV).o

ccflags-y := \
	-I$(src)/$(SOEM_ROOT)/oshw/kernel \
	-I$(src)/$(SOEM_ROOT)/osal/kernel \
	-I$(src)/$(SOEM_ROOT)/osal \
	-I$(src)/$(SOEM_ROOT)/soem \
	-I$(src)\
	-I$(EXPBOT_ROOT)/common \
	-std=gnu99 \
	-Wno-declaration-after-statement

#NOTE: common.c is a symbolic link to $(EXPBOT_ROOT)/common.c
$(DRV)-objs := \
	module.o \
	common.o \
	fops.o \
	eck_debugfs.o \
	eck_cdev.o \
	eck_ioctl.o
	#period.o
	#ecbus.o \
	#slaveinfo.o

