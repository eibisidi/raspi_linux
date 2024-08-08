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
	cbfunc.o \
	eck_ioctl.o \
	period.o \
	ecbus.o \
	slaveinfo.o \
	$(SOEM_ROOT)/soem/ethercatbase.o \
	$(SOEM_ROOT)/soem/ethercatcoe.o \
	$(SOEM_ROOT)/soem/ethercatconfig.o \
	$(SOEM_ROOT)/soem/ethercatdc.o \
	$(SOEM_ROOT)/soem/ethercateoe.o \
	$(SOEM_ROOT)/soem/ethercatfoe.o \
	$(SOEM_ROOT)/soem/ethercatmain.o \
	$(SOEM_ROOT)/soem/ethercatprint.o \
	$(SOEM_ROOT)/soem/ethercatsoe.o \
	$(SOEM_ROOT)/osal/kernel/osal.o \
	$(SOEM_ROOT)/oshw/kernel/nicdrv.o \
	$(SOEM_ROOT)/oshw/kernel/oshw.o \
	$(SOEM_ROOT)/oshw/kernel/net.o
	
