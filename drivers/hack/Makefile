DRV = hack
obj-m += $(DRV).o

ccflags-y := \
	-I$(src)\
	-std=gnu99 \
	-Wno-declaration-after-statement

$(DRV)-objs := \
	module.o 
	
