ARMGNU=aarch64-linux-gnu
DESTELF=fakeimage.elf
DESTBIN=fakeimage.bin

ASM_FILES = $(wildcard *.s)			#make file wildcard

#$(var:pattern=replacement) is eq. to $(patsubst pattern,replacement,$(var))
OBJ_FILES = $(ASM_FILES:%.s=%.o)

%.o : %.s
	$(ARMGNU)-as -o $@ $<

$(DESTELF) : $(OBJ_FILES)
	$(ARMGNU)-ld -T linker.ld -o $@ $^   #ld with link script

$(DESTBIN) : $(DESTELF)
	$(ARMGNU)-objcopy $(DESTELF) -O binary $(DESTBIN) 

