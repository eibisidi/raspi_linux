ARMGNU=aarch64-linux-gnu
DESTELF=fakeimage.elf
DESTBIN=../../fakeimage.bin

DEBUG=-g
ASM_FILES = $(wildcard *.s)			#make file wildcard
C_FILES = $(wildcard *.c)

#$(var:pattern=replacement) is eq. to $(patsubst pattern,replacement,$(var))
OBJ_ASM_FILES = $(ASM_FILES:%.s=%_s.o)
OBJ_C_FILES = $(C_FILES:%.c=%_c.o)

all : $(DESTELF) $(DESTBIN)

%_s.o : %.s
	$(ARMGNU)-as -o $@ $<

%_c.o : %.c
	$(ARMGNU)-gcc $(DEBUG) -c -o $@ $<

$(DESTELF) : $(OBJ_ASM_FILES) $(OBJ_C_FILES)
	$(ARMGNU)-ld -T linker.ld -o $@ $^   #ld with link script

$(DESTBIN) : $(DESTELF)
	$(ARMGNU)-objcopy $(DESTELF) -O binary $(DESTBIN) 

.PHONY : clean

clean:
	rm $(OBJ_ASM_FILES) $(OBJ_C_FILES) $(DESTBIN)  $(DESTELF) 

