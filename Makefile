# Makefile copied from avr-libc's example "demo"
PRG            = hover
OBJ            = hover.o dbg_putstring.o dbg_putchar.o
MCU_TARGET     = attiny861a
OPTIMIZE       = -Os
PROGRAMMER     = avrdude
PROGRAMMER_MCU = t861
PROGRAMMER_DEV = usbtiny # change this to match your programmer device!

# build options, you must run 'make clean' before 'make' after changing these!
# F_CPU: cpu speed, either 1000000 (1MHz) or 8000000 (8MHz)
F_CPU          = 1000000
# DEBUG_LOG: enable debug logging, either 1 or 0
DEBUG_LOG      = 0

DEFS           = -DF_CPU=$(F_CPU) -DDEBUG_LOG=$(DEBUG_LOG)

ifeq ($(F_CPU),1000000)
# lfuse for 1Mhz clock
LFUSE          = 0x62
else
# lfuse for 8Mhz clock
LFUSE          = 0xE2
endif

LIBS           =

# You should not have to change anything below here.

CC             = avr-gcc

# Override is only needed by avr-lib build system.

override CFLAGS        = -g -Wall $(OPTIMIZE) -mmcu=$(MCU_TARGET) $(DEFS)
override LDFLAGS       = -Wl,-Map,$(PRG).map

OBJCOPY        = avr-objcopy
OBJDUMP        = avr-objdump

program : $(PRG).hex
		avr-size $(PRG).elf
		$(PROGRAMMER) -c $(PROGRAMMER_DEV) -p $(PROGRAMMER_MCU) -e
		$(PROGRAMMER) -V -c $(PROGRAMMER_DEV) -p $(PROGRAMMER_MCU) -U lfuse:w:$(LFUSE):m
		$(PROGRAMMER) -V -c $(PROGRAMMER_DEV) -p $(PROGRAMMER_MCU) -U flash:w:$(PRG).hex

all: $(PRG).elf lst text eeprom

$(PRG).elf: $(OBJ)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ $(LIBS)

# dependency:
hover.o: dbg_putchar.h dbg_putstring.h
dbg_putstring.o: dbg_putstring.h dbg_putchar.h
dbg_putchar.o: dbg_putchar.h

clean:
	rm -rf *.o $(PRG).elf *.eps *.png *.pdf *.bak 
	rm -rf *.lst *.map $(EXTRA_CLEAN_FILES)

reset:
	$(PROGRAMMER) -c $(PROGRAMMER_DEV) -p $(PROGRAMMER_MCU) -n

term:
	$(PROGRAMMER) -c $(PROGRAMMER_DEV) -p $(PROGRAMMER_MCU) -nt

lst:  $(PRG).lst

%.lst: %.elf
	$(OBJDUMP) -h -S $< > $@

# Rules for building the .text rom images

text: hex bin srec

hex:  $(PRG).hex
bin:  $(PRG).bin
srec: $(PRG).srec

%.hex: %.elf
	$(OBJCOPY) -j .text -j .data -O ihex $< $@

%.srec: %.elf
	$(OBJCOPY) -j .text -j .data -O srec $< $@

%.bin: %.elf
	$(OBJCOPY) -j .text -j .data -O binary $< $@

# Rules for building the .eeprom rom images

eeprom: ehex ebin esrec

ehex:  $(PRG)_eeprom.hex
ebin:  $(PRG)_eeprom.bin
esrec: $(PRG)_eeprom.srec

%_eeprom.hex: %.elf
	$(OBJCOPY) -j .eeprom --change-section-lma .eeprom=0 -O ihex $< $@ \
	|| { echo empty $@ not generated; exit 0; }

%_eeprom.srec: %.elf
	$(OBJCOPY) -j .eeprom --change-section-lma .eeprom=0 -O srec $< $@ \
	|| { echo empty $@ not generated; exit 0; }

%_eeprom.bin: %.elf
	$(OBJCOPY) -j .eeprom --change-section-lma .eeprom=0 -O binary $< $@ \
	|| { echo empty $@ not generated; exit 0; }

# Every thing below here is used by avr-libc's build system and can be ignored
# by the casual user.

FIG2DEV                 = fig2dev
EXTRA_CLEAN_FILES       = *.hex *.bin *.srec

dox: eps png pdf

eps: $(PRG).eps
png: $(PRG).png
pdf: $(PRG).pdf

%.eps: %.fig
	$(FIG2DEV) -L eps $< $@

%.pdf: %.fig
	$(FIG2DEV) -L pdf $< $@

%.png: %.fig
	$(FIG2DEV) -L png $< $@

