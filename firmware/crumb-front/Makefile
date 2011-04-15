TARGET = crumb_firmware

OBJS = ADS1100.o Driver.o interrupt.o lm75.o servo.o uart.o i2cmaster.o depth_sensor.o

CC = avr-gcc

COMMON = -mmcu=atmega128

HEX_FLASH_FLAGS = -R .eeprom

HEX_EEPROM_FLAGS = -j .eeprom
HEX_EEPROM_FLAGS += --set-section-flags=.eeprom="alloc,load"
HEX_EEPROM_FLAGS += --change-section-lma .eeprom=0 --no-change-warnings


CFLAGS = $(COMMON) -Wall -O0 -std=gnu99

ASMFLAGS = $(COMMON) $(CFLAGS) -x assembler-with-cpp


.PHONY: all clean prepare

all: prepare $(TARGET).hex $(TARGET).eep

$(TARGET).elf: $(OBJS) 
	$(CC) $(LDFLAGS) $(COMMON) $(OBJS) -o $(TARGET).elf

%.hex: $(TARGET).elf
	avr-objcopy -O ihex $(HEX_FLASH_FLAGS) $< $@

%.lss: $(TARGET).elf
	avr-objdump -h -S $< >$@

%.eep: $(TARGET).elf
	-avr-objcopy $(HEX_EEPROM_FLAGS) -O ihex $< $@ || exit 0

%.o: %.c
	$(CC) $(CFLAGS) -DCOMPILE_FOR_CRUMB -c $<

%.o: %.S
	$(CC) $(ASMFLAGS) -c $<

clean:
	rm -rf $(OBJS) $(TARGET) $(TARGET).hex $(TARGET).eep size $(TARGET).elf ldscripts

# HACKHACK: Bug in crossdev, findet sonst ldscripts/avr5.x nicht
prepare:
#	if [ ! -x ldscripts ]; then ln -sf `find /usr/lib/binutils/avr -name ldscripts -type d` ldscripts; fi

reset2:
	avrdude -cavrisp2 -patmega128 -Pusb -y
reset:
	avrdude -cstk500v2 -patmega128 -P/dev/lowlevel_programmer -y

flash_via_boot: $(TARGET).hex
	avrdude -cstk500 -patmega128 -P/dev/ttyUSB0 -Uflash:w:$(TARGET).hex -F
	
#flash: 
#	avrdude -cavrisp2 -patmega128 -Pusb -Uflash:w:$(TARGET).hex -F

flash: 
	avrdude -cstk500v2 -patmega128 -P/dev/ttyUSB0 -Uflash:w:$(TARGET).hex -F

avalon-flash:
	avrdude -cavrisp2 -patmega128 -Pusb -Uflash:w:$(TARGET).hex -F

