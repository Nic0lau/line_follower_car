MCU      = atmega328p
F_CPU    = 16000000UL
BAUD     = 115200

PORT     = /dev/ttyUSB0

CC       = avr-gcc
OBJCOPY  = avr-objcopy
CFLAGS   = -mmcu=$(MCU) -DF_CPU=$(F_CPU) -Os -Wall -Wextra -Wfloat-equal -Wvla -Wpedantic -std=c99
TARGET   = line_follower

all: $(TARGET).hex

$(TARGET).elf: *.c
	$(CC) $(CFLAGS) -o $@ $^

$(TARGET).hex: $(TARGET).elf
	$(OBJCOPY) -O ihex -R .eeprom $< $@
	avr-size $<

flash: $(TARGET).hex
	avrdude -c arduino -p $(MCU) -P $(PORT) -b $(BAUD) -U flash:w:$<

fuses:
	avrdude -c arduino -p $(MCU) -P $(PORT) -b $(BAUD) -U lfuse:r:-:h -U hfuse:r:-:h -U efuse:r:-:h

clean:
	rm -f $(TARGET).elf $(TARGET).hex

.PHONY: all flash fuses clean
