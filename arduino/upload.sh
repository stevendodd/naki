#!/bin/sh

# Burn Bootloader
#/usr/share/arduino/hardware/tools/avrdude -C/usr/share/arduino/hardware/tools/avrdude.conf -q -patmega328 -cgpio -e -Ulock:w:0x3F:m -Uefuse:w:0x07:m -Uhfuse:w:0xD9:m -Ulfuse:w:0xFF:m
/usr/share/arduino/hardware/tools/avrdude -C/usr/share/arduino/hardware/tools/avrdude.conf -q -patmega328 -cgpio -Uflash:w:/usr/share/arduino/hardware/arduino/bootloaders/atmega/ATmegaBOOT_168_atmega328.hex:i -Ulock:w:0x3F:m

# Upload
/usr/share/arduino/hardware/tools/avrdude -C/usr/share/arduino/hardware/tools/avrdude.conf -q -patmega328 -cgpio -PCOM1 -b57600 -D -Uflash:w:$1:i
