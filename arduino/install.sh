#!/bin/sh

sudo mv /usr/share/arduino/hardware/arduino/boards.txt /usr/share/arduino/hardware/arduino/board.txt.bak
sudo cp boards.txt /usr/share/arduino/hardware/arduino/
sudo mv /usr/share/arduino/hardware/arduino/programmers.txt /usr/share/arduino/hardware/arduino/programmers.txt.bak
sudo cp programmers.txt /usr/share/arduino/hardware/arduino/

mkdir -p ~/sketchbook
cp -R libraries ~/sketchbook/
cp upload.sh ~/sketchbook/

cp arduino.mk ~/sketchbook/ 
cp Makefile ~/sketchbook/

sudo cp avrdude.conf /etc/avrdude.conf

# Add background process for logger
sudo cp /home/pi/hab/scripts/logTelemetry.sh /etc/init.d
sudo update-rc.d logTelemetry.sh defaults
