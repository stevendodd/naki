#!/bin/sh

# Add background process for logger
mkdir /home/pi/hab/logs
mkdir /home/pi/hab/photos
sudo cp /home/pi/hab/scripts/tracker.sh /etc/init.d
sudo update-rc.d tracker.sh defaults
