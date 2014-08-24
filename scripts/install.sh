#!/bin/sh

# Add background process for logger
sudo cp /home/pi/hab/scripts/tracker.sh /etc/init.d
sudo update-rc.d tracker.sh defaults
