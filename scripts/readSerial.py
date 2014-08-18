#!/usr/bin/env python

import serial
import re
from subprocess import call

# Log all serial output debug=1 else debug=0 for just telemetry data
debug = 0

# save telemetry data in logfile if save=1
save = 1
telemtryOutput = "/home/pi/hab/logs/telemetry.log"
f = open(telemtryOutput, 'w')

# take photos if photos=1
photos=0
photoDirectory = "/home/pi/hab/photos/"

# Read a line from the serial port
def readlineCR(port):
    rv = ""
    while True:
        ch = port.read()
        rv += ch
        #if ch=='\r' or ch=='' or ch=='\n':
        if ch=='\n':
            return rv

port = serial.Serial("/dev/ttyAMA0", baudrate=9600, timeout=3.0)

while True:
    rcv = readlineCR(port)

    if debug==1:
      print (rcv.rstrip())
      break

    # $$NAKI,04614,12:59:58,51.3254,-0.7731,125,25,28,7.42,6*31B5
    log = re.search('\$\$NAKI,(\d+),(\d+):(\d+):(\d+),([-+]?\d*\.\d+|\d+),([-+]?\d*\.\d+|\d+),([-+]?\d*\.\d+|\d+),([-+]?\d*\.\d+|\d+),([-+]?\d*\.\d+|\d+),([-+]?\d*\.\d+|\d+),(\d+)\*(.*)', rcv.rstrip())

    if log:
      messageId = log.group(1)
      hours = log.group(2)
      minutes = log.group(3)
      seconds = log.group(4)
      latitude = log.group(5)
      longitude = log.group(6)
      altitude = log.group(7)
      extTemp = log.group(8)
      intTemp = log.group(9)
      voltage = log.group(10)
      satellites = log.group(11)
      checksum = log.group(12)
     
      print "----------------------------------------------"
      print log.group()
      print "Message ID: " + messageId
      print "Time: " + hours + ":" + minutes + ":" + seconds
      print "Latitude: " + latitude
      print "Longitude: " + longitude
      print "Altitude: " + altitude
      print "External Temp: " + extTemp
      print "Internal Temp: " + intTemp
      print "Voltage: " + voltage
      print "Number of Satellites: " + satellites
      print "Checksum: " + checksum
        
      if save==1:
        f.write(log.group() + "\n")
        f.flush()

      if photos==1:
        if int(messageId)%5 == 0:
          print "<<<<<<<<<<<<<<<<<< raspistill -ex auto -mm matrix -o " + photoDirectory + messageId + ".jpg  >>>>>>>>>>>>>>>>>>>\n"
          call(["raspistill", "-ex", "auto", "-mm", "matrix", "-o",  photoDirectory + messageId + ".jpg"])

