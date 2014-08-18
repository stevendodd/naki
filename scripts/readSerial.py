#!/usr/bin/env python

import serial
import re

# Log all serial output debug=1 else debug=0 for just telemetry data
debug = 0

# save telemetry data in logfile if save=1
save = 1
telemtryOutput = "/home/pi/hab/logs/telemetry.log"
f = open(telemtryOutput, 'w')

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

    else:
      # $$NAKI,04614,12:59:58,51.3254,-0.7731,125,25,28,7.42,6*31B5
      log = re.search('\$\$NAKI,(\d+),(\d+):(\d+):(\d+),([-+]?\d*\.\d+|\d+),([-+]?\d*\.\d+|\d+),([-+]?\d*\.\d+|\d+),([-+]?\d*\.\d+|\d+),([-+]?\d*\.\d+|\d+),([-+]?\d*\.\d+|\d+),(\d+)\*(.*)', rcv.rstrip())
     
      if log:
        print "----------------------------------------------"
        print log.group()
        print "Message ID: " + log.group(1)
        print "Time: " + log.group(2) + ":" + log.group(3) + ":" + log.group(4)  
        print "Latitude: " + log.group(5)
        print "Longitude: " + log.group(6)
        print "Altitude: " + log.group(7)
        print "External Temp: " + log.group(8)
        print "Internal Temp: " + log.group(9)
        print "Voltage: " + log.group(10)
        print "Number of Satellites: " + log.group(11)
        print "Checksum: " + log.group(12)
        
        if save==1:
          f.write(log.group() + "\n")
          f.flush()
