Project Naki
============

Tracker for a hot-air balloon flight; or other high altitude activity 

Code is written for an ATMega328p microcontroller; and interfaces with http://ukhas.org.uk/ tracking system via radio.  

1. Read GPS and other sensors
2. Create telemetry string containing data
3. Transmit string using RTTY over a Radio transmitter
4. Repeat

The tracker board has the following features:

  ATMega microcontroller with external 16Mz clock
  Reset jumper to restart the ATMega program
  ublox max7 GPS receiver 
  Radiometrix NTX2 radio transmitter - allows license exempt transmitting in the UK
  SMA connector for connecting the radio antenna 
  Internal temperature sensor 
  External temperature sensor
  Voltage sensor to measure remaining power available
  Socket to connect to a Raspberry Pi 26 pin GPIO header
      Programming the ATMega directly from the Pi
      Reading serial output from the ATMega program and logging on the pi SD card
  Power jumpers to allow power from the Raspberry Pi or independent supply
  
 