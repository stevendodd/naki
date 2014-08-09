import serial
import time

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
    print ("--> %s " % (rcv.rstrip()))
