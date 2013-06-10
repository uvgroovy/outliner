import RPi.GPIO as GPIO
from time import time
import struct
from LSM303 import *

PIN = 7
    
CLICK_DISTANCE=6.25
ADDRESS=0b0011110


compass = LSM303()

def getHeadings():
    return client.getHeadings()

def handleCick():
    bearing = getHeadings()
    print time(), CLICK_DISTANCE, bearing

def init():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(PIN, GPIO.IN)

def getAngleFromData(data):
    mag,accel = data
    return compass.getHeadingFromEvent(mag)

def getData():
    GPIO.wait_for_edge(PIN, GPIO.FALLING)
    mag,accel = compass.getMagEvent(), compass.getAcclEvent()
    return mag,accel
        
def work():
    while True:
        print getAngle()
    
    
def main():
    try:
        init() 
        work()
    finally:
        GPIO.cleanup()
        
        
        
if __name__ == "__main__":
    main()
