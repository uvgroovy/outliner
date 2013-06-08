import RPi.GPIO as GPIO
from time import time
import struct

PIN = 7
    
CLICK_DISTANCE=6.25
ADDRESS=0b0011110

class Compass:
    def __init__(self, address):
        self.sock = None
        
    def getBearings(self):
        s.send(struct.pack("!I", 1))
        return struct.unpack("!I", s.recv(4))[0]

client = None

def getBearings():
    return client.getBearings()

def handleCick():
    bearing = getBearings()
    print time(), CLICK_DISTANCE, bearing
    
def work():
    GPIO.setmode(GPIO.BCM)
    
    GPIO.setup(PIN, GPIO.IN) 
    GPIO.add_event_detect(PIN, GPIO.RISING, callback=handleCick)

    
def main():
    try:
        global client
        address= ""
        client = Compass(address)
        work()
    finally:
        GPIO.cleanup()
        
        
        
if __name__ == "__main__":
    main()
