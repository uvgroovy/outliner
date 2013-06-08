import RPi.GPIO as GPIO
from time import time,sleep
import struct

PIN = 7


def main():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(PIN, GPIO.IN)
    try:
        clicks = 0
        while True:
            GPIO.wait_for_edge(PIN, GPIO.FALLING)
            clicks += 1
            # sleep so i woun't get duplications cause my wiring is lame
            sleep(.1)
            print "Clicks: ",clicks
    finally:
        GPIO.cleanup()


if __name__ == "__main__":
    main()
