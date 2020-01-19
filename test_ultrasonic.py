
#
# Test ultrasonic sensors. If some object is close to the rover, it starts rotating.
# Credit: https://electrosome.com/hc-sr04-ultrasonic-sensor-raspberry-pi/
#

import time
import numpy as np
import RPi.GPIO as GPIO
import asyncio


#
# GPIO Setup
#
trigger_pin = 23
echo_pin = 24

GPIO.setmode(GPIO.BCM)
GPIO.setup(trigger_pin, GPIO.OUT)
GPIO.setup(echo_pin, GPIO.IN)


def measure_ultrasonic_distance(trigger, echo):
    """ Measures the distance in cm for given trigger and echo GPIO pins.

        Distance = Speed * Time / 2 (we measure from rvr to obstacle back to rvr)
        Speed = 34300 cm/s. Therefore distance = time * 17150
    """
    GPIO.output(trigger, True)
    time.sleep(0.00001)
    GPIO.output(trigger, False)

    start_time = time.time()
    stop_time = time.time()

    while GPIO.input(echo) == 0:
        start_time = time.time()

    while GPIO.input(echo) == 1:
        stop_time = time.time()

    time_elapsed = stop_time - start_time

    distance = time_elapsed * 17150
    return distance


def main():
    while True:
        distance = measure_ultrasonic_distance(trigger_pin, echo_pin)
        print(distance)
        time.sleep(1)

if __name__ == '__main__':
    main()
