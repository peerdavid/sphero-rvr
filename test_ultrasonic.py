
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
front_trigger_pin = 4
front_echo_pin = 27

right_trigger_pin = 18
right_echo_pin = 23

left_trigger_pin = 17
left_echo_pin = 22


GPIO.setmode(GPIO.BCM)
GPIO.setup(front_trigger_pin, GPIO.OUT)
GPIO.setup(front_echo_pin, GPIO.IN)

GPIO.setup(right_trigger_pin, GPIO.OUT)
GPIO.setup(right_echo_pin, GPIO.IN)

GPIO.setup(left_trigger_pin, GPIO.OUT)
GPIO.setup(left_echo_pin, GPIO.IN)



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
        start_time = time.time()
        front_d = measure_ultrasonic_distance(front_trigger_pin, front_echo_pin)
        right_d = measure_ultrasonic_distance(right_trigger_pin, right_echo_pin)
        left_d = measure_ultrasonic_distance(left_trigger_pin, left_echo_pin)
        stop_time = time.time()

        time_elapsed = stop_time - start_time
        print("Front: %.2f | Right: %.2f | Left: %.2f | Time elapsed: %.3f" % (front_d, right_d, left_d, time_elapsed), 
            end="\r", 
            flush=True)

        time.sleep(0.1)

if __name__ == '__main__':
    main()
