

#
# Combine ultrasonic and rvr drive but avoid collisions and drops.
#

import numpy as np
import math
import time

import asyncio
from sphero_sdk import SpheroRvrAsync
from sphero_sdk import SerialAsyncDal
from sphero_sdk import DriveFlagsBitmask
from sphero_sdk import RvrStreamingServices
from sphero_sdk import Colors
from sphero_sdk import RvrLedGroups

import RPi.GPIO as GPIO
GPIO.setwarnings(False)


#
# Constants
#
MAX_SENSOR_VALUE = 2**31

FRONT_TRIGGER = 4
FRONT_ECHO = 27

RIGHT_TRIGGER = 18
RIGHT_ECHO = 23

LEFT_TRIGGER = 17
LEFT_ECHO = 22

SOUND = 12


#
# Init sphero rvr async sdk
#
loop = asyncio.get_event_loop()
rvr = SpheroRvrAsync(
    dal=SerialAsyncDal(
        loop
    )
)

location = {"x": 0.0, "y": 0.0}


#
# FUNCTIONS
#
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


async def led_red():
    await rvr.set_all_leds(
        led_group=RvrLedGroups.all_lights.value,
        led_brightness_values=[color for x in range(10) for color in [255, 0, 0]]
    )

async def led_green():
    await rvr.set_all_leds(
        led_group=RvrLedGroups.all_lights.value,
        led_brightness_values=[color for x in range(10) for color in [0, 255, 0]]
    )

async def led_yellow():
    await rvr.set_all_leds(
        led_group=RvrLedGroups.all_lights.value,
        led_brightness_values=[color for x in range(10) for color in [255, 255, 0]]
    )

async def led_orange():
    await rvr.set_all_leds(
        led_group=RvrLedGroups.all_lights.value,
        led_brightness_values=[color for x in range(10) for color in [255, 127, 0]]
    )


def play_tune(tone, duration, freq):
    """ ToDo This must be done async...
    """
    tone.ChangeDutyCycle(50)
    tone.ChangeFrequency(freq)
    time.sleep(float(duration) / 1000)
    tone.ChangeDutyCycle(0)
    time.sleep(0.05)


#
# M A I N
#
async def main(speed=50):
    """ This program has RVR drive around in different directions using the function drive_with_heading.

    speed: Drive distance with given speed
    distance: Distance to drive [m]
    """

    # Init ultrasonic sensors
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(FRONT_TRIGGER, GPIO.OUT)
    GPIO.setup(FRONT_ECHO, GPIO.IN)

    GPIO.setup(RIGHT_TRIGGER, GPIO.OUT)
    GPIO.setup(RIGHT_ECHO, GPIO.IN)

    GPIO.setup(LEFT_TRIGGER, GPIO.OUT)
    GPIO.setup(LEFT_ECHO, GPIO.IN)

    # Init sound
    GPIO.setup(SOUND, GPIO.OUT)
    GPIO.output(SOUND, 0)
    tone = GPIO.PWM(SOUND, 100)
    tone.start(0)

    # Play hellp
    play_tune(tone, 100, 14000)
    play_tune(tone, 100, 14000)
    play_tune(tone, 100, 8000)
    play_tune(tone, 100, 14000)

    # Init RVR
    await rvr.wake()
    await asyncio.sleep(1)

    print("------------------------------")
    print("Battery [%%]: %d" % (await rvr.get_battery_percentage())["percentage"])
    await asyncio.sleep(1)
    print("------------------------------")

    # Start driving
    await rvr.reset_yaw()
    await asyncio.sleep(1)
    ########################################
    # Driving loop
    ########################################
    num_oks = 0
    heading = 0
    while(True):

        front_d = measure_ultrasonic_distance(FRONT_TRIGGER, FRONT_ECHO)
        right_d = measure_ultrasonic_distance(RIGHT_TRIGGER, RIGHT_ECHO)
        left_d = measure_ultrasonic_distance(LEFT_TRIGGER, LEFT_ECHO)

        if(front_d < 25):
            print("Avoiding front crash with %.2f cm" % front_d)
            await rvr.raw_motors(0,0,0,0)
            await led_red()
            await asyncio.sleep(1)
            found_problem = True
            num_oks = 0
            continue

        if(right_d > 12):
            print("Avoiding right drop with %.2f cm" % right_d)
            await rvr.raw_motors(0,0,0,0)
            await led_red()
            await asyncio.sleep(1)
            found_problem = True
            num_oks = 0
            continue

        if(left_d > 12):
            print("Avoiding left drop with %.2f cm" % left_d)
            await rvr.raw_motors(0,0,0,0)
            await led_red()
            await asyncio.sleep(1)
            found_problem = True
            num_oks = 0
            continue
        
        # Only if multiple measures where ok we start again
        num_oks += 1
        if num_oks < 3:
            await led_orange()
            await asyncio.sleep(1)
            continue  
        elif num_oks == 3:
            play_tune(tone, 100, 8000)
            play_tune(tone, 100, 8000)
            play_tune(tone, 100, 14000)
            heading = 0
            await rvr.reset_yaw()
            await asyncio.sleep(1)

        if num_oks % 10 == 0 and num_oks > 0:
            heading = 0
            await rvr.reset_yaw()

        # Warning close
        if(front_d < 50):
            await led_orange()
            heading = 90
        else:
            await led_green()
        
        # Everything is fine, so letr drive straight ahead
        await rvr.drive_with_heading(
            speed=speed,
            heading=heading,
            flags=DriveFlagsBitmask.none.value
        )

        await asyncio.sleep(0.1)
    ########################################
    
    # Stop
    await rvr.close()
    await asyncio.sleep(1)


if __name__ == '__main__':
    try:
        loop.run_until_complete(
            main()
        )

    except KeyboardInterrupt:
        print('\nProgram terminated with keyboard interrupt.')
        loop.run_until_complete(
            asyncio.gather(
                rvr.sensor_control.clear(),
                rvr.close()
            )
        )

    finally:
        if loop.is_running():
            loop.close()