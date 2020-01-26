

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



class UltrasonicTimeout(Exception):
    def __init__(self, arg):
        self.strerror = arg
        self.args = {arg}

class UltrasonicDistanceTooLarge(Exception):
    def __init__(self, arg):
        self.strerror = arg
        self.args = {arg}

class UltrasonicDistanceTooSmall(Exception):
    def __init__(self, arg):
        self.strerror = arg
        self.args = {arg}



#
# Constants
#

# Ultrasonic sensors
# See also https://www.mouser.com/datasheet/2/813/HCSR04-1022824.pdf
HCSR04_MIN_RANGE = 2
HCSR04_MAX_RANGE = 4 * 100

# Sphero sensor values
MAX_SENSOR_VALUE = 2**31

# Gpio pins
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
def measure_ultrasonic_distance(trigger, echo, retry=10e5):
    """ Measures the distance in cm for given trigger and echo GPIO pins.

        Distance = Speed * Time / 2 (we measure from rvr to obstacle back to rvr)
        Speed = 34300 cm/s. Therefore distance = time * 17150
    """
    GPIO.output(trigger, True)
    time.sleep(0.00001)
    GPIO.output(trigger, False)

    start_time = time.time()
    stop_time = time.time()
    
    fail_count = 0
    while GPIO.input(echo) == 0:
        start_time = time.time()
        fail_count += 1

        if fail_count >= retry:
            raise UltrasonicTimeout("Failed to measure distance.")
    
    fail_count = 0
    while GPIO.input(echo) == 1:
        stop_time = time.time()
        fail_count += 1

        if fail_count >= retry:
            raise UltrasonicTimeout("Failed to measure distance.")

    time_elapsed = stop_time - start_time
    distance = time_elapsed * 17150
    
    if distance > HCSR04_MAX_RANGE:
        raise UltrasonicDistanceTooLarge("Measured distance is too large indicating a problem with the material (carpet, fabric etc.).")

    if distance < HCSR04_MIN_RANGE:
        raise UltrasonicDistanceTooSmall("Measured distance is too large indicating a problem with the material (carpet, fabric etc.).")

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


async def stop_robot():
    await rvr.raw_motors(0,0,0,0)
    await led_red()
    await asyncio.sleep(1)
    return True, 0

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
    while(True):

        found_problem = False

        # Measure ultrasonic sensors
        try:
            front_d = measure_ultrasonic_distance(FRONT_TRIGGER, FRONT_ECHO)
        except Exception as e:
            print(e)
            found_problem, num_oks = await stop_robot()            
            continue
        
        right_ds = []
        left_ds = []

        for _ in range(3):
            try:
                await asyncio.sleep(0.005)
                right_ds = measure_ultrasonic_distance(RIGHT_TRIGGER, RIGHT_ECHO)
                await asyncio.sleep(0.005)
                left_ds = measure_ultrasonic_distance(LEFT_TRIGGER, LEFT_ECHO)
            except UltrasonicDistanceTooLarge as e:
                print("Ultrasonic measures too large, assuming fabric material")
                right_ds = [-1]
                left_ds = [-1]
                break
            except Exception as e:
                print(e)
                found_problem, num_oks = await stop_robot()            
                break
        
        if found_problem:
            continue

        # Remove one outlier
        right_d = np.median(right_ds)
        left_d = np.median(left_ds)

        # Avoid obstacles and cliffs
        if(front_d < 30):
            print("Avoiding front crash with %.2f cm" % front_d)
            found_problem, num_oks = await stop_robot()    
            continue

        if(right_d > 15):
            print("Avoiding right drop with %.2f cm" % right_d)
            found_problem, num_oks = await stop_robot()    
            continue

        if(left_d > 15):
            print("Avoiding left drop with %.2f cm" % left_d)
            found_problem, num_oks = await stop_robot()    
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
            await led_green()
            await asyncio.sleep(1)
            await rvr.reset_yaw()
            await asyncio.sleep(1)

        # Everything is fine, so letr drive straight ahead
        await rvr.drive_with_heading(
            speed=speed,
            heading=0,
            flags=DriveFlagsBitmask.none.value
        )
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