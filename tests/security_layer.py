#####################
# Combine ultrasonic and rvr drive but avoid collisions and drops.
#
#  References:
#  [1] Asyncio - https://docs.python.org/3/library/asyncio-task.html
#####################

import numpy as np
import math
import time

import asyncio
import concurrent.futures

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

# Driving speeds
STOP = 0
DRIVE_SLOW = 35
DRIVE_NORMAL = 50
DRIVE_FAST = 65


#
# Init sphero rvr async sdk
#
loop = asyncio.get_event_loop()
rvr = SpheroRvrAsync(
    dal=SerialAsyncDal(
        loop
    )
)

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

# Set State of rvr
class RvrState: pass
state = RvrState() 
state.speed = 0
state.num_oks = 0


#
# MACHINE LEARNING
#
def model():
    x = 0
    print("#### START INFERENCE")
    for _ in range(10000):
        x = 2 * 3 + x
    return x


async def tf_inference():
    """ Inference ONCE in a seperate process to not block anything else.
    Note that this computation will be finsihed also if the main process is killed.
    """
    


async def cyclic_ml_check():
    while True:
        with concurrent.futures.ProcessPoolExecutor() as executor:
            x = await loop.run_in_executor(executor, model)
        print("#### STOP INFERENCE %d" % x)
        asyncio.sleep(2)

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

async def led_blue():
    await rvr.set_all_leds(
        led_group=RvrLedGroups.all_lights.value,
        led_brightness_values=[color for x in range(10) for color in [0, 0, 255]]
    )


async def play_tune(tone, duration, freq):
    """ ToDo This must be done async...
    """
    tone.ChangeDutyCycle(50)
    tone.ChangeFrequency(freq)
    await asyncio.sleep(float(duration) / 1000)
    tone.ChangeDutyCycle(0)
    await asyncio.sleep(0.05)


async def say_hello():
    await play_tune(tone, 100, 14000)
    await play_tune(tone, 100, 14000)
    await play_tune(tone, 100, 8000)
    await play_tune(tone, 100, 14000)


async def say_bye():
    await play_tune(tone, 100, 8000)
    await play_tune(tone, 100, 8000)
    await play_tune(tone, 100, 8000)
    await play_tune(tone, 100, 14000)
    await play_tune(tone, 100, 8000)


async def user_communication():
    global state
    old_speed = state.speed

    while True:
        await asyncio.sleep(0.5)
        
        if old_speed == state.speed:
            continue

        if state.speed == STOP:
            await led_red()
            await play_tune(tone, 100, 14000)
            await play_tune(tone, 100, 8000)
            await play_tune(tone, 400, 4000)

        elif state.speed == DRIVE_FAST:
            await led_blue()
            await play_tune(tone, 1000, 14000)

        elif state.speed == DRIVE_NORMAL:
            await led_green()

        elif state.speed == DRIVE_SLOW:
            await led_yellow()
        
        old_speed = state.speed


async def stop_robot():
    state.speed = STOP
    await rvr.raw_motors(0,0,0,0)


async def security_loop():
    global state

    num_oks = -50   # Init time
    ok_steps = 70
    while True:
        # Measure front
        try:
            await asyncio.sleep(0.005)
            front_d = measure_ultrasonic_distance(FRONT_TRIGGER, FRONT_ECHO)
        except Exception as e:
            print(e)
            await stop_robot()
            num_oks = 0
            continue
        
        # Measure bottom left and right and handle fabric material
        try:
            await asyncio.sleep(0.005)
            right_d = measure_ultrasonic_distance(RIGHT_TRIGGER, RIGHT_ECHO)
            await asyncio.sleep(0.005)
            left_d = measure_ultrasonic_distance(LEFT_TRIGGER, LEFT_ECHO)
        except UltrasonicDistanceTooLarge as e:
            print("Ultrasonic measures of one sensor is too large. Assume fabric material but drive slow...")
            left_d, right_d = (-1, -1)
        except Exception as e:
            print(e)
            await stop_robot()
            num_oks = 0
            continue

        # Adapt speed to front sensor
        if(front_d < 20):
            print("Avoiding front crash | F: %.2f cm" % front_d)
            await stop_robot() 
            num_oks = 0
            continue

        # Decrease speed if we detect a obstacle
        if(front_d < 80):
            state.speed -= 1 if state.speed > DRIVE_SLOW else 0

        # Adapt to cliffs
        if(right_d > 15 or left_d > 15):
            print("Avoiding drop | R: %.2f cm | L: %.2f cm" % (right_d, left_d))
            await stop_robot()
            num_oks = 0
            continue
        
        # We need multiple oks before we start again
        num_oks += 1
        if num_oks < ok_steps:
            continue

        # Everything is fine so determine speed
        if(num_oks == 3 * ok_steps):
            state.speed = DRIVE_FAST
        elif(num_oks == 2 * ok_steps):
            state.speed = DRIVE_NORMAL
        elif(num_oks == ok_steps):
            state.speed = DRIVE_SLOW


async def drive():
    """ This program has RVR drive around in different directions using the function drive_with_heading.

    speed: Drive distance with given speed
    distance: Distance to drive [m]
    """
    global state

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
    while(True):
        await asyncio.sleep(0.01)
        if state.speed <= 0:
            continue

        await rvr.drive_with_heading(
                speed=state.speed,
                heading=0,
                flags=DriveFlagsBitmask.none.value
            )
    ########################################


async def tear_down():
    await stop_robot(),
    await rvr.sensor_control.clear(),
    await rvr.close()


#
# M A I N
#
if __name__ == '__main__':
    try:
        

        loop.run_until_complete(
            asyncio.gather(
                #cyclic_ml_check(),
                say_hello(),
                security_loop(),
                drive(),
                user_communication()
            )
        )

    except Exception as e:
        print(e)
        print('\nProgram terminated.')
        loop.run_until_complete(
            asyncio.gather(
                tear_down(),
                say_bye()
            )
        )

    finally:
        if loop.is_running():
            loop.close()
            