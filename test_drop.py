

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
# Sensor handlers
#
async def locator_handler(locator_data):
    """ Handle location data (x,y) position of robot.
        See also https://community.sphero.com/t/programming-questions/829/4
    """
    location["x"] = locator_data['Locator']['X']
    location["y"] = locator_data['Locator']['Y']
    print("Position: x: %.2f [m] | y: %.2f [m]" % (location["x"], location["y"]))


async def quaternion_handler(quaternion_data):
    w = quaternion_data['Quaternion']['W']
    x = quaternion_data['Quaternion']['X']
    y = quaternion_data['Quaternion']['Y']
    z = quaternion_data['Quaternion']['Z']
    print("Quaternion: w %.2f | x %.2f | y %.2f | z %.2f" % (w, x, y, z))
    


async def gyroscope_handler(gyroscope_data):
    x = gyroscope_data['Gyroscope']['X'] * 2 * math.pi / 360
    y = gyroscope_data['Gyroscope']['Y'] * 2 * math.pi / 360
    z = gyroscope_data['Gyroscope']['Z'] * 2 * math.pi / 360
    print("Gyroscope: x %.2f | y %.2f | z %.2f" % (x, y, z))
    


async def velocity_handler(velocity_data):
    x = velocity_data['Velocity']['X'] * 5.0 / MAX_SENSOR_VALUE
    y = velocity_data['Velocity']['Y'] * 5.0 / MAX_SENSOR_VALUE
    print("Velocity: x %.2f | y %.2f" % (x, y))


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

    # Init RVR
    await rvr.wake()
    await asyncio.sleep(1)

    print("------------------------------")
    print("RVR data")
    print("------------------------------")
    print("Battery [%%]: %d" % (await rvr.get_battery_percentage())["percentage"])
    await asyncio.sleep(1)
    print("MAC-Address: %s" % (await rvr.get_mac_address())["macAddress"])
    print("------------------------------")
    await asyncio.sleep(1)

    # All leds off
    await rvr.set_all_leds(
        led_group=RvrLedGroups.all_lights.value,
        led_brightness_values=[color for _ in range(10) for color in Colors.off.value]
    )
    await asyncio.sleep(1)

    # Initialize sensors 
    await rvr.sensor_control.add_sensor_data_handler(
        service=RvrStreamingServices.locator,
        handler=locator_handler,
    )
    # await rvr.sensor_control.add_sensor_data_handler(
    #     service=RvrStreamingServices.quaternion,
    #     handler=quaternion_handler,
    # )
    # await rvr.sensor_control.add_sensor_data_handler(
    #     service=RvrStreamingServices.gyroscope,
    #     handler=gyroscope_handler,
    # )
    # await rvr.sensor_control.add_sensor_data_handler(
    #     service=RvrStreamingServices.velocity,
    #     handler=velocity_handler,
    # )
    await rvr.sensor_control.start(interval=100)
    await asyncio.sleep(1)

    # Reset coordinates
    await rvr.reset_locator_x_and_y()
    await asyncio.sleep(1)

    # Start driving
    await rvr.reset_yaw()
    await asyncio.sleep(1)

    await rvr.set_all_leds(
        led_group=RvrLedGroups.all_lights.value,
        led_brightness_values=[color for x in range(10) for color in [0, 255, 0]]
    )
    await asyncio.sleep(0.5)
    await rvr.set_all_leds(
        led_group=RvrLedGroups.all_lights.value,
        led_brightness_values=[color for _ in range(10) for color in Colors.off.value]
    )
    await asyncio.sleep(0.5)
    await rvr.set_all_leds(
        led_group=RvrLedGroups.all_lights.value,
        led_brightness_values=[color for x in range(10) for color in [0, 255, 0]]
    )

    ########################################
    # Driving loop
    ########################################
    while(True):

        front_d = measure_ultrasonic_distance(FRONT_TRIGGER, FRONT_ECHO)
        right_d = measure_ultrasonic_distance(RIGHT_TRIGGER, RIGHT_ECHO)
        left_d = measure_ultrasonic_distance(LEFT_TRIGGER, LEFT_ECHO)

        if(front_d < 25):
            print("Avoiding front crash with %.2f cm" % front_d)
            break

        if(right_d > 12):
            print("Avoiding right drop with %.2f cm" % right_d)
            break

        if(left_d > 12):
            print("Avoiding left drop with %.2f cm" % left_d)
            break

        if(front_d < 50):
            await rvr.set_all_leds(
                led_group=RvrLedGroups.all_lights.value,
                led_brightness_values=[color for x in range(10) for color in [255, 255, 0]]
            )
        
        # Everything is fine, so letr drive straight ahead
        await rvr.drive_with_heading(
            speed=speed,
            heading=0,
            flags=DriveFlagsBitmask.none.value
        )

        await asyncio.sleep(0.10)
    ########################################
    
    # Stop
    await rvr.raw_motors(0,0,0,0)
    await asyncio.sleep(1)

    await rvr.set_all_leds(
        led_group=RvrLedGroups.all_lights.value,
        led_brightness_values=[color for x in range(10) for color in [255, 0, 0]]
    )

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