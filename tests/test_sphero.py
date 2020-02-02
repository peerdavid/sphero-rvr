
import numpy as np
import math

import asyncio
from sphero_sdk import SpheroRvrAsync
from sphero_sdk import SerialAsyncDal
from sphero_sdk import DriveFlagsBitmask
from sphero_sdk import RvrStreamingServices
from sphero_sdk import Colors
from sphero_sdk import RvrLedGroups


#
# Constants
#
MAX_SENSOR_VALUE = 2**31


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
    

#
# M A I N
#
async def main(speed=255, distance=1.0):
    """ This program has RVR drive around in different directions using the function drive_with_heading.

    speed: Drive distance with given speed
    distance: Distance to drive [m]
    """
    await rvr.wake()
    await asyncio.sleep(2)
    battery = (await rvr.get_battery_percentage())["percentage"]
    print("------------------------------")
    print("RVR data")
    print("------------------------------")
    print("Battery [%%]: %d" % battery)
    print("------------------------------")
    await asyncio.sleep(2)

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

    ########################################
    # Driving loop
    ########################################
    while(location["y"] < distance):
        await rvr.drive_with_heading(
            speed=speed,
            heading=0,
            flags=DriveFlagsBitmask.none.value
        )

        await asyncio.sleep(0.25)
    ########################################
    
    # Stop
    await rvr.raw_motors(0,0,0,0)
    await asyncio.sleep(1)

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