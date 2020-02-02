
import numpy as np
import math

import asyncio
from sphero_sdk import SpheroRvrAsync
from sphero_sdk import SerialAsyncDal

#
# Init sphero rvr async sdk
#
loop = asyncio.get_event_loop()
rvr = SpheroRvrAsync(
    dal=SerialAsyncDal(
        loop
    )
)


#
# M A I N
#
async def main(speed=255, distance=1.0):
    """ This program has RVR drive around in different directions using the function drive_with_heading.

    speed: Drive distance with given speed
    distance: Distance to drive [m]
    """
    await rvr.wake()
    await asyncio.sleep(1)
    battery = (await rvr.get_battery_percentage())["percentage"]
    await asyncio.sleep(1)


    print("------------------------------")
    print("RVR Informations")
    print("------------------------------")
    print("Battery [%%]: %d" % battery)
    print("------------------------------")
    
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
                rvr.close()
            )
        )

    finally:
        if loop.is_running():
            loop.close()