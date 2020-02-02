#####################
# Combine ultrasonic and rvr drive but avoid collisions and drops
# and say hello to persons if detected...
#  References:
#  [1] Asyncio - https://docs.python.org/3/library/asyncio-task.html
#####################

import numpy as np
import math
import time
import io

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

from PIL import Image
from picamera import PiCamera
from tflite_runtime.interpreter import Interpreter


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
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480


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
state.watch_dog = time.monotonic()
state.num_oks = 0
state.persons = 0
state.speed=0


#
# MACHINE LEARNING
#
def set_input_tensor(interpreter, image):
  """Sets the input tensor."""
  tensor_index = interpreter.get_input_details()[0]['index']
  input_tensor = interpreter.tensor(tensor_index)()[0]
  input_tensor[:, :] = image


def get_output_tensor(interpreter, index):
  """Returns the output tensor at the given index."""
  output_details = interpreter.get_output_details()[index]
  tensor = np.squeeze(interpreter.get_tensor(output_details['index']))
  return tensor


def detect_objects(interpreter, image, threshold):
  """Returns a list of detection results, each a dictionary of object info."""
  set_input_tensor(interpreter, image)
  interpreter.invoke()

  # Get all output details
  boxes = get_output_tensor(interpreter, 0)
  classes = get_output_tensor(interpreter, 1)
  scores = get_output_tensor(interpreter, 2)
  count = int(get_output_tensor(interpreter, 3))

  results = []
  for i in range(count):
    if scores[i] >= threshold:
      result = {
          'bounding_box': boxes[i],
          'class_id': classes[i],
          'score': scores[i]
      }
      results.append(result)
  return results


def inference():
    global state
    
    interpreter = Interpreter("models/mobilenet_v1/detect.tflite")
    interpreter.allocate_tensors()
    _, input_height, input_width, _ = interpreter.get_input_details()[0]['shape']
    stream = io.BytesIO()

    camera = PiCamera(resolution=(CAMERA_WIDTH, CAMERA_HEIGHT))
    camera.start_preview()
    camera.rotation=90

    try:
        while(True):

            # last_ack_ms = (time.monotonic() - state.watch_dog) * 1000
            # print("last_ack_ms", last_ack_ms)
            # if last_ack_ms > 5000:
            #     print("(Error) Main process died after %.3f." % last_ack_ms)
            #     raise Exception("Main process died.")

            print("Camera")
            # Capture camera
            camera.capture(stream, format='jpeg')
            stream.seek(0)  # "Rewind" the stream to the beginning so we can read its content

            # Get image from camera
            image = Image.open(stream).convert('RGB').resize(
                    (input_width, input_height), Image.ANTIALIAS) #.rotate(270)
            
            #image.save("capture.png")

            # Inference given tflite model
            print("Inference")
            results = detect_objects(interpreter, image, 0.6)
            state.persons = len([p for p in results if p["class_id"] == 0])
            
            # Clear camera stream
            stream.seek(0)
            stream.truncate()

            time.sleep(2)
    finally:
        camera.stop_preview()


async def computer_vision():
    with concurrent.futures.ThreadPoolExecutor() as executor:
        await loop.run_in_executor(executor, inference)

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
    await play_tune(tone, 200, 8000)
    await play_tune(tone, 200, 4000)


async def user_communication():
    global state
    old_state = RvrState()
    old_state.persons = 0
    old_state.speed = 0

    while True:
        await asyncio.sleep(0.5)
        
        if old_state.speed != state.speed:
            old_state.speed = state.speed

            if state.speed == STOP:
                await led_red()
                #await play_tune(tone, 100, 14000)
                #await play_tune(tone, 100, 8000)
                #await play_tune(tone, 400, 4000)

            elif state.speed == DRIVE_FAST:
                await led_blue()
                #await play_tune(tone, 1000, 14000)

            elif state.speed == DRIVE_NORMAL:
                await led_green()

            elif state.speed == DRIVE_SLOW:
                await led_yellow()
        
        if old_state.persons != state.persons:
            old_state.persons = state.persons

            if state.persons <= 0:
                await say_bye()
            else:
                await say_hello()

async def stop_robot():
    global state

    state.speed = STOP
    await rvr.raw_motors(0,0,0,0)


async def security_loop():
    global state

    num_oks = -50   # Init time
    ok_steps = 70
    while True:
        # Update watchdog
        state.watch_dog = time.monotonic()

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
                security_loop(),
                drive(),
                computer_vision(),
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