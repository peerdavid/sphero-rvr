# Sphero RVR
This projects extends your Sphero RVR with different hardware modules:
[x] Sound module (like R2D2)
[x] Ultrasonic modules
[x] Camera

And includes different software addons:
[x] Security layer that is ALWAYS active to avoid crashes and cliffs
[] Object detection using asyncio with TFLite 2.1
[] Follow me
[] (Random?) Autonomous driving
[] Create maps of rooms
[] Drive to a room after a map is created and take pictures
[] Security robot -> check if there is any anomaly in any of the rooms (unknown faces etc.)

Note: This project is WIP and thereofe we can not avoid bugs etc. Also the 
security layer can avoid crashes only with some probability, because its not 
a hard realtime system.

# Setup
## Software
Clone this repository and execute ./install.sh
Note that we assume here that the cloned folder name is sphero-rvr.
Note also that in the directory above the sphero python sdk is installed.

To add new packages run pipenv install PACKAGE

## Hardware
For the hardware we need to connect the raspberry pi 4b to the sphero rvr, install the raspberry camera, install ultrasonic sensors (security layer) and speakers:

Ultrasonic: The raspberry has 3.3V, the sensor 5V therefore we want 3,3V for the echo:
R_2 = 3.9k and R_1 = 2.2k
and therefore 
as V_out / V_in = R_2 / (R_1 + R_2)

such that V_in ~ 3.2V

### Sphero RVR connection

### Ultrasonic sensors

### Speakers

### Camera


# Description
## Security Layer 
Every example can access functionality of the robot only through the security layer. This layer ensures that no crash etc. can happen. Therefore autonomous driving etc. can focus on this funcitonality and not on security relevant stuff.


# Tipps for devs
1. To develop on the Sphero RVR I use VsCode (on a dev machine) directly debug the raspberry on the robot over ssh.


# References
[1] Ultrasonic RVR example - https://sdk.sphero.com/docs/samples_content/raspberry_pi/python/ultrasonic_rvr_sample/ <br />
[2] Install Sphero RVR SDK - https://sdk.sphero.com/docs/getting_started/raspberry_pi/raspberry_pi_setup/ <br />
[3] Raspberry GPIO pins - https://www.raspberrypi.org/documentation/usage/gpio/ <br/>
[4] Tool to create laser-cut box - https://www.makercase.com/#/kerfbox <br />