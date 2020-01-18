# Sphero RVR
This projects extends your Sphero RVR with different functionality:

[] Security layer to avoid crashes etc.
[] Sound module (like R2D2)
[] Object detection


# Setup
## Software
Execute the following to install all dependencies and create a virtual environment:

1. git clone https://github.com/sphero-inc/sphero-sdk-raspberrypi-python
2. git clone https://github.com/peerdavid/sphero-rvr
3. ./sphero-sdk-raspberrypi-python/first-time-setup.sh
5. cd sphero-rvr
6. pipenv --python /usr/bin/python3.7
7. pipenv shell


## Hardware
For the hardware we need to connect the raspberry pi 4b to the sphero rvr, install the raspberry camera, install ultrasonic sensors (security layer) and speakers:

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