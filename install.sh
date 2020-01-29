#!/bin/bash

# Install tensorflow 2.1 and all requirements
sudo apt-get install -y libhdf5-dev libc-ares-dev libeigen3-dev
sudo apt-get install -y openmpi-bin libopenmpi-dev
sudo apt-get install -y libatlas-base-dev
pip3 install -U --user six wheel mock
wget https://github.com/PINTO0309/Tensorflow-bin/raw/master/tensorflow-2.1.0-cp37-cp37m-linux_armv7l.whl
sudo pip3 uninstall tensorflow
sudo -H pip3 install tensorflow-2.1.0-cp37-cp37m-linux_armv7l.whl

# Setup sphero python sdk
cd ..
git clone https://github.com/sphero-inc/sphero-sdk-raspberrypi-python
./sphero-sdk-raspberrypi-python/first-time-setup.sh

# Install all pip packages
# For TF see also https://www.tensorflow.org/lite/guide/python#install_just_the_tensorflow_lite_interpreter
pipenv --python /usr/bin/python3.7
pipenv shell