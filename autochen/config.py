"""
CAR CONFIG

This file is read by your car application's manage.py script to change the car
performance.

EXMAPLE
-----------
import dk
cfg = dk.load_config(config_path='~/mycar/config.py')
print(cfg.CAMERA_RESOLUTION)

"""


import os

#PATHS
CAR_PATH = PACKAGE_PATH = os.path.dirname(os.path.realpath(__file__))
DATA_PATH = os.path.join(CAR_PATH, 'data')
MODELS_PATH = os.path.join(CAR_PATH, 'models')

#VEHICLE
DRIVE_LOOP_HZ = 20
MAX_LOOPS = 100000

#ACKERMANN STEERING
ACKERMANN_LENGTH = 0.12
ACKERMANN_WIDTH = 0.13

#CAMERA
CAMERA_RESOLUTION = (120, 160) #(height, width)
CAMERA_FRAMERATE = DRIVE_LOOP_HZ

#MOTORHAT
MOTORHAT_ADDR = 0x60
MOTORHAT_LEFT_FRONT_ID = -4
MOTORHAT_LEFT_REAR_ID = -3
MOTORHAT_RIGHT_FRONT_ID = 2
MOTORHAT_RIGHT_REAR_ID = 1

#TRAINING
BATCH_SIZE = 128
TRAIN_TEST_SPLIT = 0.8

#SPEKTRUM REMOTE RECEIVER
SPEKTRUM_OFFSET = [-1024] * 16
SPEKTRUM_SCALE = [1.0/700] * 16
SPEKTRUM_SCALE[1] *= -1
SPEKTRUM_DEFAULT = [0.0] * 16
SPEKTRUM_DEFAULT[5] = 1
SPEKTRUM_SERIALPORT = "/dev/serial0"

#MOVE32 RECEIVER (805 us - 1630 us)
MOVE32_OFFSET = [-(805+(1630-805)/2)] * 16
MOVE32_SCALE = [1.0/((1630-805)/2)] * 16
MOVE32_SCALE[1] *= -1
MOVE32_DEFAULT = [0.0] * 16
MOVE32_DEFAULT[5] = 1
MOVE32_SERIALPORT = "/dev/serial0"
MOVE32_RXTYPE=1
MOVE32_RXAUTO=1
MOVE32_TIMEOUT=0.2

USE_JOYSTICK_AS_DEFAULT = False
JOYSTICK_MAX_THROTTLE = 0.25
JOYSTICK_STEERING_SCALE = 1.0
AUTO_RECORD_ON_THROTTLE = True

TUB_PATH = os.path.join(CAR_PATH, 'tub') # if using a single tub

#ROPE.DONKEYCAR.COM
ROPE_TOKEN="GET A TOKEN AT ROPE.DONKEYCAR.COM"
