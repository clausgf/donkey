#!/usr/bin/env python3
"""
Scripts to drive a donkey 2 car and train a model for it.

Usage:
    manage.py (drive) [--model=<model>] [--js] [--chaos]
    manage.py (train) [--tub=<tub1,tub2,..tubn>]  (--model=<model>) [--base_model=<base_model>] [--no_cache]

Options:
    -h --help        Show this screen.
    --tub TUBPATHS   List of paths to tubs. Comma separated. Use quotes to use wildcards. ie "~/tubs/*"
    --js             Use physical joystick.
    --chaos          Add periodic random steering when manually driving
"""
import os
from docopt import docopt

import donkeycar as dk
from donkeycar.redis_memory import RedisMemory

#import parts
from donkeycar.parts.camera import PiCamera
from donkeycar.parts.transform import Lambda
from donkeycar.parts.keras import KerasCategorical
from donkeycar.parts.datastore import TubGroup, TubWriter
from donkeycar.parts.controller import LocalWebController, JoystickController
from donkeycar.parts.clock import Timestamp
from donkeycar.parts.autochen import *


def drive(cfg, model_path=None, use_joystick=False, use_chaos=False):
    """
    Construct a working robotic vehicle from many parts.
    Each part runs as a job in the Vehicle loop, calling either
    it's run or run_threaded method depending on the constructor flag `threaded`.
    All parts are updated one after another at the framerate given in
    cfg.DRIVE_LOOP_HZ assuming each part finishes processing in a timely manner.
    Parts may have named outputs and inputs. The framework handles passing named outputs
    to parts requesting the same named input.
    """

    memory = RedisMemory()
    V = dk.vehicle.Vehicle(mem=memory)

    clock = Timestamp()
    V.add(clock, outputs='timestamp')

    # ***** CAMERA *****
    print("Starting camera")
    cam = PiCamera(resolution=cfg.CAMERA_RESOLUTION)
    V.add(cam, outputs=['cam/image_array'], threaded=True)

    # ***** Web Controller *****
    print("Starting web controller")
    ctr = LocalWebController(use_chaos=use_chaos)
    V.add(ctr,
          inputs=['cam/image_array'],
          outputs=['user/angle', 'user/throttle', 'user/mode', 'recording'],
          threaded=True)

    # ***** SPEKTRUM/MOVE32 REMOTE *****
    print("Starting Spektrum/Move32")
    #rc = SpektrumRemoteReceiver(cfg.SPEKTRUM_OFFSET, cfg.SPEKTRUM_SCALE, cfg.SPEKTRUM_DEFAULT, cfg.SPEKTRUM_SERIALPORT)
    rc = Move32Receiver(cfg.MOVE32_OFFSET, cfg.MOVE32_SCALE, cfg.MOVE32_DEFAULT, cfg.MOVE32_SERIALPORT, cfg.MOVE32_RXTYPE, cfg.MOVE32_RXAUTO, cfg.MOVE32_TIMEOUT)
    V.add(rc, threaded=True, outputs=['rc0', 'rc1', 'rc2', 'rc3', 'rc4', 'rc5', 'rc6', 'rc7'])

    def rc_convert_func(*args):
        angle = args[0]
        throttle = args[1]
        mode = 'manual' if args[2] > 0.3 else 'auto' if args[2] < -0.3 else 'auto_angle'
        recording = args[3] <= 0.3
        return angle, throttle, mode, recording
    V.add(Lambda(rc_convert_func),
          inputs=['rc0','rc2','rc5','rc4'],
          outputs=['user/angle', 'user/throttle', 'user/mode', 'recording'])

    # ***** user/mode -> run_pilot *****
    V.add(Lambda(lambda mode: mode.lower() != 'manual'),
          inputs=['user/mode'],
          outputs=['run_pilot'])

    # ***** cam/image_array -> pilot/angle,pilot_throttle *****
    # Run the pilot if the mode is not user.
    print("Starting KerasCategorical")
    kl = KerasCategorical()
    if model_path:
        print("Loading model...")
        kl.load(model_path)
    V.add(kl, inputs=['cam/image_array'],
              outputs=['pilot/angle', 'pilot/throttle'],
              run_condition='run_pilot')

    # ***** user/*, pilot/* -> angle, throttle *****
    # Choose what inputs should change the car.
    def drive_mode(mode,
                   user_angle, user_throttle,
                   pilot_angle, pilot_throttle):
        if mode == 'manual':
            return user_angle, user_throttle
        elif mode == 'auto_angle':
            return pilot_angle, user_throttle
        else:
            return pilot_angle, pilot_throttle
    drive_mode_part = Lambda(drive_mode)
    V.add(drive_mode_part,
          inputs=['user/mode', 'user/angle', 'user/throttle',
                  'pilot/angle', 'pilot/throttle'],
          outputs=['angle', 'throttle'])

    # ***** throttle, angle -> motor_left, motor_right *****
    ackermann_to_diff_converter = AckermannToDifferentialDriveConverter(
                cfg.ACKERMANN_LENGTH,
                cfg.ACKERMANN_WIDTH)
    V.add(ackermann_to_diff_converter,
          inputs=['angle', 'throttle'],
          outputs=['motor_left', 'motor_right'])

    # ***** motor_left, motor_right -> DRIVE *****
    motors_part = DifferentialDriveActuator_MotorHat(
                cfg.MOTORHAT_ADDR,
                cfg.MOTORHAT_LEFT_FRONT_ID,
                cfg.MOTORHAT_LEFT_REAR_ID,
                cfg.MOTORHAT_RIGHT_FRONT_ID,
                cfg.MOTORHAT_RIGHT_REAR_ID)
    V.add(motors_part, inputs=['motor_left', 'motor_right'])

    # ***** output debug data *****
    debug_keys = ['user/mode', 'recording', 'run_pilot', "angle", "throttle", "motor_left", "motor_right",
            ]#'rc1', 'rc2', 'rc3', 'rc4', 'rc5', 'rc6', 'rc7', 'rc8']
    def debug_func(*args):
        print(args[0], " ", args[1], " ", args[2], " ".join("{:5.2f}".format(e) for e in args[3:]))
    V.add(Lambda(debug_func), inputs=debug_keys)

    # add tub to save data
    inputs = ['cam/image_array', 'user/angle', 'user/throttle', 'user/mode', 'timestamp']
    types = ['image_array', 'float', 'float',  'str', 'str']

    #multiple tubs
    #th = TubHandler(path=cfg.DATA_PATH)
    #tub = th.new_tub_writer(inputs=inputs, types=types)

    # single tub
    tub = TubWriter(path=cfg.TUB_PATH, inputs=inputs, types=types)
    V.add(tub, inputs=inputs, run_condition='recording')

    # run the vehicle
    V.start(rate_hz=cfg.DRIVE_LOOP_HZ,
            max_loop_count=cfg.MAX_LOOPS)


def train(cfg, tub_names, new_model_path, base_model_path=None ):
    """
    use the specified data in tub_names to train an artifical neural network
    saves the output trained model as model_name
    """
    X_keys = ['cam/image_array']
    y_keys = ['user/angle', 'user/throttle']
    def train_record_transform(record):
        """ convert categorical steering to linear and apply image augmentations """
        record['user/angle'] = dk.util.data.linear_bin(record['user/angle'])
        # TODO add augmentation that doesn't use opencv
        return record

    def val_record_transform(record):
        """ convert categorical steering to linear """
        record['user/angle'] = dk.util.data.linear_bin(record['user/angle'])
        return record

    new_model_path = os.path.expanduser(new_model_path)

    kl = KerasCategorical()
    if base_model_path is not None:
        base_model_path = os.path.expanduser(base_model_path)
        kl.load(base_model_path)

    print('tub_names', tub_names)
    if not tub_names:
        tub_names = os.path.join(cfg.DATA_PATH, '*')
    tubgroup = TubGroup(tub_names)
    train_gen, val_gen = tubgroup.get_train_val_gen(X_keys, y_keys,
                                                    train_record_transform=train_record_transform,
                                                    val_record_transform=val_record_transform,
                                                    batch_size=cfg.BATCH_SIZE,
                                                    train_frac=cfg.TRAIN_TEST_SPLIT)

    total_records = len(tubgroup.df)
    total_train = int(total_records * cfg.TRAIN_TEST_SPLIT)
    total_val = total_records - total_train
    print('train: %d, validation: %d' % (total_train, total_val))
    steps_per_epoch = total_train // cfg.BATCH_SIZE
    print('steps_per_epoch', steps_per_epoch)

    kl.train(train_gen,
             val_gen,
             saved_model_path=new_model_path,
             steps=steps_per_epoch,
             train_split=cfg.TRAIN_TEST_SPLIT)


if __name__ == '__main__':
    args = docopt(__doc__)
    cfg = dk.load_config()

    if args['drive']:
        drive(cfg, model_path = args['--model'], use_joystick=args['--js'], use_chaos=args['--chaos'])

    elif args['train']:
        tub = args['--tub']
        new_model_path = args['--model']
        base_model_path = args['--base_model']
        cache = not args['--no_cache']
        train(cfg, tub, new_model_path, base_model_path)