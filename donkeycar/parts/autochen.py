import serial
import time
import math
import numpy as np
import logging
from struct import *


class Move32Receiver:
    """Use a move32 controller based on a cheap Naze32 quadrocopter flight
    controller to receive signals form an rc receiver; could be extended
    to output signals for servos/ESCs.
    """

    def __init__(self, channel_offset, channel_scale, channel_default,
                 serialPort="/dev/serial0", rx_type=1, rx_auto=1, timeout=0.2):
        """Construct a controller for a serial Move32 module, which is
        connected to a rc receiver (no support for servo/ESC).

        Parameters:
        channel_offset: list of offsets to add to each channel before scaling
        channel_scale: list of scaling factors for each channel after adding the offset
        channel_default: list of default for each channel after timeout
        serialPort: path to serial interface device file
        rx_type: Move32 receiver type, 0=full receiver, 1=CPPM receiver
        rx_auto: Move32 auto rx report interval, 0: off; 1: onChange; >1: interval in ms
        timeout: timeout before transmitting default values
        """
        self.running = True
        self.timestamp = 0
        self.channel_offset = channel_offset
        self.channel_scale = channel_scale
        self.channel_default = channel_default
        self.channels = channel_default
        self.serialPort = serialPort
        self.rx_type = rx_type
        self.rx_auto = rx_auto
        self.timeout = timeout
        self.serial = serial.Serial(port=self.serialPort, timeout=self.timeout,
                                    baudrate=115200,
                                    bytesize=serial.EIGHTBITS,
                                    parity=serial.PARITY_NONE,
                                    stopbits=serial.STOPBITS_ONE)

    def update(self):
        self.serial.write('rx_type {}\n'.format(self.rx_type).encode('utf-8'))
        self.serial.write('rx_auto {}\n'.format(self.rx_auto).encode('utf-8'))
        while self.running:
            line = self.serial.readline().decode('utf-8')
            answer = line.split()
            if len(answer) == 2 and answer[0] == 'ok':
                # we've receive a command confirmation
                pass
            elif len(answer) > 3 and answer[0] == 'ok' and answer[1] == 'rx':
                # we've received something like this: ok rx 10000 1500 1500 1500 ...
                t = answer[2]
                channels = [ int(c) for c in answer[3:] ]
                for i in range(0, len(channels)-1):
                    self.channels[i] = (channels[i] + self.channel_offset[i]) * self.channel_scale[i]
                self.timestamp = time.time()
            else:
                # we're unable to parse the answer
                pass

    def run_threaded(self):
        if (time.time() - self.timestamp) < self.timeout:
            out = self.channels
        else:
            out = self.channel_default
        return list(out)

    def run(self):
        raise Exception("We expect for this part to be run with the threaded=True argument.")
        return False

    def shutdown(self):
        self.running = False


class SpektrumRemoteReceiver:
    """Monitor a Spektrum Remote Receiver in 2048 Mode via the serial protocol.
    To remain synchronized, this part runs a separate thread.
    Specified in https://www.spektrumrc.com/ProdInfo/Files/Remote%20Receiver%20Interfacing%20Rev%20A.pdf
    Inspired by https://github.com/samfok/remote_receiver_tutorial

    The remote receiver must be connected to the following pins (Raspi Model 3): TODO
    Don't forget to enable the serial port via raspi-config!

    TODO: Unfortunately, the Raspi is not fast enough for reliable
    synchronization in auto mode - manual mode, however, works find;
    idea: fiddle with realtime priorities and improve resynchronization.
    """

    def __init__(self, servo_offset, servo_scale, servo_default,
                 serialPort="/dev/serial0", timeout=0.2):
        self.running = True
        self.timestamp = 0
        self.servo_offset = servo_offset
        self.servo_scale = servo_scale
        self.servo_default = servo_default
        self.servo_positions = servo_default
        self.serialPort = serialPort
        self.timeout = timeout
        self.serial = serial.Serial(port=self.serialPort, baudrate=115200,
                                    bytesize=serial.EIGHTBITS,
                                    parity=serial.PARITY_NONE,
                                    stopbits=serial.STOPBITS_ONE)

    def synchronize(self):
        """Synchronize with the remote receiver by detecting the gap
        between successive packets based on their timing.
        Rationale from https://github.com/samfok/remote_receiver_tutorial:
        Packets are communicated every 11ms. At 115200 bps, a bit is read in
        approximately 8.69us, so a 16 byte (128 bit) packet will take around
        1.11ms to be communicated, leaving a gap of about 9.89ms between
        packets.
        """
        dt = 0
        dt_threshold = 0.005
        # start reading and find a byte preceded by a gap
        self.serial.read(1)   # read the first byte
        t_lastbit = time.time()
        while dt < dt_threshold:
            self.serial.read(1)
            t = time.time()
            dt = t - t_lastbit
            t_lastbit = t
        # We've found a gap between bytes of at least 5 ms!
        # skip the rest of the data packet (16 - 1 bytes)
        self.serial.read(15)

    def read(self):
        """
        Read 16 byte and decode them into 8 16-Bit integers.
        """
        raw_data = self.serial.read(16)
        t = time.time()
        data = unpack('>HHHHHHHH', raw_data)
        # just assume 2048 Mode:
        # Bit 15 Servo Phase
        # Bit 14-11: Channel ID
        # Bit 10-0 Servo Position
        fade = data[0]
        for i in range(1,8):
            channel = (data[i] & 0x7800) >> 11  # bit 14-11
            position = (data[i] & 0x07ff)       # bit 10-0
            if channel < len(self.servo_positions):
                normalized_position = ((position) + self.servo_offset[channel]) * self.servo_scale[channel]
                self.servo_positions[channel] = normalized_position
        self.timestamp = t

    def update(self):
        self.synchronize()
        while self.running:
            self.read()

    def run_threaded(self):
        if (time.time() - self.timestamp) < self.timeout:
            out = self.servo_positions
        else:
            out = self.servo_default
        return list(out)

    def run(self):
        raise Exception("We expect for this part to be run with the threaded=True argument.")
        return False

    def shutdown(self):
        self.running = False


class DifferentialDriveActuator_MotorHat:
    """Differential Drive Robot Actuator
    for either two or four motors controlled using the Adafruit Motor Hatself.
    The motor_id parameters of the constructor (left_front_id...) are the
    motor head ids of the respective motor. Use a motor_id < 0 to reverse
    any speed given to a motor, and motor_id = 0 if the respective motor does
    not exist (e.g. in a two-wheel/three-wheel differential drive bot).
    """

    class MotorWithMotorHat:
        """
        Controller for a single motor using the Adafruit Motor Hat
        """
        def __init__(self, motor_hat, motor_id):
            from Adafruit_MotorHAT import Adafruit_MotorHAT
            self.FORWARD = Adafruit_MotorHAT.FORWARD
            self.BACKWARD = Adafruit_MotorHAT.BACKWARD
            self.RELEASE = Adafruit_MotorHAT.RELEASE
            self.mh = motor_hat
            self.motor_id = motor_id
            self.motor = self.mh.getMotor(abs(self.motor_id))

        def set_speed(self, speed):
            if self.motor_id == 0:
                return
            if speed > 1 or speed < -1:
                raise ValueError("Speed must be between 1(forward) and -1(reverse)")
            if self.motor_id < 0:
                speed = -speed  # reverse motor direction
            mh_speed = int(abs(255*speed))
            mh_speed = max(0, min(255, mh_speed))
            if speed > -1.0/255 and speed < -1.0/255:
                self.motor.run(self.RELEASE)
                self.motor.setSpeed(0)
            elif speed < 0:
                self.motor.run(self.RELEASE)
                self.motor.setSpeed(mh_speed)
                self.motor.run(self.BACKWARD)
            else:
                self.motor.run(self.RELEASE)
                self.motor.setSpeed(mh_speed)
                self.motor.run(self.FORWARD)

    def __init__(self,
                 addr=0x60,
                 left_front_id=1, left_rear_id=2,
                 right_front_id=3, right_rear_id=4):
        from Adafruit_MotorHAT import Adafruit_MotorHAT
        import atexit
        self.mh = Adafruit_MotorHAT(addr)
        self.lf = self.MotorWithMotorHat(self.mh, left_front_id)
        self.lr = self.MotorWithMotorHat(self.mh, left_rear_id)
        self.rf = self.MotorWithMotorHat(self.mh, right_front_id)
        self.rr = self.MotorWithMotorHat(self.mh, right_rear_id)
        self.stop_motors()
        # call of the shutdown method does not seem to be reliable, thus
        # register an own exit handler
        atexit.register(self.stop_motors)

    def set_motors(self, lf_speed, lr_speed, rf_speed, rr_speed):
        self.lf.set_speed(lf_speed)
        self.lr.set_speed(lr_speed)
        self.rf.set_speed(rf_speed)
        self.rr.set_speed(rr_speed)
        #print('set_motors {:5.2f} {:5.2f} {:5.2f} {:5.2f}'.format(lf_speed, lr_speed, rf_speed, rr_speed))

    def stop_motors(self):
        self.set_motors(0, 0, 0, 0)

    def run(self, left_speed, right_speed):
        self.set_motors(left_speed, left_speed, right_speed, right_speed)

    def shutdown(self):
        self.stop_motors()


class AckermannToDifferentialDriveConverter:
    """Simulate Ackermann steering with two-wheel differential drive robot.
    In a four-wheel differential drive, motors on each side could be controlled
    in the same way.
    This class expects a steeering parameter in terms of inverse radius of
    curvature normalized to the vehicle length, $s=L/r$. The other control
    parameter is the speed of the tangential
    motion of the robot's center.
    """

    def __init__(self, length, width):
        """Construct Ackermann Steering simulation with differential drive robot.

        Parameters:
        length: length from axle to axle
        width: width form wheel to wheel measured along an axle
        """
        self.length = length
        self.width = width

    def limit_motors(self, ml, mr):

        def correction(speed):
            correction = 0
            if speed > 1:
                correction = 1 - speed
            if speed < -1:
                correction = -1 - speed
            return correction

        # at least one correction should be zero, thus the overall correction is:
        corr = correction(ml) + correction(mr)
        return ml + corr, mr + corr

    def run(self, steering, throttle):
        """Compute left and right motors' speed from steering angle and throttle.
        The steering parameter has priority over the speed, i.e. the speed is
        automatically reduces when necessary to steer the correct angle.
        With a given steering parameter, the method tries to
        drive on the same circle independent of the speed parameter $v$.

        Parameters:
        steering: Steering parameter in terms of inverse radius
        of curvature normalized to the vehicle length, $s=L/r$.
        *Steering* ranges from
        $-1$ (tight left turn) over $0$ (straight) to $1$ (tight right turn).
        throttle: Tangential speed $v$ the vehicle's center (arbitrary units).
        *Throttle* ranges from -1 (full speed backwards) over 0 (stop) to
        1 (full speed forward).
        """
        l = self.length
        w = self.width
        s = np.clip(steering, -1, 1)
        v = np.clip(throttle, -1, 1)
        vi = v * math.sqrt( (1 - w/2 * s/l) ** 2 + (s/2) ** 2 )
        vo = v * math.sqrt( (1 + w/2 * s/l) ** 2 + (s/2) ** 2 )
        ml, mr = self.limit_motors(vo, vi)
        #print("ACKERMANN {:5.2f} {:5.2f} {:5.2f}/{:5.2f} {:5.2f}/{:5.2f}".format( steering, throttle, vi, vo, ml, mr))
        return ml, mr
