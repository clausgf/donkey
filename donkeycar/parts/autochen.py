import serial
import logging
from struct import *


class SpektrumRemoteReceiver:
    """
    Monitor a Spektrum Remote Receiver in 2048 Mode via the serial protocol.
    To remain synchronized, this part runs a separate thread.
    Specified in https://www.spektrumrc.com/ProdInfo/Files/Remote%20Receiver%20Interfacing%20Rev%20A.pdf
    Inspired by https://github.com/samfok/remote_receiver_tutorial

    The remote receiver must be connected to the following pins (Raspi Model 3):
    TODO

    TODO: Don't forget to enable the Raspi's serial port via raspi-config!

    TODO: Unfortunately, the Raspi is not fast enough for reliable
    synchronization, thus fiddle with realtime priorities and
    improve resynchronization.
    """

    def __init__(self, channels = (), serialPort="/dev/serial0"):
        self.channels = channels
        self.running = True
        self.timestamp = 0
        self.serialPort = serialPort
        self.serial = serial.Serial(port=self._serialPort, baudrate=115200,
                                    bytesize=serial.EIGHTBITS,
                                    parity=serial.PARITY_NONE,
                                    stopbits=serial.STOPBITS_ONE)

    def synchronize(self):
        """
        Synchronize with the remote receiver by detecting the gap
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
            dt = t-t_lastbit
            t_lastbit = t
        # We've found a gap between bytes of at least 5 ms!
        # skip the rest of the data packet (16 - 1 bytes)
        self._serial.read(15)

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
                normalized_position = ((position) - self.servo_offset[channel]) * self.servo_scale[channel]
                self.servo_positions[channel] = normalized_position
        self.timestamp = t

    def update(self):
        self.synchronize()
        while self.running:
            self.read()

    def run_threaded(self):
        out = (self.servo_positions[channel] for channel self.channels)
        return out

    def run(self):
        raise Exception("We expect for this part to be run with the threaded=True argument.")
        return False

    def shutdown(self):
        self.running = False
        time.sleep(0.5)


class DifferentialDriveActuator_MotorHat:
    """
    Differential Drive Robot Actuator
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
        self.run(0)  # stop vehicle


class AckermannToDifferentialDriveConverter:
    """
    Simulate Ackermann steering with differential drive.
    TODO privide a correct implementation
    """

    def __init__(self):

    def drive(self, speed, steering):

        def correction(speed):
            correction = 0
            if speed > 1:
                correction = 1 - speed
            if speed < -1:
                correction = -1 - speed
            return correction

        ml = speed + steering
        mr = speed - steering
        # at least one correction should be zero, thus the overall correction is:
        c = correction(ml) + correction(mr)
        #print('drive({:5.2f}, {:5.2f}) {:5.2f} {:5.2f} {:5.2f}'.format(speed, steering, ml, mr, c))
        return ml + c, mr + c

    def run(self, throttle, angle):
        ml, mr = self.drive(throttle, 0.5*angle)
        return ml, mr
