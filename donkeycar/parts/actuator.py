"""
actuators.py
Classes to control the motors and servos. These classes
are wrapped in a mixer class before being used in the drive loop.
"""

import time
import donkeycar as dk


class PCA9685:
    """
    PWM motor controler using PCA9685 boards.
    This is used for most RC Cars
    """
    def __init__(self, channel, frequency=60):
        import Adafruit_PCA9685
        # Initialise the PCA9685 using the default address (0x40).
        self.pwm = Adafruit_PCA9685.PCA9685()
        self.pwm.set_pwm_freq(frequency)
        self.channel = channel

    def set_pulse(self, pulse):
        self.pwm.set_pwm(self.channel, 0, pulse)

    def run(self, pulse):
        self.set_pulse(pulse)


class PWMSteering:
    """
    Wrapper over a PWM motor cotnroller to convert angles to PWM pulses.
    """
    LEFT_ANGLE = -1
    RIGHT_ANGLE = 1

    class Settings:
        inputs = ['angle']
        left_pulse = 290
        right_pulse = 490

    def __init__(self, controller=None,
                 left_pulse=290, right_pulse=490):

        self.controller = controller
        self.left_pulse = left_pulse
        self.right_pulse = right_pulse

    def run(self, angle):
        # map absolute angle to angle that vehicle can implement.
        pulse = dk.util.data.map_range(
            angle,
            self.LEFT_ANGLE, self.RIGHT_ANGLE,
            self.left_pulse, self.right_pulse
        )

        self.controller.set_pulse(pulse)

    def shutdown(self):
        self.run(0)  # set steering straight


class PWMThrottle:
    """
    Wrapper over a PWM motor cotnroller to convert -1 to 1 throttle
    values to PWM pulses.
    """
    MIN_THROTTLE = -1
    MAX_THROTTLE = 1

    def __init__(self,
                 controller=None,
                 max_pulse=300,
                 min_pulse=490,
                 zero_pulse=350):

        self.controller = controller
        self.max_pulse = max_pulse
        self.min_pulse = min_pulse
        self.zero_pulse = zero_pulse

        # send zero pulse to calibrate ESC
        self.controller.set_pulse(self.zero_pulse)
        time.sleep(1)

    def run(self, throttle):
        if throttle > 0:
            pulse = dk.util.data.map_range(throttle,
                                           0, self.MAX_THROTTLE,
                                           self.zero_pulse, self.max_pulse)
        else:
            pulse = dk.util.data.map_range(throttle,
                                           self.MIN_THROTTLE, 0,
                                           self.min_pulse, self.zero_pulse)

        self.controller.set_pulse(pulse)

    def shutdown(self):
        self.run(0)  # stop vehicle


class Adafruit_DCMotor_Hat:
    """
    Adafruit DC Motor Controller
    Used for each motor on a differential drive car.
    """
    def __init__(self, motor_num):
        from Adafruit_MotorHAT import Adafruit_MotorHAT
        import atexit

        self.FORWARD = Adafruit_MotorHAT.FORWARD
        self.BACKWARD = Adafruit_MotorHAT.BACKWARD
        self.mh = Adafruit_MotorHAT(addr=0x60)

        self.motor = self.mh.getMotor(motor_num)
        self.motor_num = motor_num

        atexit.register(self.turn_off_motors)
        self.speed = 0
        self.throttle = 0

    def run(self, speed):
        """
        Update the speed of the motor where 1 is full forward and
        -1 is full backwards.
        """
        if speed > 1 or speed < -1:
            raise ValueError("Speed must be between 1(forward) and -1(reverse)")

        self.speed = speed
        self.throttle = int(dk.util.data.map_range(abs(speed), -1, 1, -255, 255))

        if speed > 0:
            self.motor.run(self.FORWARD)
        else:
            self.motor.run(self.BACKWARD)

        self.motor.setSpeed(self.throttle)

    def shutdown(self):
        self.mh.getMotor(self.motor_num).run(Adafruit_MotorHAT.RELEASE)


class DifferentialDriveWithMotorHat:
    """
    Differential Drive Robot Controllter
    for either two or four motors controlled using the Adafruit Motor Hat
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

        def set_speed(self, speed):
            if motor_id < 0:
                return
            if speed > 1 or speed < -1:
                raise ValueError("Speed must be between 1(forward) and -1(reverse)")
            mh_speed = int(abs(255*speed))
            mh_speed = max(0, min(255, speed))
            if speed > -1.0/255 and speed < -1.0/255:
                self.mh.run(self.RELEASE)
                self.mh.setSpeed(0)
            elif speed < 0:
                self.mh.run(self.RELEASE)
                self.mh.setSpeed(mh_speed)
                self.mh.run(self.BACKWARD)
            else:
                self.mh.run(self.RELEASE)
                self.mh.setSpeed(mh_speed)
                self.mh.run(self.FORWARD)

    def __init__(self,
                 addr=0x60,
                 left_front_id=1, left_rear_id=2,
                 right_front_id=3, right_rear_id=4):
        from Adafruit_MotorHAT import Adafruit_MotorHAT
        self.mh = Adafruit_MotorHAT(addr)
        self.lf = MotorWithMotorHat(self.mh, left_front_id)
        self.lr = MotorWithMotorHat(self.mh, left_rear_id)
        self.rf = MotorWithMotorHat(self.mh, right_front_id)
        self.rr = MotorWithMotorHat(self.mh, right_rear_id)
        self.stop_motors()

    def set_motors(self, lf_speed, lr_speed, rf_speed, rr_speed):
        self.lf.set_speed(lf_speed)
        self.lr.set_speed(lr_speed)
        self.rf.set_speed(rf_speed)
        self.rr.set_speed(rr_speed)

    def stop_motors():
        self.set_motors(0, 0, 0, 0)

    def drive(self, speed, steering):

        def correction(speed):
            correction = 0
            if speed > 1:
                correction = 1 - speed
            if speed < -1:
                correction = -1 - speed
            return correction

        ml = speed + direction
        mr = speed - direction
        # at least one correction should be zero, thus the overall correction is:
        c = correction(ml) + correction(mr)
        self.set_motors(ml + c, ml + c, mr + c, mr + c)

    def run(self, throttle, angle):
        self.drive(throttle, angle)

    def shutdown(self):
        self.run(0)  # stop vehicle
