#!/usr/bin/env/python3
import time
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import motor
import curses
import math

# motor_EN_A: Pin7  |  motor_EN_B: Pin11
# motor_A:  Pin8,Pin10    |  motor_B: Pin13,Pin12


class DC_Motor():
    def __init__(self):
        MOTOR_M1_IN1 = 15  # Define the positive pole of M1
        MOTOR_M1_IN2 = 14  # Define the negative pole of M1
        # def setup():
        i2c = busio.I2C(SCL, SDA)
        # Create a simple PCA9685 class instance.
        #  pwm_motor.channels[7].duty_cycle = 0xFFFF
        self.pwm_motor = PCA9685(i2c, address=0x5f)  # default 0x40
        self.pwm_motor.frequency = 50

        self.motor1 = motor.DCMotor(self.pwm_motor.channels[MOTOR_M1_IN1], self.pwm_motor.channels[MOTOR_M1_IN2])
        self.motor1.decay_mode = (motor.SLOW_DECAY)

        self.actual_speed = 0
        self.actual_direction = 0

        self.motorStop()

    def motorStop(self):  # Motor stops
        self._ramp(0.2, 0)
        self.motor1.throttle = 0

    def _map(self, x, in_min, in_max, out_min, out_max):
        return (x - in_min) / (in_max - in_min) * (out_max - out_min) + out_min

    def destroy(self):
        self.motorStop()
        self.pwm_motor.deinit()

    def _power(self, direction, motor_speed):
        '''
        Power the motor
        '''
        if motor_speed > 100:
            motor_speed = 100
        elif motor_speed < 0:
            motor_speed = 0
        speed = self._map(motor_speed, 0, 100, 0, 1.0)
        if direction == -1:
            speed = -speed
        self.motor1.throttle = speed

    def _ramp(self, stop_time, speed):
        '''
        Slowly accelerate or stop
        '''
        if stop_time > 1:
            stop_time = 1

        if speed == 0:
            step = self.actual_speed / (stop_time * 10)
        else:
            step = speed / (stop_time*10)

        for i in range(0, math.round(stop_time*10)):
            if speed == 0:
                self.actual_speed -= step
            else:
                self.actual_speed += step
            self._power(self.actual_direction, self.actual_speed)
            time.sleep(0.1)

    def control(self, direction, motor_speed, duration=1, ramp=1, slow=0):
        '''
        Command to move forward or backward, movement won't stop until stopped (DC_Motor.motoStop())
        '''
        if self.stopped:
            self.stopped = 0
        # if slow mode, no ramp and 1/4 power (25)
        if direction != self.actual_direction:
            self.motorStop()
        self.actual_direction = direction
        if slow == 1:
            self._power(self.actual_direction, 25)
        # else ramp
        else:
            if duration < 1:
                self._ramp(duration, motor_speed)
                self.motorStop()
            else:
                self._ramp(1, motor_speed)


if __name__ == '__main__':

    #To controle with the keyboard

    stdscr.nodelay(True)
    while True:
        c = stdscr.getch()
        if c == ord('z'):
            print("You pressed 'q'")
            break
        # Your existing code here
        time.sleep(0.1)


    curses.wrapper(main)

    #To test forward and backward

    # my_motor = DC_Motor()
    # try:
    #     for i in range(10):
    #         my_motor.control(1, 100)
    #
    #         print("Forward")
    #         time.sleep(3)
    #         my_motor.control(-1, 100)
    #         print("Backward")
    #         time.sleep(3)
    #     my_motor.destroy()
    # except KeyboardInterrupt:
    #     my_motor.destroy()

