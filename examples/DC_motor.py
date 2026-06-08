#!/usr/bin/env/python3
import time
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import motor
import curses

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

        self.stopped = 1
        self.motorStop()

    def motorStop():  # Motor stops
        self.motor1.throttle = 0
        self.stopped = 1

    def _map(x, in_min, in_max, out_min, out_max):
        return (x - in_min) / (in_max - in_min) * (out_max - out_min) + out_min

    def destroy():
        self.motorStop()
        pwm_motor.deinit()

    def _power(direction, motor_speed):
        '''
        Power the motor
        '''
        if motor_speed > 100:
            motor_speed = 100
        elif motor_speed < 0:
            motor_speed = 0
        speed = map(motor_speed, 0, 100, 0, 1.0)
        if direction == -1:
            speed = -speed
        motor1.throttle = speed

    def _ramp(self, stop_time, direction, speed):
        '''
        Slowly accelerate
        '''
        if stop_time > 1:
            stop_time = 1
        step = speed / (stop_time*10)
        actual_speed = 0
        for i in range(0, stop_time*10):
            actual_speed += step
            self._power(direction, actual_speed)
            time.sleep(0.1)

    def control(self, direction, motor_speed, duration=1, ramp=1, slow=0):
        '''
        Command to move forward or backward, movement won't stop until stopped (DC_Motor.motoStop())
        '''
        if self.stopped:
            self.stopped = 0
        # if slow mode, no ramp and 1/4 power (25)
        if slow == 1:
            _power(direction, 25)
        # else ramp
        else:
            if duration < 1:
                self._ramp(duration, direction, motor_speed)
                self.motorStop()
            else:
                self._ramp(1, direction, motor_speed)

    # def run(self, ramp=1, slow=0):
    #     '''
    #     Run by reacting to the keyboard
    #     '''
    #     def main(stdscr):
    #         stdscr.nodelay(True)
    #         while True:
    #             c = stdscr.getch()
    #             if c == ord('q'):
    #                 print("You pressed 'q'")
    #                 break
    #             # Your existing code here
    #             time.sleep(0.1)
    #
    #     curses.wrapper(main)




if __name__ == '__main__':
    try:
        my_motor = DC_Motor()
        for i in range(10):
            my_motor.control(1, 100)

            print("Forward")
            time.sleep(3)
            my_motor.control(-1, 100)
            print("Backward")
            time.sleep(3)
        destroy()
    except KeyboardInterrupt:
        destroy()

