#!/usr/bin/env/python
# File name   : BatteryLevelMonitoring.py
# Website     : www.Adeept.com
# Author      : Adeept
# Date        : 2025/03/11




import time
import board
import busio
from adafruit_bus_device.i2c_device import I2CDevice
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import motor
from adafruit_motor import servo


# FOR THE MOTOR
MOTOR_M1_IN1 = 15  # Define the positive pole of M1
MOTOR_M1_IN2 = 14  # Define the negative pole of M1

def map(x, in_min, in_max, out_min, out_max):
    return (x - in_min) / (in_max - in_min) * (out_max - out_min) + out_min

# def setup():
i2c = busio.I2C(SCL, SDA)
# Create a simple PCA9685 class instance.
#  pca.channels[7].duty_cycle = 0xFFFF
pca = PCA9685(i2c, address=0x5f)  # default 0x40
pca.frequency = 50

motor1 = motor.DCMotor(pca.channels[MOTOR_M1_IN1], pca.channels[MOTOR_M1_IN2])
motor1.decay_mode = (motor.SLOW_DECAY)

#  motorStop()
def Motor(channel, direction, motor_speed):
    if motor_speed > 100:
        motor_speed = 100
    elif motor_speed < 0:
        motor_speed = 0
    speed = map(motor_speed, 0, 100, 0, 1.0)
    if direction == -1:
        speed = -speed

    if channel == 1:
        motor1.throttle = speed


def motorStop():  # Motor stops
    motor1.throttle = 0


def destroy():
    motorStop()
    pca.deinit()


# FOR THE BATTERY

# ADS7830 adress 0x48
device = I2CDevice(i2c, 0x48)

# Define constants
Vref = 8.4
WarningThreshold = 6.75
R15 = 3000
R17 = 1000
DivisionRatio = R17 / (R15 + R17)

#Define the ADC channel and command.
cmd = 0x84
channel = 0
control_byte = cmd | (((channel << 2 | channel >> 1) & 0x07) << 4)

NB_VAL_AVG = 40

if __name__ == "__main__":
    buffer = [1]
    battery_perc_vals = []

    Motor(1, 1, 50)
    servo_angle = servo.Servo(pca.channels[0], min_pulse=500, max_pulse=2400, actuation_range=180)
    servo_angle.angle = 30
    try:
        # avance et tourne
        while True:
            device.write_then_readinto(bytes([control_byte]), buffer)
            adcValue = buffer[0]
            A0Voltage = (adcValue / 255) * 5
            ActualBatteryVoltage = A0Voltage / DivisionRatio

            BatteryPercentage = (ActualBatteryVoltage - WarningThreshold) / (Vref - WarningThreshold) * 100
            battery_perc_vals.append(BatteryPercentage)

            if len(battery_perc_vals) == NB_VAL_AVG:
                battery_perc_vals = battery_perc_vals[1:]
                avg_bat = sum(battery_perc_vals) / NB_VAL_AVG
                print(f"Current battery level: {avg_bat:.2f} %")

            time.sleep(0.5)
    except KeyboardInterrupt:
        destroy()

