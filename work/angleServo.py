import time
from board import SCL, SDA
import busio
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685


i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c, address=0x5f) #default 0x40
pca.frequency = 50

def getAngle(ID):
    servo_angle = servo.Servo(pca.channels[ID], min_pulse=500, max_pulse=2400, actuation_range=180)
    print("wheel_angle,", servo_angle.angle)
    time.sleep(0.5)

if __name__ == "__main__":
    channel = 0
    while True:
        getAngle(channel)