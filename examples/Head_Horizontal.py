import time
from board import SCL, SDA
import busio
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685

i2c = busio.I2C(SCL, SDA)
# Create a simple PCA9685 class instance.
pca = PCA9685(i2c, address=0x5f) #default 0x40

pca.frequency = 50

# The pulse range is 750 - 2250 by default. This range typically gives 135 degrees of
# range, but the default is to use 180 degrees. You can specify the expected range if you wish:
class servo_head(ID):
    def __init__(self):
        self.servo = servo.Servo(pca.channels[ID], min_pulse=500, max_pulse=2400,actuation_range=135)
        servo.angle = 180
        self.angle = servo.angle

    def set_angle(angle):
        self.angle = angle

    def right():
        for i in range(180):
            set_angle(channel, 180)
            time.sleep(0.01)

    def left():
        for i in range(180):
            set_angle(channel, 180 - i)
            time.sleep(0.01)

    def test(self):
        while True:
            print("Right")
            self.right()
            print("Left")
            self.left()

if __name__ == "__main__":
    head = servo_head(1)

