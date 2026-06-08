import time
from board import SCL, SDA
import busio
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685

i2c = busio.I2C(SCL, SDA)
# Create a simple PCA9685 class instance.
pca = PCA9685(i2c, address=0x5f) #default 0x40

pca.frequency = 50

class Head():

    class servo_motor():
        def __init__(self,ID):
            self.servo = servo.Servo(pca.channels[ID], min_pulse=500, max_pulse=2400, actuation_range=180)
            self.sleep = 0.01

            while (self.servo.angle >= 90):
                self.servo.angle = self.servo.angle - 1
            while (self.servo.angle <= 90):
                self.servo.angle = self.servo.angle + 1
            self.servo.angle = 90

        def set_angle(self, angle):
            self.servo.angle = angle

        def left(self, max):
            for i in range(int(max)):
                if (self.servo.angle + 1) >= 170:
                    break
                else:
                    self.set_angle(self.servo.angle + 1)
                    time.sleep(self.sleep)

        def right(self, max):
            for i in range(int(max)):
                if (self.servo.angle - 1)<= 10:
                    break
                else:
                    self.set_angle(self.servo.angle - 1)
                    time.sleep(self.sleep)

        def finish(self):
            if (self.servo.angle >= 90):
                max = self.servo.angle -90
                self.right(max)
            else:
                max = 90 - self.servo.angle
                self.left(max)
            time.sleep(0.05)
            self.servo.angle = 90
            time.sleep(0.5)

    def __init__(self):
        self.horizontal = Head.servo_motor(1)
        self.vertical = Head.servo_motor(2)
        self.weel = Head.servo_motor(0)

    def test(self):
        for i in range (30):
            self.horizontal.set_angle(self.horizontal.servo.angle + 1)
            self.vertical.set_angle(self.vertical.servo.angle + 1)
            self.weel.set_angle(self.weel.servo.angle + 1)
            time.sleep(self.horizontal.sleep)

        print("Step1")
        time.sleep(self.horizontal.sleep)

        for i in range (30):
            self.horizontal.set_angle(self.horizontal.servo.angle - 1)
            self.vertical.set_angle(self.vertical.servo.angle - 1)
            self.weel.set_angle(self.weel.servo.angle - 1)
            time.sleep(self.horizontal.sleep)

        print("Step2")
        time.sleep(self.horizontal.sleep)

    def finish(self):
        self.horizontal.finish()
        self.vertical.finish()
        self.weel.finish()
        time.sleep(0.5)



if __name__ == "__main__":

    head = Head()
    try:
        head.test()
    except KeyboardInterrupt:
        head.finish()
        print("\nProgram terminated. Goodbye!")
        print("Program developed by Team C - MasterCamp SE 2026.")