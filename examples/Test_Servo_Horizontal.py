import time
from board import SCL, SDA
import busio
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685

# PCA9685 cree UNE SEULE FOIS au niveau module et reutilise pour tous les
# appels set_angle — evite le tremblement cause par la recreation I2C/PCA
# a chaque mouvement (bug present dans RPIservo.py).
i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c, address=0x5f)
pca.frequency = 50


class ServoHorizontal:
    """Servo horizontal de la tete (canal PCA9685 : 1).

    Reprend la logique de Head_Horizontal.py et Head.py :
    - left / right : deplacement incremental avec bornes
    - center : retour doux vers 90 degres
    - destroy : retour centre + deinit propre du PCA9685
    """

    CENTER = 90
    MIN_ANGLE = 10
    MAX_ANGLE = 170
    STEP_DELAY = 0.01  # secondes entre chaque degre (meme valeur que Head.py)

    def __init__(self, channel=1):
        # L'objet servo.Servo est cree une fois et reutilise — pas recrée a
        # chaque mouvement, ce qui eliminait les tremblements.
        self.servo = servo.Servo(
            pca.channels[channel],
            min_pulse=500,
            max_pulse=2400,
            actuation_range=180,
        )
        self.servo.angle = self.CENTER

    def set_angle(self, angle):
        angle = max(self.MIN_ANGLE, min(self.MAX_ANGLE, int(angle)))
        self.servo.angle = angle

    def left(self, degrees):
        """Tourne a gauche de <degrees> degres (angle croissant)."""
        for _ in range(int(degrees)):
            if self.servo.angle + 1 >= self.MAX_ANGLE:
                break
            self.set_angle(self.servo.angle + 1)
            time.sleep(self.STEP_DELAY)

    def right(self, degrees):
        """Tourne a droite de <degrees> degres (angle decroissant)."""
        for _ in range(int(degrees)):
            if self.servo.angle - 1 <= self.MIN_ANGLE:
                break
            self.set_angle(self.servo.angle - 1)
            time.sleep(self.STEP_DELAY)

    def center(self):
        """Retour doux vers 90 degres, puis positionnement precis."""
        if self.servo.angle > self.CENTER:
            self.right(self.servo.angle - self.CENTER)
        else:
            self.left(self.CENTER - self.servo.angle)
        time.sleep(0.05)
        self.servo.angle = self.CENTER
        time.sleep(0.3)

    def destroy(self):
        """Retour centre + liberation propre du PCA9685 (GPIO freed)."""
        self.center()
        pca.deinit()


if __name__ == "__main__":
    head = ServoHorizontal(channel=1)

    print("=== Test servo horizontal ===")
    print("Rotation droite -> centre -> gauche -> centre (boucle)")
    print("Ctrl+C pour arreter proprement\n")

    try:
        while True:
            print("Droite")
            head.right(40)
            head.center()

            print("Gauche")
            head.left(40)
            head.center()

    except KeyboardInterrupt:
        print("\nArret demande — retour au centre et liberation GPIO...")
        head.destroy()
        print("Programme termine.")
