import time
import smbus
from collections import namedtuple

LightReading = namedtuple("LightReading", ["left", "right", "direction"])
"""
left      : int 0-255  valeur ADC capteur quand la tete pointe a gauche
right     : int 0-255  valeur ADC capteur quand la tete pointe a droite
direction : "LEFT" | "RIGHT" | "CENTER"
"""


class ADS7830:
    """ADC ADS7830 via I2C — 8 canaux, resolution 8 bits (0-255)."""

    def __init__(self, address: int = 0x48):
        self.cmd     = 0x84
        self.bus     = smbus.SMBus(1)
        self.address = address

    def analogRead(self, chn: int) -> int:
        return self.bus.read_byte_data(
            self.address,
            self.cmd | (((chn << 2 | chn >> 1) & 0x07) << 4),
        )


class LightFollowingModule:
    """
    Suivi de lumiere a capteur unique (canal ADC) monte sur un servo.

    Principe : on tourne la tete a gauche, on lit la lumiere, on tourne a
    droite, on relit. Les deux lectures donnent "exposition gauche" et
    "exposition droite". La direction est la plus lumineuse des deux.

    Utilisation depuis un autre fichier :
        from work.t8_light_following import LightFollowingModule
        sensor = LightFollowingModule(channel=1, threshold=15)
        reading = sensor.scan(set_angle_fn=head.set_angle)
        print(reading.left, reading.right, reading.direction)

    Parametres
    ----------
    channel    : canal ADC du capteur (defaut 1)
    threshold  : ecart minimal entre left et right pour decider LEFT/RIGHT
    left_angle : angle servo pour pointer a gauche  (defaut 130)
    right_angle: angle servo pour pointer a droite  (defaut 50)
    settle     : secondes a attendre apres avoir tourne (defaut 0.08)
    """

    def __init__(
        self,
        channel: int     = 1,
        threshold: int   = 15,
        left_angle: int  = 130,
        right_angle: int = 50,
        settle: float    = 0.08,
    ):
        self.adc         = ADS7830()
        self.channel     = channel
        self.threshold   = threshold
        self.left_angle  = left_angle
        self.right_angle = right_angle
        self.settle      = settle

    def scan(self, set_angle_fn) -> LightReading:
        """
        Oriente la tete gauche puis droite, lit le capteur a chaque position.

        set_angle_fn : callable(angle: int) — ex: head.set_angle
        Retourne LightReading(left, right, direction).
        Peut lever OSError si l'ADS7830 n'est pas accessible en I2C.
        """
        set_angle_fn(self.left_angle)
        time.sleep(self.settle)
        left = self.adc.analogRead(self.channel)

        set_angle_fn(self.right_angle)
        time.sleep(self.settle)
        right = self.adc.analogRead(self.channel)

        diff = left - right
        if diff > self.threshold:
            direction = "LEFT"
        elif diff < -self.threshold:
            direction = "RIGHT"
        else:
            direction = "CENTER"

        return LightReading(left=left, right=right, direction=direction)


# ---------------------------------------------------------------------------
# Test autonome — cree son propre PCA9685 uniquement ici
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    from board import SCL, SDA
    import busio
    from adafruit_pca9685 import PCA9685
    from adafruit_motor import servo as adafruit_servo

    i2c = busio.I2C(SCL, SDA)
    pca = PCA9685(i2c, address=0x5f)
    pca.frequency = 50
    head_servo = adafruit_servo.Servo(
        pca.channels[1], min_pulse=500, max_pulse=2400, actuation_range=180
    )
    head_servo.angle = 90

    sensor = LightFollowingModule(channel=1, threshold=15)

    print("Scan de lumiere gauche/droite (Ctrl+C pour arreter)\n")
    print(f"{'Gauche':>8}  {'Droite':>8}  Direction")
    print("-" * 32)

    try:
        while True:
            try:
                r = sensor.scan(set_angle_fn=lambda a: setattr(head_servo, "angle", a))
                head_servo.angle = 90   # retour centre apres chaque scan
                print(f"{r.left:>8}  {r.right:>8}  {r.direction:<6}", end="\r")
                time.sleep(0.3)
            except OSError as e:
                print(f"\n[I2C] ADS7830 inaccessible (0x48) : {e}")
                time.sleep(0.5)

    except KeyboardInterrupt:
        print("\n\nArret — retour au centre.")
        head_servo.angle = 90
        time.sleep(0.3)
        pca.deinit()
        print("Programme developpe par l'Equipe C - MasterCamp SE 2026.")
