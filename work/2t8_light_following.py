import time
import smbus
from collections import namedtuple

LightReading = namedtuple("LightReading", ["left", "right", "direction"])
"""
left      : int 0-255  valeur ADC capteur gauche
right     : int 0-255  valeur ADC capteur droit
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
    Lecture des deux capteurs de lumiere (gauche + droite) sur deux canaux ADC.

    Utilisation depuis un autre fichier :
        from work.t8_light_following import LightFollowingModule
        sensor = LightFollowingModule(ch_left=0, ch_right=1)
        r = sensor.read()
        print(r.left, r.right, r.direction)

    Parametres
    ----------
    ch_left   : canal ADC du capteur gauche  (defaut 0)
    ch_right  : canal ADC du capteur droit   (defaut 1)
    threshold : ecart minimal pour decider LEFT/RIGHT  (defaut 15)
    """

    def __init__(self, ch_left: int = 0, ch_right: int = 1, threshold: int = 15):
        self.adc       = ADS7830()
        self.ch_left   = ch_left
        self.ch_right  = ch_right
        self.threshold = threshold

    def read(self) -> LightReading:
        """
        Lit les deux capteurs et retourne un LightReading.
        Peut lever OSError si l'ADS7830 n'est pas accessible en I2C.
        """
        left  = self.adc.analogRead(self.ch_left)
        right = self.adc.analogRead(self.ch_right)

        diff = left - right
        if diff > self.threshold:
            direction = "LEFT"
        elif diff < -self.threshold:
            direction = "RIGHT"
        else:
            direction = "CENTER"

        return LightReading(left=left, right=right, direction=direction)


if __name__ == "__main__":
    # Scan de tous les canaux pour identifier lesquels sont des capteurs de lumiere
    adc = ADS7830()
    print("Scan de tous les canaux — bougez la lumiere pour voir lesquels reagissent")
    print("Canaux non branches = valeur fixe. Canaux actifs = valeur variable.\n")
    print("  " + "  ".join(f"CH{i}" for i in range(8)))
    print("-" * 44)

    try:
        while True:
            try:
                vals = [adc.analogRead(i) for i in range(8)]
                print("  " + "  ".join(f"{v:>3}" for v in vals), end="\r")
            except OSError as e:
                print(f"\n[I2C] ADS7830 inaccessible (0x48) : {e}")
                time.sleep(0.5)
            time.sleep(0.2)

    except KeyboardInterrupt:
        print("\n\nIdentifiez les canaux actifs, puis utilisez :")
        print("  sensor = LightFollowingModule(ch_left=X, ch_right=Y)")
        print("Programme developpe par l'Equipe C - MasterCamp SE 2026.")
