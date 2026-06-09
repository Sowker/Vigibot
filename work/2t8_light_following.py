import time
import smbus
from collections import namedtuple

LightReading = namedtuple("LightReading", ["left", "right", "direction"])
"""
left      : int 0-255, valeur ADC capteur gauche (canal ch_left)
right     : int 0-255, valeur ADC capteur droit  (canal ch_right)
direction : "LEFT" | "RIGHT" | "CENTER"
"""


class ADS7830:
    """
    Convertisseur Analogique-Numerique ADS7830 via I2C.
    8 canaux disponibles (0-7), resolution 8 bits (0-255).
    Adresse I2C par defaut : 0x48.
    """

    def __init__(self, address: int = 0x48):
        self.cmd     = 0x84
        self.bus     = smbus.SMBus(1)
        self.address = address

    def analogRead(self, chn: int) -> int:
        """Lit la valeur ADC du canal chn (0-7). Retourne 0-255."""
        return self.bus.read_byte_data(
            self.address,
            self.cmd | (((chn << 2 | chn >> 1) & 0x07) << 4),
        )


class LightFollowingModule:
    """
    Module de suivi de lumiere a deux capteurs (gauche + droite).

    Utilisation depuis un autre fichier :
        from work.t8_light_following import LightFollowingModule
        sensor = LightFollowingModule()
        reading = sensor.read()          # LightReading(left, right, direction)
        print(reading.left, reading.right, reading.direction)

    Parametres
    ----------
    ch_left    : canal ADC du capteur gauche  (defaut 0)
    ch_right   : canal ADC du capteur droit   (defaut 1)
    threshold  : ecart minimal entre les deux valeurs pour decider
                 LEFT ou RIGHT (evite les oscillations sur seuil)  (defaut 10)
    """

    def __init__(self, ch_left: int = 0, ch_right: int = 1, threshold: int = 10):
        self.adc       = ADS7830()
        self.ch_left   = ch_left
        self.ch_right  = ch_right
        self.threshold = threshold

    def read(self) -> LightReading:
        """
        Lit les deux capteurs et retourne un LightReading.
        Peut lever OSError si le module I2C n'est pas connecte.
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
    sensor = LightFollowingModule(ch_left=0, ch_right=1, threshold=10)
    print("Suivi de lumiere — deux capteurs (Ctrl+C pour arreter)\n")
    print(f"{'Canal gauche':>14}  {'Canal droit':>11}  Direction")
    print("-" * 42)

    try:
        while True:
            try:
                r = sensor.read()
                print(f"Gauche : {r.left:>3}    Droite : {r.right:>3}    {r.direction:<6}", end="\r")
            except OSError as e:
                print(f"\n[I2C] ADS7830 inaccessible (0x48) : {e}")
                print("Verifiez le branchement du capteur sur le bus I2C.")
                time.sleep(0.5)
            time.sleep(0.2)

    except KeyboardInterrupt:
        print("\n\nProgramme interrompu.")
        print("Programme developpe par l'Equipe C - MasterCamp SE 2026.")
