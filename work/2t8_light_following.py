import time
import smbus
from collections import namedtuple

# Resultat de lecture : valeurs des deux capteurs + direction deduite
LightReading = namedtuple("LightReading", ["left", "right", "direction"])

# 5 angles de braquage possibles (repris de t3_servomotors.py)
WHEEL_ANGLE_CENTER = 90
STEER_SOFT_DEG     = 15
STEER_HARD_DEG     = 35

STEER_HARD_LEFT  = WHEEL_ANGLE_CENTER - STEER_HARD_DEG   # 55
STEER_SOFT_LEFT  = WHEEL_ANGLE_CENTER - STEER_SOFT_DEG   # 75
STEER_CENTER     = WHEEL_ANGLE_CENTER                    # 90
STEER_SOFT_RIGHT = WHEEL_ANGLE_CENTER + STEER_SOFT_DEG   # 105
STEER_HARD_RIGHT = WHEEL_ANGLE_CENTER + STEER_HARD_DEG   # 125


class ADS7830:
    # ADC I2C, 8 canaux (0-7), valeurs 0-255

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
    def __init__(self, ch_left: int = 0, ch_right: int = 1, threshold: int = 15):
        self.adc       = ADS7830()
        self.ch_left   = ch_left
        self.ch_right  = ch_right
        self.threshold = threshold

    def read(self) -> LightReading:
        # lit les deux capteurs et determine de quel cote vient la lumiere
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

    def get_steer_angle(self, reading: LightReading = None,
                         soft_threshold: int = 15,
                         hard_threshold: int = 50) -> float:
        # convertit l'ecart de lumiere en un des 5 angles de braquage
        if reading is None:
            reading = self.read()

        diff = reading.left - reading.right

        if diff > hard_threshold:
            return STEER_HARD_LEFT
        elif diff > soft_threshold:
            return STEER_SOFT_LEFT
        elif diff < -hard_threshold:
            return STEER_HARD_RIGHT
        elif diff < -soft_threshold:
            return STEER_SOFT_RIGHT
        else:
            return STEER_CENTER


if __name__ == "__main__":
    try:
        from t3_servomotors import Head, SERVO_PCA
        _HAS_SERVO = True
    except ImportError:
        _HAS_SERVO = False

    sensor = LightFollowingModule(ch_left=0, ch_right=1)

    print("=== Suivi de lumiere -> braquage roue (Ctrl+C pour arreter) ===")
    print(f"{'Gauche':>7}  {'Droite':>7}  {'Ecart':>6}  Angle roue")
    print("-" * 38)

    head = Head(SERVO_PCA) if _HAS_SERVO else None
    if not _HAS_SERVO:
        print("(t3_servomotors introuvable — affichage seul, pas de servo)\n")

    try:
        while True:
            try:
                r     = sensor.read()
                angle = sensor.get_steer_angle(r)
                print(f"{r.left:>7}  {r.right:>7}  {r.left - r.right:>6}  {angle:>5.0f}°", end="\r")

                if head is not None:
                    head.wheel.set_angle(angle)

            except OSError as e:
                print(f"\n[I2C] ADS7830 inaccessible (0x48) : {e}")
                time.sleep(0.5)

            time.sleep(0.2)

    except KeyboardInterrupt:
        print("\n\nArret — recentrage de la roue...")
        if head is not None:
            head.wheel.center()
        print("Programme developpe par l'Equipe C - MasterCamp SE 2026.")
