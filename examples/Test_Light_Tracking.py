import time
import smbus
import importlib.util
import os

# Chargement dynamique de Test_Servo_Horizontal pour reutiliser ServoHorizontal
# (meme PCA9685 persistant, pas de recreation a chaque appel)
_here = os.path.dirname(os.path.abspath(__file__))

def _load_module(name, filename):
    path = os.path.join(_here, filename)
    spec = importlib.util.spec_from_file_location(name, path)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m

_servo_mod = _load_module("test_servo_horizontal", "Test_Servo_Horizontal.py")
ServoHorizontal = _servo_mod.ServoHorizontal

# Seuils repris de functions.py (trackLightProcessing)
LIGHT_ADC_CENTER = 127   # valeur ADC de reference (lumiere neutre)
LIGHT_THRESHOLD  = 15    # ecart min pour declencher un mouvement

# Angles cibles du servo selon la direction de la lumiere
ANGLE_LEFT   = 130
ANGLE_CENTER = 90
ANGLE_RIGHT  = 50


class ADS7830:
    """Lecture ADC via I2C — repris de 09_lightTracking.py."""

    def __init__(self):
        self.cmd = 0x84
        self.bus = smbus.SMBus(1)
        self.address = 0x48

    def analogRead(self, chn):
        value = self.bus.read_byte_data(
            self.address,
            self.cmd | (((chn << 2 | chn >> 1) & 0x07) << 4),
        )
        return value


def get_direction(adc_value):
    if adc_value < LIGHT_ADC_CENTER - LIGHT_THRESHOLD:
        return "GAUCHE"
    elif adc_value > LIGHT_ADC_CENTER + LIGHT_THRESHOLD:
        return "DROITE"
    else:
        return "CENTER"


if __name__ == "__main__":
    adc  = ADS7830()
    head = ServoHorizontal(channel=1)

    print("=== Test Light Tracking (tete) ===")
    print(f"Reference ADC : {LIGHT_ADC_CENTER}  |  Seuil : ±{LIGHT_THRESHOLD}")
    print("La tete suit la source lumineuse — Ctrl+C pour arreter\n")

    last_direction = None

    try:
        while True:
            value     = adc.analogRead(1)
            direction = get_direction(value)

            if direction != last_direction:
                if direction == "GAUCHE":
                    head.set_angle(ANGLE_LEFT)
                    print(f"ADC={value:3d}  →  GAUCHE  (angle {ANGLE_LEFT}°)")
                elif direction == "DROITE":
                    head.set_angle(ANGLE_RIGHT)
                    print(f"ADC={value:3d}  →  DROITE  (angle {ANGLE_RIGHT}°)")
                else:
                    head.set_angle(ANGLE_CENTER)
                    print(f"ADC={value:3d}  →  CENTER  (angle {ANGLE_CENTER}°)")
                last_direction = direction
            else:
                print(f"ADC={value:3d}", end="\r")

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nArret — retour au centre et liberation GPIO...")
        head.destroy()
        print("Programme termine.")
