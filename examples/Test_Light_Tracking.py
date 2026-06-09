import time
import smbus
import importlib.util
import os

_here = os.path.dirname(os.path.abspath(__file__))


def _load_module(name, filename):
    path = os.path.join(_here, filename)
    spec = importlib.util.spec_from_file_location(name, path)
    m    = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


_servo_mod          = _load_module("test_servo_horizontal", "Test_Servo_Horizontal.py")
ServoHorizontal     = _servo_mod.ServoHorizontal

_front_mod          = _load_module("test_leds_t1", "Test_LEDs_T1.py")

_spi_mod            = _load_module("spi_ws2812", "06_Spi_WS2812.py")
Adeept_SPI_LedPixel = _spi_mod.Adeept_SPI_LedPixel

# ---------------------------------------------------------------------------
# Parametres
# ---------------------------------------------------------------------------
LIGHT_ADC_CENTER = 127   # valeur ADC de reference (lumiere neutre)
LIGHT_THRESHOLD  = 15    # ecart min pour declencher un mouvement

ANGLE_LEFT   = 130
ANGLE_CENTER = 90
ANGLE_RIGHT  = 50

ORANGE_GREEN = 0.45           # niveau vert PWM pour simuler orange (LED RGB)
ORANGE_RGB   = [255, 128, 0]  # orange WS2812
WS_LEFT      = [2, 3, 4, 11, 12, 13]
WS_RIGHT     = [5, 6, 7, 8, 9, 10]
WS_COUNT     = 14

BLINK_HALF   = 5   # 5 iterations x 0.1 s = 0.5 s par demi-periode → cligno 1 Hz


class ADS7830:
    """Lecture ADC via I2C — repris de 09_lightTracking.py."""

    def __init__(self):
        self.cmd     = 0x84
        self.bus     = smbus.SMBus(1)
        self.address = 0x48

    def analogRead(self, chn):
        return self.bus.read_byte_data(
            self.address,
            self.cmd | (((chn << 2 | chn >> 1) & 0x07) << 4),
        )


def get_direction(adc_value):
    if adc_value < LIGHT_ADC_CENTER - LIGHT_THRESHOLD:
        return "GAUCHE"
    elif adc_value > LIGHT_ADC_CENTER + LIGHT_THRESHOLD:
        return "DROITE"
    else:
        return "CENTER"


def update_blinkers(side, tick, front, strip):
    """Fait clignoter a 1 Hz les LED du cote du virage (avant + arriere)."""
    on    = (tick % (2 * BLINK_HALF)) < BLINK_HALF
    l_on  = (side == "GAUCHE") and on
    r_on  = (side == "DROITE") and on

    front.leds[4].value = 1.0          if l_on else 0.0
    front.leds[5].value = ORANGE_GREEN if l_on else 0.0
    front.leds[7].value = 1.0          if r_on else 0.0
    front.leds[8].value = ORANGE_GREEN if r_on else 0.0

    for i in WS_LEFT:
        strip.set_led_rgb_data(i, ORANGE_RGB if l_on else [0, 0, 0])
    for i in WS_RIGHT:
        strip.set_led_rgb_data(i, ORANGE_RGB if r_on else [0, 0, 0])
    strip.show()


def stop_blinkers(front, strip):
    for num in (4, 5, 7, 8):
        front.leds[num].value = 0.0
    for i in WS_LEFT + WS_RIGHT:
        strip.set_led_rgb_data(i, [0, 0, 0])
    strip.show()


if __name__ == "__main__":
    _front_mod.setup()

    adc   = ADS7830()
    head  = ServoHorizontal(channel=1)
    strip = Adeept_SPI_LedPixel(WS_COUNT, 255)

    if strip.check_spi_state() == 0:
        print("Attention : SPI indisponible, ruban WS2812 desactive")
    strip.set_all_led_color(0, 0, 0)

    print("=== Test Light Tracking (tete + clignotants) ===")
    print(f"Reference ADC : {LIGHT_ADC_CENTER}  |  Seuil : ±{LIGHT_THRESHOLD}")
    print("La tete et les clignotants suivent la lumiere — Ctrl+C pour arreter\n")

    last_direction = None
    tick           = 0

    try:
        while True:
            # --- lecture ADC avec gestion erreur I2C ---
            try:
                value = adc.analogRead(1)
            except OSError as e:
                print(f"\n[I2C] ADS7830 non accessible (adresse 0x48) : {e}")
                print("Verifiez le branchement du capteur de lumiere (I2C bus 1).")
                time.sleep(0.5)
                continue

            direction = get_direction(value)

            # --- mouvement tete ---
            if direction != last_direction:
                if direction == "GAUCHE":
                    head.set_angle(ANGLE_LEFT)
                    print(f"\nADC={value:3d}  →  GAUCHE  (angle {ANGLE_LEFT}°)")
                elif direction == "DROITE":
                    head.set_angle(ANGLE_RIGHT)
                    print(f"\nADC={value:3d}  →  DROITE  (angle {ANGLE_RIGHT}°)")
                else:
                    head.set_angle(ANGLE_CENTER)
                    print(f"\nADC={value:3d}  →  CENTER  (angle {ANGLE_CENTER}°)")
                last_direction = direction

            print(f"ADC={value:3d}", end="\r")

            # --- clignotants directionnels (avant + arriere) ---
            update_blinkers(direction, tick, _front_mod, strip)
            tick += 1

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nArret — retour au centre et liberation GPIO...")
        stop_blinkers(_front_mod, strip)
        head.destroy()
        strip.led_close()
        print("Programme termine.")
