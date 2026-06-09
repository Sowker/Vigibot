import time
import importlib.util
import os
import sys
from gpiozero import TonalBuzzer

# ---------------------------------------------------------------------------
# Chargement dynamique des modules
# ---------------------------------------------------------------------------
_here    = os.path.dirname(os.path.abspath(__file__))
_web_dir = os.path.abspath(os.path.join(_here, "..", "web"))


def _load_module(name, filepath):
    spec = importlib.util.spec_from_file_location(name, filepath)
    m    = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


_lt_mod            = _load_module("line_tracking_08",
                                  os.path.join(_here, "08_lineTracking.py"))
LineTrackingModule = _lt_mod.LineTrackingModule
Color              = _lt_mod.Color

_servo_mod         = _load_module("test_servo_horizontal",
                                  os.path.join(_here, "Test_Servo_Horizontal.py"))
ServoHorizontal    = _servo_mod.ServoHorizontal

_front_mod         = _load_module("test_leds_t1",
                                  os.path.join(_here, "Test_LEDs_T1.py"))

_spi_mod           = _load_module("spi_ws2812",
                                  os.path.join(_here, "06_Spi_WS2812.py"))
Adeept_SPI_LedPixel = _spi_mod.Adeept_SPI_LedPixel

sys.path.insert(0, _web_dir)
import move

# ---------------------------------------------------------------------------
# Parametres
# ---------------------------------------------------------------------------
DRIVE_SPEED  = 28    # vitesse moteurs 0-100
TURN_INNER   = 8     # vitesse de la roue interieure en virage (effet differentiel)

HEAD_LEFT    = 130
HEAD_CENTER  = 90
HEAD_RIGHT   = 50

ORANGE_GREEN = 0.45          # niveau vert pour simuler orange (LED RGB)
ORANGE_RGB   = [255, 128, 0] # orange pour le ruban WS2812
WS_COUNT     = 14            # nombre de LED sur le ruban

BEEP_FREQ    = "A4"          # frequence des bips (note gpiozero)
BEEP_ON      = 0.09          # duree bip actif (s)
BEEP_OFF     = 0.07          # silence entre bips (s)
WARN_PAUSE   = 1.3           # pause entre deux cycles d'alerte (s)

# ---------------------------------------------------------------------------
# Commande moteurs (differentiel — reprend les canaux de move.py)
# ---------------------------------------------------------------------------

def drive_straight(speed=DRIVE_SPEED):
    move.Motor(1,  move.M1_Direction, speed)
    move.Motor(2,  move.M2_Direction, speed)


def drive_left(speed=DRIVE_SPEED):
    """Roue gauche ralentie, roue droite a vitesse normale."""
    move.Motor(1,  move.M1_Direction, TURN_INNER)
    move.Motor(2,  move.M2_Direction, speed)


def drive_right(speed=DRIVE_SPEED):
    """Roue droite ralentie, roue gauche a vitesse normale."""
    move.Motor(1,  move.M1_Direction, speed)
    move.Motor(2,  move.M2_Direction, TURN_INNER)


# ---------------------------------------------------------------------------
# Alerte orange (LEDs avant RGB + ruban WS2812 arriere)
# ---------------------------------------------------------------------------

def _set_all_orange(front, strip, on):
    """Allume ou eteint toutes les LED orange (avant + arriere)."""
    if on:
        front.leds[4].value = 1.0
        front.leds[5].value = ORANGE_GREEN
        front.leds[7].value = 1.0
        front.leds[8].value = ORANGE_GREEN
    else:
        for num in (4, 5, 7, 8):
            front.leds[num].value = 0.0

    color = ORANGE_RGB if on else [0, 0, 0]
    for i in range(WS_COUNT):
        strip.set_led_rgb_data(i, color)
    strip.show()


def warning_cycle(front, strip, buzzer):
    """Un cycle d'alerte : 2 bips rapides + flash orange, puis pause.

    Reproduit le comportement des vehicules encombrants / pannes sur
    autoroute : deux eclairs orange brefs suivis d'une pause (~1,5 s).
    """
    for _ in range(2):
        _set_all_orange(front, strip, True)
        buzzer.play(BEEP_FREQ)
        time.sleep(BEEP_ON)
        _set_all_orange(front, strip, False)
        buzzer.stop()
        time.sleep(BEEP_OFF)

    time.sleep(WARN_PAUSE)


# ---------------------------------------------------------------------------
# Logique de suivi de ligne
# ---------------------------------------------------------------------------

def process_line(left, middle, right, head):
    """Applique la direction roues + tete selon les 3 capteurs.

    Convention (08_lineTracking.py) : 0 = noir detecte, 1 = sol blanc.
    Retourne (action_str, state) ou state est 'tracking' ou 'lost'.
    """
    # --- ligne au centre ---
    if middle == 0 and left == 1 and right == 1:
        head.set_angle(HEAD_CENTER)
        drive_straight()
        return f"{Color.ACTION_GO}Tout droit{Color.END}", "tracking"

    # --- derive a droite : capteur gauche capte le noir en premier ---
    if left == 0 and middle == 0 and right == 1:
        head.set_angle(HEAD_LEFT)
        drive_left()
        return f"{Color.ACTION_TURN}Gauche (leger) ←{Color.END}", "tracking"

    if left == 0 and middle == 1 and right == 1:
        head.set_angle(HEAD_LEFT)
        drive_left()
        return f"{Color.ACTION_TURN}Gauche ←{Color.END}", "tracking"

    # --- derive a gauche : capteur droit capte le noir ---
    if left == 1 and middle == 0 and right == 0:
        head.set_angle(HEAD_RIGHT)
        drive_right()
        return f"{Color.ACTION_TURN}Droite (leger) →{Color.END}", "tracking"

    if left == 1 and middle == 1 and right == 0:
        head.set_angle(HEAD_RIGHT)
        drive_right()
        return f"{Color.ACTION_TURN}Droite →{Color.END}", "tracking"

    # --- intersection (tous sur noir) : continuer tout droit ---
    if left == 0 and middle == 0 and right == 0:
        head.set_angle(HEAD_CENTER)
        drive_straight()
        return f"{Color.ACTION_GO}Intersection{Color.END}", "tracking"

    # --- ligne perdue (tous sur blanc) ---
    head.set_angle(HEAD_CENTER)
    move.motorStop()
    return f"{Color.ACTION_WARN}Ligne perdue !{Color.END}", "lost"


# ---------------------------------------------------------------------------
# Point d'entree
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    move.setup()
    _front_mod.setup()

    sensors = LineTrackingModule(pin_left=22, pin_middle=27, pin_right=17)
    head    = ServoHorizontal(channel=1)
    strip   = Adeept_SPI_LedPixel(WS_COUNT, 255)
    buzzer  = TonalBuzzer(18)

    if strip.check_spi_state() == 0:
        print("Attention : SPI indisponible, ruban WS2812 desactive")
    strip.set_all_led_color(0, 0, 0)

    print("=== Test Line Tracking (roues + tete) ===")
    print("0 = ligne noire  |  1 = sol blanc")
    print("Ctrl+C pour arreter\n")

    last_action = None

    try:
        while True:
            left, middle, right = sensors.read()
            action, state       = process_line(left, middle, right, head)
            visual              = sensors.get_visual_bar()
            raw                 = f"(L:{left} M:{middle} R:{right})"

            if action != last_action:
                print(f"{visual} {raw:<14} -> {action}")
                last_action = action

            if state == "lost":
                warning_cycle(_front_mod, strip, buzzer)
            else:
                # Affichage live en mode suivi
                print(f"{visual} {raw}", end="\r")
                time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nArret — retour au centre et liberation GPIO...")
        move.motorStop()
        _set_all_orange(_front_mod, strip, False)
        head.center()
        move.destroy()
        strip.led_close()
        print("Programme termine.")
