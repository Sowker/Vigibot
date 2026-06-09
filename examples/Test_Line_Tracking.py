import time
import importlib.util
import os
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo as adafruit_servo, motor as adafruit_motor
from gpiozero import InputDevice, PWMOutputDevice

# --------------------------------------------------------------------------
# PCA9685 unique — partage servo tete ET moteurs DC sur le meme chip 0x5f.
# Deux instances Python sur la meme adresse provoquent des conflits de
# frequence et font trembler le servo ; ici il n'y en a qu'une.
# --------------------------------------------------------------------------
i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c, address=0x5f)
pca.frequency = 50

# --- Servo tete horizontale (canal 1, repris de Head.py) ---
HEAD_CHANNEL   = 1
HEAD_MIN_PULSE = 500
HEAD_MAX_PULSE = 2400
HEAD_LEFT       = 130
HEAD_CENTER     = 90
HEAD_RIGHT      = 50
HEAD_STEP_DELAY = 0.01   # secondes entre chaque degre pour le retour centre
# Marge anti-tremblement : la tete ne bouge que si le meme angle est demande
# HEAD_CONFIRM fois de suite. Augmenter si la tete tremble encore.
HEAD_CONFIRM    = 2
_head_pending   = [HEAD_CENTER, 0]   # [angle_demande, compteur_consecutif]

head_servo = adafruit_servo.Servo(
    pca.channels[HEAD_CHANNEL],
    min_pulse=HEAD_MIN_PULSE,
    max_pulse=HEAD_MAX_PULSE,
    actuation_range=180,
)
head_servo.angle = HEAD_CENTER

# --- Moteurs DC (canaux repris de move.py) ---
# M1 gauche / M2 droit (M2 monte en sens inverse -> throttle negatif = avant)
DRIVE           = 0.30   # vitesse tout droit (les deux cotes egaux)
TURN_IN         = 0.08   # roue interieure en virage
# Roue exterieure par sens de virage — ajuster si un cote tourne plus vite :
#   droite trop rapide -> baisser TURN_OUT_RIGHT
#   gauche trop rapide -> baisser TURN_OUT_LEFT
TURN_OUT_RIGHT  = 0.26   # roue gauche (exterieure) lors d'un virage droite
TURN_OUT_LEFT   = 0.30   # roue droite (exterieure) lors d'un virage gauche

motor1 = adafruit_motor.DCMotor(pca.channels[15], pca.channels[14])
motor1.decay_mode = adafruit_motor.SLOW_DECAY
motor2 = adafruit_motor.DCMotor(pca.channels[12], pca.channels[13])
motor2.decay_mode = adafruit_motor.SLOW_DECAY
motor3 = adafruit_motor.DCMotor(pca.channels[11], pca.channels[10])
motor3.decay_mode = adafruit_motor.SLOW_DECAY
motor4 = adafruit_motor.DCMotor(pca.channels[8],  pca.channels[9])
motor4.decay_mode = adafruit_motor.SLOW_DECAY

# --------------------------------------------------------------------------
# Chargement du module de capteurs de ligne (08_lineTracking.py)
# --------------------------------------------------------------------------
_here = os.path.dirname(os.path.abspath(__file__))


def _load_module(name, filepath):
    spec = importlib.util.spec_from_file_location(name, filepath)
    m    = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


_lt_mod            = _load_module("line_tracking_08",
                                  os.path.join(_here, "08_lineTracking.py"))
LineTrackingModule = _lt_mod.LineTrackingModule
Color              = _lt_mod.Color

# --- WS2812 (ruban LED arriere) ---
_spi_mod            = _load_module("spi_ws2812",
                                   os.path.join(_here, "06_Spi_WS2812.py"))
Adeept_SPI_LedPixel = _spi_mod.Adeept_SPI_LedPixel

# --- LED RGB avant (GPIO, sans conflit PCA9685) ---
ORANGE_GREEN = 0.45
ORANGE_RGB   = [255, 128, 0]
WS_LEFT      = [2, 3, 4, 11, 12, 13]
WS_RIGHT     = [5, 6, 7, 8, 9, 10]
WS_COUNT     = 14
BLINK_HALF   = 5   # iterations (x0.1 s) par demi-periode -> 1 Hz

led_left_r  = PWMOutputDevice(13, active_high=False)
led_left_g  = PWMOutputDevice(19, active_high=False)
led_right_r = PWMOutputDevice(1,  active_high=False)
led_right_g = PWMOutputDevice(5,  active_high=False)

# ============================================================
# BIPS (decommenter quand la section bips sera activee)
# ============================================================
# from gpiozero import TonalBuzzer
# BEEP_FREQ  = "A4"
# BEEP_ON    = 0.09
# BEEP_OFF   = 0.07
# WARN_PAUSE = 1.3
# buzzer = TonalBuzzer(18)
# ============================================================


# --------------------------------------------------------------------------
# Mouvement servo tete
# --------------------------------------------------------------------------

def head_set(angle):
    """Bouge la tete seulement si le meme angle est demande HEAD_CONFIRM fois de suite."""
    global _head_pending
    angle = max(10, min(170, int(angle)))
    if _head_pending[0] == angle:
        _head_pending[1] += 1
    else:
        _head_pending = [angle, 1]
    if _head_pending[1] >= HEAD_CONFIRM:
        head_servo.angle = angle


def head_center():
    """Retour doux vers 90 degres (meme logique que Head.py finish())."""
    current = head_servo.angle or HEAD_CENTER
    if current > HEAD_CENTER:
        for a in range(int(current), HEAD_CENTER, -1):
            head_servo.angle = a
            time.sleep(HEAD_STEP_DELAY)
    else:
        for a in range(int(current), HEAD_CENTER):
            head_servo.angle = a
            time.sleep(HEAD_STEP_DELAY)
    head_servo.angle = HEAD_CENTER


# --------------------------------------------------------------------------
# Mouvement moteurs (differentiel M1 gauche / M2 droit)
# --------------------------------------------------------------------------

def motors_stop():
    motor1.throttle = 0
    motor2.throttle = 0
    motor3.throttle = 0
    motor4.throttle = 0


def drive_straight():
    motor1.throttle =  DRIVE    # M1 gauche avant
    motor2.throttle = -DRIVE    # M2 droit  avant (monte en sens inverse)
    motor3.throttle =  DRIVE
    motor4.throttle = -DRIVE


def drive_left():
    motor1.throttle =  TURN_IN
    motor2.throttle = -TURN_OUT_LEFT
    motor3.throttle =  TURN_IN
    motor4.throttle = -TURN_OUT_LEFT


def drive_right():
    motor1.throttle =  TURN_OUT_RIGHT
    motor2.throttle = -TURN_IN
    motor3.throttle =  TURN_OUT_RIGHT
    motor4.throttle = -TURN_IN


# --------------------------------------------------------------------------
# Clignotants directionnels (avant GPIO + arriere WS2812)
# --------------------------------------------------------------------------

def update_blinkers(side, tick, strip):
    on   = (tick % (2 * BLINK_HALF)) < BLINK_HALF
    l_on = (side == "left")  and on
    r_on = (side == "right") and on

    led_left_r.value  = 1.0          if l_on else 0.0
    led_left_g.value  = ORANGE_GREEN if l_on else 0.0
    led_right_r.value = 1.0          if r_on else 0.0
    led_right_g.value = ORANGE_GREEN if r_on else 0.0

    for i in WS_LEFT:
        strip.set_led_rgb_data(i, ORANGE_RGB if l_on else [0, 0, 0])
    for i in WS_RIGHT:
        strip.set_led_rgb_data(i, ORANGE_RGB if r_on else [0, 0, 0])
    strip.show()


def stop_blinkers(strip):
    led_left_r.value  = 0.0
    led_left_g.value  = 0.0
    led_right_r.value = 0.0
    led_right_g.value = 0.0
    for i in WS_LEFT + WS_RIGHT:
        strip.set_led_rgb_data(i, [0, 0, 0])
    strip.show()


# --------------------------------------------------------------------------
# Alerte ligne perdue
# --------------------------------------------------------------------------

def _all_orange(strip, on):
    color = ORANGE_RGB if on else [0, 0, 0]
    led_left_r.value  = 1.0          if on else 0.0
    led_left_g.value  = ORANGE_GREEN if on else 0.0
    led_right_r.value = 1.0          if on else 0.0
    led_right_g.value = ORANGE_GREEN if on else 0.0
    for i in range(WS_COUNT):
        strip.set_led_rgb_data(i, color)
    strip.show()


def warning_cycle(strip):
    """Flash orange sur toutes les LED x2.

    ============================================================
    BIPS : decommenter le bloc ci-dessous quand c'est pret
    ============================================================
    for _ in range(2):
        _all_orange(strip, True)
        buzzer.play(BEEP_FREQ)
        time.sleep(BEEP_ON)
        _all_orange(strip, False)
        buzzer.stop()
        time.sleep(BEEP_OFF)
    time.sleep(WARN_PAUSE)
    ============================================================
    """
    # --- version sans bips (active pour l'instant) ---
    for _ in range(2):
        _all_orange(strip, True)
        time.sleep(0.09)
        _all_orange(strip, False)
        time.sleep(0.07)
    time.sleep(1.3)


# --------------------------------------------------------------------------
# Logique de suivi de ligne
# --------------------------------------------------------------------------

def process_line(left, middle, right):
    """Retourne (action_str, state, blink_side).
    Convention : 0 = noir detecte, 1 = sol blanc.
    """
    # ligne centree
    if middle == 0 and left == 1 and right == 1:
        head_set(HEAD_CENTER)
        drive_straight()
        return f"{Color.ACTION_GO}Tout droit{Color.END}", "tracking", None

    # capteur gauche (+ milieu) sur noir -> virer a gauche
    if left == 0 and middle == 0 and right == 1:
        head_set(HEAD_LEFT)
        drive_left()
        return f"{Color.ACTION_TURN}Gauche leger ←{Color.END}", "tracking", "left"

    if left == 0 and middle == 1 and right == 1:
        head_set(HEAD_LEFT)
        drive_left()
        return f"{Color.ACTION_TURN}Gauche ←{Color.END}", "tracking", "left"

    # capteur droit (+ milieu) sur noir -> virer a droite
    if left == 1 and middle == 0 and right == 0:
        head_set(HEAD_RIGHT)
        drive_right()
        return f"{Color.ACTION_TURN}Droite leger →{Color.END}", "tracking", "right"

    if left == 1 and middle == 1 and right == 0:
        head_set(HEAD_RIGHT)
        drive_right()
        return f"{Color.ACTION_TURN}Droite →{Color.END}", "tracking", "right"

    # intersection (tous sur noir) -> continuer tout droit
    if left == 0 and middle == 0 and right == 0:
        head_set(HEAD_CENTER)
        drive_straight()
        return f"{Color.ACTION_GO}Intersection{Color.END}", "tracking", None

    # ligne perdue (tous sur blanc)
    head_set(HEAD_CENTER)
    motors_stop()
    return f"{Color.ACTION_WARN}Ligne perdue !{Color.END}", "lost", None


# --------------------------------------------------------------------------
# Point d'entree
# --------------------------------------------------------------------------

if __name__ == "__main__":
    sensors = LineTrackingModule(pin_left=22, pin_middle=27, pin_right=17)
    strip   = Adeept_SPI_LedPixel(WS_COUNT, 255)

    if strip.check_spi_state() == 0:
        print("Attention : SPI indisponible, ruban WS2812 desactive")
    strip.set_all_led_color(0, 0, 0)

    print("=== Test Line Tracking (roues + tete + clignotants) ===")
    print("0 = ligne noire  |  1 = sol blanc")
    print("Ctrl+C pour arreter\n")

    last_action = None
    tick        = 0

    try:
        while True:
            left, middle, right       = sensors.read()
            action, state, blink_side = process_line(left, middle, right)
            visual                    = sensors.get_visual_bar()
            raw                       = f"(L:{left} M:{middle} R:{right})"

            if action != last_action:
                print(f"\n{visual} {raw:<14} -> {action}")
                last_action = action

            if state == "lost":
                stop_blinkers(strip)
                warning_cycle(strip)
                tick = 0
            else:
                update_blinkers(blink_side, tick, strip)
                print(f"{visual} {raw}", end="\r")
                tick += 1
                time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nArret — retour au centre et liberation GPIO...")

    finally:
        # Arret propre garanti meme en cas d'erreur
        motors_stop()
        stop_blinkers(strip)
        head_center()
        head_servo.angle = None   # relache le servo (stoppe le signal PWM)
        strip.led_close()
        pca.deinit()              # libere le PCA9685 (servo + moteurs)
        print("Programme termine.")
