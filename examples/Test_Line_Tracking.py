import time
import importlib.util
import os
import sys

# ---------------------------------------------------------------------------
# Chargement dynamique des modules necessaires
# ---------------------------------------------------------------------------
_here    = os.path.dirname(os.path.abspath(__file__))
_web_dir = os.path.join(_here, "..", "web")

def _load_module(name, filepath):
    spec = importlib.util.spec_from_file_location(name, filepath)
    m    = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m

# LineTrackingModule  — repris de 08_lineTracking.py
_lt_mod          = _load_module("line_tracking_08",
                                os.path.join(_here, "08_lineTracking.py"))
LineTrackingModule = _lt_mod.LineTrackingModule
Color              = _lt_mod.Color

# ServoHorizontal    — repris de Test_Servo_Horizontal.py (PCA9685 persistant)
_servo_mod       = _load_module("test_servo_horizontal",
                                os.path.join(_here, "Test_Servo_Horizontal.py"))
ServoHorizontal  = _servo_mod.ServoHorizontal

# move               — repris de web/move.py (controle moteurs DC)
sys.path.insert(0, _web_dir)
import move

# ---------------------------------------------------------------------------
# Parametres de conduite (repris de functions.py / trackLineProcessing)
# ---------------------------------------------------------------------------
DRIVE_SPEED   = 28    # vitesse roues (0-100)
HEAD_ANGLE_L  = 130   # angle tete quand virage gauche
HEAD_ANGLE_C  = 90    # angle tete tout droit
HEAD_ANGLE_R  = 50    # angle tete quand virage droit


def process(sensors, head):
    """Decide du mouvement roues + tete selon les 3 capteurs de ligne.

    Convention capteurs (reprise de 08_lineTracking.py) :
        0 = ligne noire detectee   1 = sol blanc
    """
    left, middle, right = sensors

    # --- ligne centree (capteur milieu sur le noir) ---
    if middle == 0 and left == 1 and right == 1:
        head.set_angle(HEAD_ANGLE_C)
        move.move(DRIVE_SPEED, 1, "mid")
        return f"{Color.ACTION_GO}Tout droit{Color.END}"

    # --- derive a droite : capteur gauche capte le noir ---
    if left == 0 and right == 1:
        head.set_angle(HEAD_ANGLE_L)
        move.move(DRIVE_SPEED, 1, "left")
        return f"{Color.ACTION_TURN}Virage gauche ←{Color.END}"

    # --- derive a gauche : capteur droit capte le noir ---
    if left == 1 and right == 0:
        head.set_angle(HEAD_ANGLE_R)
        move.move(DRIVE_SPEED, 1, "right")
        return f"{Color.ACTION_TURN}Virage droit →{Color.END}"

    # --- intersection / tous les capteurs sur le noir ---
    if left == 0 and middle == 0 and right == 0:
        head.set_angle(HEAD_ANGLE_C)
        move.move(DRIVE_SPEED, 1, "mid")
        return f"{Color.ACTION_GO}Intersection{Color.END}"

    # --- ligne perdue ---
    head.set_angle(HEAD_ANGLE_C)
    move.motorStop()
    return f"{Color.ACTION_WARN}Ligne perdue — arret{Color.END}"


if __name__ == "__main__":
    move.setup()
    sensors = LineTrackingModule(pin_left=22, pin_middle=27, pin_right=17)
    head    = ServoHorizontal(channel=1)

    print("=== Test Line Tracking (roues + tete) ===")
    print("0 = ligne noire  |  1 = sol blanc")
    print("Ctrl+C pour arreter\n")

    last_action = None

    try:
        while True:
            reading    = sensors.read()
            visual_bar = sensors.get_visual_bar()
            raw        = f"(L:{reading[0]} M:{reading[1]} R:{reading[2]})"
            action     = process(reading, head)

            if action != last_action:
                print(f"{visual_bar} {raw:<14} -> {action}")
                last_action = action
            else:
                print(f"{visual_bar} {raw}", end="\r")

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nArret — retour au centre et liberation GPIO...")
        move.motorStop()
        head.destroy()
        move.destroy()
        print("Programme termine.")
