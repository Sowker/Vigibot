import time
from typing import Optional

from OpenGL.raw.GLES2.OES import texture_half_float_linear

from t11_robot import Robot
import logger


from t3_servomotors import WHEEL_ANGLE_MIN, WHEEL_ANGLE_MAX, HEAD_ANGLE_MIN, HEAD_ANGLE_CENTER, HEAD_ANGLE_MAX
from t4_dc_motor import Direction, SPEED_BACKWARD, SPEED_TURNING_PCT, SPEED_NORMAL_PCT, SPEED_ADJUSTING_PCT, SPEED_HIGH

# Constantes

# ── Buzzer ─────────────────────────────────────────────────────────
# Son joué pendant les manœuvres de récupération (recul + virage quand
# la ligne est perdue) :
#   None      -> silence
#   "MII"     -> thème MII (comme en roulage normal)
#   "POLICE"  -> sirène POLICE (comme en urgence obstacle)
LINE_LOST_SOUND = "MII"

CTRL_INTERVAL_S       = 0.05   # s — période du thread contrôleur
SENSOR_INTERVAL_S     = 0.05   # s — période des threads capteurs

# ═══════════════════════════════════════════════════════════════════
#  THREADS
# ═══════════════════════════════════════════════════════════════════

scan = []

SCAN_ANGLE = 60
SCAN_DIST_ACTION = 20 # in cm !!!

TURN_RIGHT = True
TURN_LEFT = False

def thread_ultrasonic(robot: Robot, interval: float) -> None:
    """Lit le capteur ultrason en boucle et met à jour RobotState."""
    log = logger.get_logger("US")
    log.info("Thread démarré (intervalle=%.3f s)", interval)
    global scan
    def scan_20_cm() -> list:
        HR_MOTOR = 1
        VR_MOTOR = 2
        data = []
        start_position = int(HEAD_ANGLE_CENTER+(SCAN_ANGLE/2))
        end_position = int(HEAD_ANGLE_CENTER-(SCAN_ANGLE/2))
        robot.head.set_angle_motor(VR_MOTOR, HEAD_ANGLE_CENTER+5)
        robot.head.set_angle_motor(HR_MOTOR, start_position)
        time.sleep(0.3)
        for angle in range(start_position, end_position-1, -1):
            robot.head.set_angle_motor(HR_MOTOR, angle)
            time.sleep(0.01)
            data.append(robot.ultrasonic.read_mm()/10)
        robot.head.set_angle_motor(HR_MOTOR, HEAD_ANGLE_CENTER)
        return data

    while True:
        with robot.state.lock:
            if not robot.state.running:
                break

        scan = scan_20_cm()

        # time.sleep(interval)

    log.info("Thread arrêté")

def should_bypass_right(scan, min_dist):
    index = scan.index(min_dist)
    angle = HEAD_ANGLE_CENTER - (SCAN_ANGLE / 2) + index
    if angle <= 90:
        return TURN_RIGHT
    else:
        return TURN_LEFT


def bypass(robot, bypass_direction):
    if bypass_direction == TURN_RIGHT:
        turn = WHEEL_ANGLE_MIN
        counter_turn = WHEEL_ANGLE_MAX
    else:
        counter_turn = WHEEL_ANGLE_MAX
        turn = WHEEL_ANGLE_MIN

    sleep_time = 1.2

    # turn
    robot.head.set_angle_motor(0, turn)
    time.sleep(0.5)
    robot.motor.drive(Direction.FORWARD, SPEED_NORMAL_PCT)
    time.sleep(sleep_time)

    robot.motor.stop()

    # counter_turn
    robot.head.set_angle_motor(0, counter_turn)
    time.sleep(0.5)
    robot.motor.drive(Direction.FORWARD, SPEED_NORMAL_PCT)
    time.sleep(2*sleep_time)
    robot.motor.stop()

    # realign
    robot.head.set_angle_motor(0, turn)
    time.sleep(0.5)
    robot.motor.drive(Direction.FORWARD, SPEED_NORMAL_PCT)
    time.sleep(sleep_time)

    robot.motor.stop()


def thread_controller(robot: Robot, interval: float) -> None:
    """
    Boucle de décision : lit l'action synthétisée, décide et pilote les moteurs.
    """
    log  = logger.get_logger("CTRL")
    log.info("Thread démarré (intervalle=%.3f s)", interval)
    global scan

    while True:
        # DRIVING AVOID OBJECTS LOGIC
        if scan:
            actual_scan = scan
            min_dist = min(actual_scan)
            if min_dist <= SCAN_DIST_ACTION:
                robot.motor.stop()
                if should_bypass_right(actual_scan, min_dist):
                    print("turn right")
                    bypass(robot, TURN_RIGHT)
                else:
                    print("turn left")
                    bypass(robot, TURN_LEFT)
            else:
                print("drive")
                robot.motor.drive(Direction.FORWARD, SPEED_NORMAL_PCT)
        else:
            print("no data yet")

        time.sleep(interval)

    # ── Arrêt propre en fin de thread ─────────────────────────────
    robot.motor.stop()
    robot.head.steer_center()
    log.info("Thread arrêté")
