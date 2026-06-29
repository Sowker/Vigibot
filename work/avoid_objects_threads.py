import time

from t11_robot import Robot
import logger


from t3_servomotors import WHEEL_ANGLE_MIN, WHEEL_ANGLE_MAX, HEAD_ANGLE_MIN, HEAD_ANGLE_CENTER, HEAD_ANGLE_MAX
from t4_dc_motor import Direction, SPEED_NORMAL_PCT

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
SCAN_DIST_ACTION = 30 # in cm !!!

TURN_RIGHT = True
TURN_LEFT = False

AVOID_OBJ_SPEED = SPEED_NORMAL_PCT * 0.5
BYPASS_SPEED = SPEED_NORMAL_PCT

def thread_ultrasonic(robot: Robot, interval: float) -> None:
    """Lit le capteur ultrason en boucle et met à jour RobotState."""
    log = logger.get_logger("US")
    log.info("Thread démarré (intervalle=%.3f s)", interval)
    global scan
    def scan_cm() -> list:
        HR_MOTOR = 1
        VR_MOTOR = 2
        data = []
        start_position = int(HEAD_ANGLE_CENTER + (SCAN_ANGLE/2))
        end_position = int(HEAD_ANGLE_CENTER - (SCAN_ANGLE/2))
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

        scan = scan_cm()

        # time.sleep(interval)

    log.info("Thread arrêté")

def bypass_side(scan, min_dist):
    index = scan.index(min_dist)
    angle = HEAD_ANGLE_CENTER - (SCAN_ANGLE / 2) + index
    if angle <= HEAD_ANGLE_CENTER:
        return TURN_LEFT
    else:
        return TURN_RIGHT


def bypass(robot, bypass_direction, obj_angle):
    if bypass_direction == TURN_RIGHT:
        turn = WHEEL_ANGLE_MIN
        counter_turn = WHEEL_ANGLE_MAX
    else:
        counter_turn = WHEEL_ANGLE_MAX
        turn = WHEEL_ANGLE_MIN

    sleep_time = 0.1 + 0.1 * (SCAN_ANGLE/2 - obj_angle)
    print("sleep time", sleep_time)

    # turn
    robot.head.set_angle_motor(0, turn)
    time.sleep(0.5)
    robot.motor.drive(Direction.FORWARD, BYPASS_SPEED)
    time.sleep(sleep_time)

    robot.motor.stop()

    # counter_turn
    robot.head.set_angle_motor(0, counter_turn)
    time.sleep(0.5)
    robot.motor.drive(Direction.FORWARD, BYPASS_SPEED)
    time.sleep(2*sleep_time)
    robot.motor.stop()

    # realign
    robot.head.set_angle_motor(0, turn)
    time.sleep(0.5)
    robot.motor.drive(Direction.FORWARD, BYPASS_SPEED)
    time.sleep(sleep_time)

    robot.motor.stop()

def get_absolute_angle(scan, dist):
    idx = scan.index(dist)
    print("idx ", idx)
    if idx <= SCAN_ANGLE/2: #left
        print("left")
        return SCAN_ANGLE/2 - idx
    else: # right
        print("right")
        return idx - SCAN_ANGLE/2


def thread_controller(robot: Robot, interval: float) -> None:
    """
    Boucle de décision : lit l'action synthétisée, décide et pilote les moteurs.
    """
    log  = logger.get_logger("CTRL")
    log.info("Thread démarré (intervalle=%.3f s)", interval)
    global scan
    try:
        while True:
            # DRIVING AVOID OBJECTS LOGIC
            if scan:
                actual_scan = scan
                min_dist = min(actual_scan)
                if min_dist <= SCAN_DIST_ACTION:
                    robot.motor.stop()
                    object_angle = get_absolute_angle(actual_scan, min_dist)
                    print("min_dist", min_dist)
                    print("object angle ", object_angle)
                    if bypass_side(actual_scan, min_dist) == TURN_RIGHT:
                        print("turn right")
                        bypass(robot, TURN_RIGHT, object_angle)
                    else:
                        print("turn left")
                        bypass(robot, TURN_LEFT, object_angle)
                else:
                    print("drive")
                    robot.motor.drive(Direction.FORWARD, AVOID_OBJ_SPEED)
            else:
                print("no data yet")

            time.sleep(interval)
    except KeyboardInterrupt:
        # ── Arrêt propre en fin de thread ─────────────────────────────
        robot.motor.stop()
        robot.head.steer_center()
        log.info("Thread arrêté")
