import time

from t11_robot import Robot
import logger


from t3_servomotors import WHEEL_ANGLE_MIN, WHEEL_ANGLE_MAX, WHEEL_ANGLE_CENTER, HEAD_ANGLE_MIN, HEAD_ANGLE_CENTER, HEAD_ANGLE_MAX
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

SCAN_ANGLE = 80
SCAN_DIST_ACTION = 20 # in cm !!!

TURN_RIGHT = True
TURN_LEFT = False

AVOID_OBJ_SPEED = SPEED_NORMAL_PCT * 0.35
BYPASS_SPEED = SPEED_NORMAL_PCT * 0.8

SCAN_STEP = 5
SCAN_WAIT_TIME = 0.07

def thread_ultrasonic_scanning(robot: Robot, interval: float) -> None:
    """Lit le capteur ultrason en boucle en balayant de droite à gauche et met à jour la variable global scan."""
    log = logger.get_logger("US")
    log.info("Thread démarré (intervalle=%.3f s)", interval)
    global scan
    def scan_cm() -> list:
        # scanning from left to right using the ultrasonic module
        HR_MOTOR = 1
        VR_MOTOR = 2
        data = []
        start_position = int(HEAD_ANGLE_CENTER - (SCAN_ANGLE/2))  # right
        end_position = int(HEAD_ANGLE_CENTER + (SCAN_ANGLE/2))    # left
        robot.head.set_angle_motor(VR_MOTOR, HEAD_ANGLE_CENTER + 5) # looking forward vertically
        robot.head.set_angle_motor(HR_MOTOR, start_position)      #setting at start position
        time.sleep(0.2) # waiting head to be ready
        data_str = ""
        for angle in range(start_position, end_position+1, SCAN_STEP): # scanning from left ro right
            robot.head.set_angle_motor(HR_MOTOR, angle)
            time.sleep(SCAN_WAIT_TIME)
            distance_cm = robot.ultrasonic.read_mm()/10
            data.append(distance_cm)
            data_str = str(round(distance_cm)) + " " + data_str
        print(data_str)
        print()
        robot.head.set_angle_motor(HR_MOTOR, HEAD_ANGLE_CENTER)
        return data

    while True:
        with robot.state.lock:
            if not robot.state.running:
                break

        scan = scan_cm() # scanning and putting the result in the global scan variable

        # time.sleep(interval) # no need of a time interval because there are already time.sleep statement in scan_cm

    log.info("Thread arrêté")


def bypass_side(index):
    """Determine if we should bypass by the left of the right, given an index"""
    angle = HEAD_ANGLE_CENTER - (SCAN_ANGLE / 2) + index * SCAN_STEP
    if angle <= HEAD_ANGLE_CENTER: # if object on the right
        return TURN_LEFT
    else: # object on the left
        return TURN_RIGHT


def bypass(robot, bypass_direction, obj_angle):
    """Bypassing an object by the left or by the right"""
    if bypass_direction == TURN_RIGHT: # good direction from indications
        turn = WHEEL_ANGLE_MIN
        counter_turn = WHEEL_ANGLE_MAX
    else:
        turn = WHEEL_ANGLE_MAX
        counter_turn = WHEEL_ANGLE_MIN


    # the sleep time allow to do a bigger or smaller maneuver depending on where is the obj (obj_angle)
    # sleep_time = 0.1 + 0.1 * (SCAN_ANGLE/2 - obj_angle)
    # print("sleep time", sleep_time)
    sleep_time = 2

    # # backward a bit first
    # robot.motor.drive(Direction.FORWARD, AVOID_OBJ_SPEED)
    # robot.head.set_angle_motor(0, WHEEL_ANGLE_CENTER)
    # time.sleep(sleep_time/5) # TODO ajuster selon la distance avec l'obstacle

    # turn
    robot.head.set_angle_motor(0, turn)
    time.sleep(0.3)
    robot.motor.drive(Direction.FORWARD, BYPASS_SPEED)
    time.sleep(sleep_time)

    robot.motor.stop()

    # counter_turn
    robot.head.set_angle_motor(0, counter_turn)
    time.sleep(0.3)
    robot.motor.drive(Direction.FORWARD, BYPASS_SPEED)
    time.sleep(2.5*sleep_time)
    robot.motor.stop()

    # realign
    robot.head.set_angle_motor(0, turn)
    time.sleep(0.3)
    robot.motor.drive(Direction.FORWARD, BYPASS_SPEED)
    time.sleep(sleep_time*0.5)

    # reset T pose
    robot.motor.stop()
    robot.head.set_angle_motor(0, WHEEL_ANGLE_CENTER)

# def get_absolute_angle(scan, idx):
#     """From a given distance in a scan we determine the absolute angle from the front of the robot"""
#     if idx <= SCAN_ANGLE/2: #left
#         print("left")
#         return SCAN_ANGLE/2 - idx
#     else: # right
#         print("right")
#         return idx - SCAN_ANGLE/2


def thread_controller(robot: Robot, interval: float) -> None:
    """
    Boucle de décision : lit l'action synthétisée, décide et pilote les moteurs.
    """
    log  = logger.get_logger("CTRL")
    log.info("Thread démarré (intervalle=%.3f s)", interval)
    global scan
    try:
        driving = False
        while True:
            with robot.state.lock: # stopping the loop when program is stopped
                if not robot.state.running:
                    break

            # DRIVING AVOID OBJECTS LOGIC
            if scan:
                actual_scan = scan
                min_dist = min(actual_scan)
                if min_dist <= SCAN_DIST_ACTION:
                    # doing a second scan when we are stopped
                    robot.motor.stop()
                    time.sleep(SCAN_ANGLE/SCAN_STEP * SCAN_WAIT_TIME +0.3)
                    actual_scan = scan
                    min_dist = min(actual_scan)
                    driving = False

                    # print("min_dist", min_dist)
                    min_dist_idx = scan.index(min_dist)
                    # print("min_dist_idx", min_dist_idx)
                    object_angle = 0  # = get_absolute_angle(actual_scan, min_dist_idx)
                    # print("object angle ", object_angle)

                    if bypass_side(min_dist_idx) == TURN_RIGHT:
                        print("turn right")
                        robot.motor.stop()
                        # input("next action")
                        bypass(robot, TURN_RIGHT, object_angle)
                    else:
                        print("turn left")
                        robot.motor.stop()
                        # input("next action")
                        bypass(robot, TURN_LEFT, object_angle)
                elif not driving:
                    print("drive")
                    robot.motor.stop()
                    # input("next action")
                    driving = True
                    robot.head.set_angle_motor(0, WHEEL_ANGLE_CENTER)
                    robot.motor.drive(Direction.FORWARD, AVOID_OBJ_SPEED)
            else:
                print("no data yet")

            time.sleep(interval)
    except KeyboardInterrupt:
        # ── Arrêt propre en fin de thread ─────────────────────────────
        robot.motor.stop()
        robot.head.steer_center()
        log.info("Thread arrêté")
