import time
from typing import Optional
from picamera2 import Picamera2

from t11_robot import Robot
import logger

from t3_servomotors import STEER_HARD_DEG, STEER_SOFT_DEG
from t4_dc_motor import Direction, SPEED_BACKWARD, SPEED_TURNING_PCT, SPEED_NORMAL_PCT, SPEED_ADJUSTING_PCT, SPEED_HIGH

from CameraDetection import get_direction, init_camera, shutdown, adjust_position

def get_arrow_derection(camera : Picamera2)->Direction:
    direction = 0
    for i in range(10):
        if get_direction(camera) == "left":
            direction -= 1
        else:
            direction += 1

    if direction > 0:
        # On average we detected that we need to turn right
        return "right"
    elif direction < 0:
        return "left"
    else:
        return "straight"

def L_turn(robot : Robot, direction : str) -> None:
    robot.motor.drive(Direction.FORWARD, SPEED_HIGH, fast_accel=True)
    time.sleep(0.03)
    if direction == "left":
        for i in range(7):
            robot.head.steer_left(STEER_HARD_DEG)
            robot.motor.drive(Direction.FORWARD, SPEED_HIGH, fast_accel=True)
            time.sleep(0.20)

            robot.head.steer_right(STEER_HARD_DEG)
            robot.motor.drive(Direction.BACKWARD, SPEED_HIGH, fast_accel=True)
            time.sleep(0.15)


    elif direction == "right":
        for i in range(7):
            robot.head.steer_right(STEER_HARD_DEG)
            robot.motor.drive(Direction.FORWARD, SPEED_HIGH, fast_accel=True)
            time.sleep(0.20)

            robot.head.steer_left(STEER_HARD_DEG)
            robot.motor.drive(Direction.BACKWARD, SPEED_HIGH, fast_accel=True)
            time.sleep(0.15)
    else:
        pass

def thread_drive(robot: Robot, interval: float, camera : Picamera2) -> None:
    log = logger.get_logger("CTRL")
    log.info("Thread démarré (intervalle=%.3f s)", interval)
    while True:
        # ── 1. Lecture de l'état actuel ───────────────────────────
        with robot.state.lock:
            if not robot.state.running:
                break
            emergency = robot.state.emergency_stop

        # ── 2. Gestion de l'urgence (Priorité Absolue) ────────────
        if emergency:
            robot.motor.stop()
            robot.head.steer_center()
            log.warning("⚠ OBSTACLE détecté — arrêt d'urgence")
            direction = get_arrow_derection(camera)
            L_turn(robot, direction)
        robot.head.steer_center()
        robot.motor.drive(Direction.FORWARD, SPEED_TURNING_PCT, fast_accel=True)

        #  ── 3. Adjust the position of the robot to be straigth ────────────
        corretion = adjust_position(camera)
        if corretion == "left":
            robot.head.steer_left(STEER_SOFT_DEG)
            robot.motor.drive(Direction.FORWARD, SPEED_TURNING_PCT, fast_accel=True)
        elif corretion == "right":
            robot.head.steer_right(STEER_SOFT_DEG)
            robot.motor.drive(Direction.FORWARD, SPEED_TURNING_PCT, fast_accel=True)
        else:
            pass


def thread_ultrasonic(robot: Robot, interval: float) -> None:
    """
    Lit le capteur ultrason en boucle pour détecter les obstacles.
    Met à jour l'état d'urgence du robot si un objet est trop près.
    """
    log = logger.get_logger("US")
    log.info("Thread démarré (intervalle=%.3f s)", interval)

    while True:
        # Vérifie si le robot est toujours censé fonctionner
        with robot.state.lock:
            if not robot.state.running:
                break

        # Lecture de la distance de l'obstacle en millimètres
        dist_mm = robot.ultrasonic.read_mm()

        # Mise à jour sécurisée de l'état global du robot
        with robot.state.lock:
            robot.state.distance_mm = dist_mm
            # Déclenche l'arrêt d'urgence si la distance est sous le seuil critique
            robot.state.emergency_stop = dist_mm < 250

        time.sleep(interval)

    log.info("Thread arrêté")
