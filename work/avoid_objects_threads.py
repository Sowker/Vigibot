import time
from typing import Optional

from t11_robot import Robot
import logger


from t3_servomotors import WHEEL_ANGLE_MIN, WHEEL_ANGLE_MAX
from t4_dc_motor import Direction, SPEED_BACKWARD, SPEED_TURNING_PCT, SPEED_NORMAL_PCT, SPEED_ADJUSTING_PCT, SPEED_HIGH
from t6_line_tracking import LinePosition
from t11_buzzer_Sirene import POLICE, MII, play

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

def thread_ultrasonic(robot: Robot, interval: float) -> None:
    """Lit le capteur ultrason en boucle et met à jour RobotState."""
    log = logger.get_logger("US")
    log.info("Thread démarré (intervalle=%.3f s)", interval)

    while True:
        with robot.state.lock:
            if not robot.state.running:
                break

        dist_mm = robot.ultrasonic.read_mm()

        with robot.state.lock:
            robot.state.distance_mm    = dist_mm
            robot.state.emergency_stop = dist_mm < robot._obstacle_threshold_mm

        time.sleep(interval)

    log.info("Thread arrêté")


def thread_line(robot: Robot, interval: float) -> None:
    """
    Lit l'action décodée des capteurs en boucle (via read_action)
    et met à jour directement l'action sur le RobotState.
    """
    log = logger.get_logger("LINE")
    log.info("Thread démarré (intervalle=%.3f s)", interval)

    while True:
        with robot.state.lock:
            if not robot.state.running:
                break

        # Capture matérielle et décodage atomique (Hors du Lock pour optimiser)
        current_action = robot.line_tracker.read_action()

        with robot.state.lock:
            robot.state.line_action = current_action

        time.sleep(interval)

    log.info("Thread arrêté")

def thread_LED(robot: Robot, interval: float):
    """
    Pilote les LEDs arrière (WS2812) ET les LEDs avant en parallèle :
    - Obstacle (priorité 1) ou ligne perdue -> warning (avant + arrière)
    - Virage gauche/droite -> clignotant correspondant (avant + arrière)
    - Tout droit / intersection -> extinction des deux
    """
    log = logger.get_logger("LED")
    log.info("Thread démarré (intervalle=%.3f s)", interval)

    last_front_state = None  # 'left', 'right', 'warning' ou None

    while True:
        with robot.state.lock:
            if not robot.state.running:
                break
            action    = robot.state.line_action
            emergency = robot.state.emergency_stop

        if emergency:
            target_state = 'warning'
            robot.led.warning()
        elif action == LinePosition.TURN_LEFT_SOFT or action == LinePosition.TURN_LEFT_HARD:
            target_state = 'left'
            robot.led.clignotant_gauche()
        elif action == LinePosition.TURN_RIGHT_SOFT or action == LinePosition.TURN_RIGHT_HARD:
            target_state = 'right'
            robot.led.clignotant_droit()
        elif action == LinePosition.LINE_LOST:
            target_state = 'warning'
            robot.led.warning()
        else:  # STRAIGHT / INTERSECTION
            target_state = None
            robot.led.arreter_clignotants()
            robot.led.arreter_warning()

        # front_leds.set_blink() est un toggle -> ne l'appeler que sur changement d'état
        if target_state != last_front_state:
            robot.front_leds.set_blink(target_state)
            last_front_state = target_state

        time.sleep(interval)

    robot.front_leds.cancel_blink()
    log.info("Thread arrêté")


def thread_controller(robot: Robot, interval: float) -> None:
    """
    Boucle de décision : lit l'action synthétisée, décide et pilote les moteurs.
    """
    log  = logger.get_logger("CTRL")
    log.info("Thread démarré (intervalle=%.3f s)", interval)

    last_action: Optional[LinePosition] = None

    last_turn = 0 # -1 left, 0 None, 1 Right
    # maneuver_state = 0 # 0 init maneuver, 1 running backward wait for line, 2 found line still going backward, 3 begin to loose line
    END_COUNT_MANEUVER = 100
    count_maneuver = END_COUNT_MANEUVER # letting time to the maneuver when we are going forward again

    while True:
        # ── Lecture atomique de l'état simplifié ──────────────────
        with robot.state.lock:
            if not robot.state.running:
                break
            emergency = robot.state.emergency_stop
            action    = robot.state.line_action
            maneuver = robot.state.maneuver

        # ── Arrêt d'urgence obstacle (Priorité 1) ─────────────────
        if emergency:
            robot.motor.stop()
            robot.head.steer_center()
            log.warning("⚠ OBSTACLE détecté — arrêt d'urgence")
            time.sleep(interval)
            continue

        # ── Suivi de ligne décodé (Priorité 2) ────────────────────
        sleep_time = 2

        robot.head.set_angle_motor(0,WHEEL_ANGLE_MAX)
        time.sleep(0.5)
        robot.motor.drive(Direction.FORWARD, SPEED_NORMAL_PCT)
        time.sleep(sleep_time)

        robot.head.set_angle_motor(0, WHEEL_ANGLE_MIN)
        time.sleep(0.5)
        robot.motor.drive(Direction.FORWARD, SPEED_NORMAL_PCT)
        time.sleep(2*sleep_time)

        robot.head.set_angle_motor(0, WHEEL_ANGLE_MAX)
        time.sleep(0.5)
        robot.motor.drive(Direction.FORWARD, SPEED_NORMAL_PCT)
        time.sleep(sleep_time)

        robot.motor.stop()
        robot.head.steer_center()
        time.sleep(20)

        time.sleep(interval)

    # ── Arrêt propre en fin de thread ─────────────────────────────
    robot.motor.stop()
    robot.head.steer_center()
    log.info("Thread arrêté")
