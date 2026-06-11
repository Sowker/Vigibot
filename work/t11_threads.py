import time
from typing import Optional

from t11_robot import Robot
import logger


from t3_servomotors import STEER_HARD_DEG, STEER_SOFT_DEG
from t4_dc_motor import Direction, SPEED_BACKWARD, SPEED_TURNING_PCT, SPEED_NORMAL_PCT
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
        current_action = robot.line_tracker.read_action(switch_sensors=True)

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
        if action != last_action:
            log.info("Changement de comportement → %s", action.name)
            if action == LinePosition.LINE_LOST:
                log.warning("Ligne perdue — recherche active / attente…")
            elif action == LinePosition.INTERSECTION:
                log.info("Intersection détectée — passage tout droit")
            last_action = action

        if not robot.state.maneuver: # not in maneuver
            if count_maneuver != END_COUNT_MANEUVER:
                if (last_turn == -1 and (action == LinePosition.TURN_LEFT_SOFT or action == LinePosition.TURN_LEFT_HARD)) or (last_turn == 1 and (action == LinePosition.TURN_RIGHT_SOFT or action == LinePosition.TURN_RIGHT_HARD)):
                    count_maneuver = END_COUNT_MANEUVER
            if count_maneuver == END_COUNT_MANEUVER: # letting time to the forward (end) of the maneuver
                if action == LinePosition.STRAIGHT:
                    robot.head.steer_center()
                    robot.motor.drive(Direction.FORWARD, SPEED_NORMAL_PCT, fast_accel=True)
                    last_turn = 0

                elif action == LinePosition.TURN_LEFT_SOFT:
                    robot.head.steer_left(STEER_SOFT_DEG)
                    robot.motor.drive(Direction.FORWARD, SPEED_TURNING_PCT, fast_accel=True)
                    last_turn = -1

                elif action == LinePosition.TURN_RIGHT_SOFT:
                    robot.head.steer_right(STEER_SOFT_DEG)
                    robot.motor.drive(Direction.FORWARD, SPEED_TURNING_PCT, fast_accel=True)
                    last_turn = 1

                elif action == LinePosition.TURN_LEFT_HARD:
                    robot.head.steer_left(STEER_HARD_DEG)
                    robot.motor.drive(Direction.FORWARD, SPEED_BACKWARD, fast_accel=True)
                    last_turn = -1

                elif action == LinePosition.TURN_RIGHT_HARD:
                    robot.head.steer_right(STEER_HARD_DEG)
                    robot.motor.drive(Direction.FORWARD, SPEED_BACKWARD, fast_accel=True)
                    last_turn = 1

                elif action == LinePosition.INTERSECTION:
                    robot.head.steer_center()
                    robot.motor.drive(Direction.FORWARD, SPEED_NORMAL_PCT, fast_accel=True)
                    last_turn = 0

                else:  # LinePosition.LINE_LOST
                    log.info("lose the line, last turn is "+str(last_turn))
                    robot.motor.stop()
                    robot.head.steer_center()
                    if last_turn == 0:
                        robot.head.steer_center()
                        robot.motor.drive(Direction.FORWARD, SPEED_NORMAL_PCT, fast_accel=True)
                        robot.state.maneuver = False
                    elif last_turn == -1:  # if we were turning left
                        robot.head.steer_right(STEER_HARD_DEG)
                        robot.motor.drive(Direction.BACKWARD, SPEED_BACKWARD, fast_accel=True)
                    elif last_turn == 1:  # if we were turning right
                        robot.head.steer_left(STEER_HARD_DEG)
                        robot.motor.drive(Direction.BACKWARD, SPEED_BACKWARD, fast_accel=True)
                    robot.state.maneuver = True
            else:
                count_maneuver += 1
        else: # in maneuver
            if last_turn == -1: # if we were turning left
                if action == LinePosition.TURN_RIGHT_HARD:
                    log.info("LinePosition.TURN_RIGHT_HARD")
                    robot.head.steer_center()
                    robot.motor.drive(Direction.FORWARD, SPEED_TURNING_PCT, fast_accel=True)
                    robot.state.maneuver = False
                    count_maneuver = 0
            elif last_turn == 1: # if we were turning right
                if action == LinePosition.TURN_LEFT_HARD:
                    robot.head.steer_center()
                    robot.motor.drive(Direction.FORWARD, SPEED_TURNING_PCT, fast_accel=True)
                    robot.state.maneuver = False
                    count_maneuver = 0
                    log.info("LinePosition.TURN_LEFT_HARD")



        time.sleep(interval)

    # ── Arrêt propre en fin de thread ─────────────────────────────
    robot.motor.stop()
    robot.head.steer_center()
    log.info("Thread arrêté")


def thread_buzzer(robot: Robot) -> None:
    """
    - Obstacle (emergency_stop)         -> sirène POLICE
    - Manœuvre de récupération (maneuver) -> son défini par LINE_LOST_SOUND
    - Robot en mouvement (driving)      -> thème MII
    - À l'arrêt                         -> silence
    """
    log = logger.get_logger("BUZZER")
    log.info("Thread démarré")

    def emergency_active() -> bool:
        with robot.state.lock:
            return robot.state.running and robot.state.emergency_stop

    def maneuver_active() -> bool:
        with robot.state.lock:
            return robot.state.running and robot.state.maneuver and not robot.state.emergency_stop

    def driving_active() -> bool:
        with robot.state.lock:
            return robot.state.running and robot.state.driving and not robot.state.emergency_stop

    while False:
        with robot.state.lock:
            running   = robot.state.running
            emergency = robot.state.emergency_stop
            maneuver  = robot.state.maneuver
            driving   = robot.state.driving

        if not running:
            break

        if emergency:
            play(POLICE, emergency_active)
        elif maneuver and LINE_LOST_SOUND == "POLICE":
            play(POLICE, maneuver_active)
        elif maneuver and LINE_LOST_SOUND == "MII":
            play(MII, maneuver_active)
        elif driving:
            play(MII, driving_active)
        else:
            time.sleep(SENSOR_INTERVAL_S)

    log.info("Thread arrêté")

