"""
╔════════════════════════════════════════════════════════════════════╗
║         Robot Line Follower — TEST manœuvre de récupération        ║
║         Team C — MasterCamp SE 2026                                ║
╠════════════════════════════════════════════════════════════════════╣
║  Variante de t11_line_following.py :                               ║
║    • Manœuvre de récupération en 2 phases (recul puis ré-avance)   ║
║      du côté opposé/correspondant au dernier virage connu.         ║
║    • Pas de thème MII pendant le roulage normal (silence).         ║
╚════════════════════════════════════════════════════════════════════╝
"""

# ═══════════════════════════════════════════════════════════════════
#  IMPORTS
# ═══════════════════════════════════════════════════════════════════
import time
import threading
import logging
from typing import Optional

import logger
from t11_argument_parser import parse_args
from t11_robot import Robot
from t11_threads import thread_ultrasonic, thread_line, thread_LED

from t3_servomotors import STEER_HARD_DEG, STEER_SOFT_DEG
from t4_dc_motor import Direction, SPEED_SLOW_PCT, SPEED_TURNING_PCT, SPEED_NORMAL_PCT
from t6_line_tracking import LinePosition
from t11_buzzer_Sirene import POLICE, MII, play

# ── Buzzer ─────────────────────────────────────────────────────────
# Son joué pendant la manœuvre de récupération (recul + ré-avance) :
#   None      -> silence
#   "MII"     -> thème MII
#   "POLICE"  -> sirène POLICE
LINE_LOST_SOUND = "MII"

SENSOR_INTERVAL_S = 0.05   # s — période des threads capteurs

# ── Manœuvre de récupération (ligne perdue en plein virage) ────────
MANEUVER_REVERSE_DURATION_S = 0.4   # recul, braqué du côté opposé au virage
MANEUVER_FORWARD_DURATION_S = 0.6   # ré-avance + réajustement du bon côté


# ═══════════════════════════════════════════════════════════════════
#  THREADS
# ═══════════════════════════════════════════════════════════════════

def thread_controller(robot: Robot, interval: float) -> None:
    """
    Boucle de décision : lit l'action synthétisée, décide et pilote les moteurs.
    Implémente une manœuvre de récupération en 2 phases quand la ligne est
    perdue en plein virage : recul (côté opposé) puis ré-avance (bon côté).
    """
    log  = logger.get_logger("CTRL")
    log.info("Thread démarré (intervalle=%.3f s)", interval)

    last_action: Optional[LinePosition] = None

    last_turn = 0  # -1 = dernier virage à gauche, 0 = tout droit/inconnu, 1 = droite
    maneuver_phase = "reverse"  # "reverse" puis "forward", en boucle tant que la ligne n'est pas retrouvée
    maneuver_t0 = 0.0

    while True:
        # ── Lecture atomique de l'état simplifié ──────────────────
        with robot.state.lock:
            if not robot.state.running:
                break
            emergency = robot.state.emergency_stop
            action    = robot.state.line_action
            maneuver  = robot.state.maneuver

        # ── Arrêt d'urgence obstacle (Priorité 1) ─────────────────
        if emergency:
            robot.motor.stop()
            robot.head.steer_center()
            with robot.state.lock:
                robot.state.driving  = False
                robot.state.maneuver = False
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

        if not maneuver:
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
                robot.motor.drive(Direction.FORWARD, SPEED_SLOW_PCT, fast_accel=True)
                last_turn = -1

            elif action == LinePosition.TURN_RIGHT_HARD:
                robot.head.steer_right(STEER_HARD_DEG)
                robot.motor.drive(Direction.FORWARD, SPEED_SLOW_PCT, fast_accel=True)
                last_turn = 1

            elif action == LinePosition.INTERSECTION:
                robot.head.steer_center()
                robot.motor.drive(Direction.FORWARD, SPEED_NORMAL_PCT, fast_accel=True)
                last_turn = 0

            else:  # LinePosition.LINE_LOST -> démarrage de la manœuvre de récupération
                log.warning("Démarrage manœuvre de récupération (dernier virage=%s)",
                             {-1: "gauche", 0: "inconnu", 1: "droite"}[last_turn])
                maneuver_phase = "reverse"
                maneuver_t0 = time.monotonic()
                with robot.state.lock:
                    robot.state.maneuver = True

            with robot.state.lock:
                robot.state.driving = action != LinePosition.LINE_LOST

        else:  # ── Manœuvre de récupération en cours ─────────────
            if action != LinePosition.LINE_LOST:
                # Ligne retrouvée -> fin de la manœuvre, le tour suivant
                # reprend le suivi normal avec la nouvelle action lue.
                log.info("Ligne retrouvée — fin de la manœuvre")
                with robot.state.lock:
                    robot.state.maneuver = False
                    robot.state.driving  = False
                continue

            elapsed = time.monotonic() - maneuver_t0

            if maneuver_phase == "reverse":
                # Recul, roue braquée du côté OPPOSÉ au virage en cours
                if last_turn == -1:
                    robot.head.steer_right(STEER_HARD_DEG)
                elif last_turn == 1:
                    robot.head.steer_left(STEER_HARD_DEG)
                else:
                    robot.head.steer_center()
                robot.motor.drive(Direction.BACKWARD, SPEED_TURNING_PCT, fast_accel=True)

                if elapsed >= MANEUVER_REVERSE_DURATION_S:
                    maneuver_phase = "forward"
                    maneuver_t0 = time.monotonic()

            else:  # "forward" — ré-avance + réajustement du bon côté
                if last_turn == -1:
                    robot.head.steer_left(STEER_HARD_DEG)
                elif last_turn == 1:
                    robot.head.steer_right(STEER_HARD_DEG)
                else:
                    robot.head.steer_center()
                robot.motor.drive(Direction.FORWARD, SPEED_SLOW_PCT, fast_accel=True)

                if elapsed >= MANEUVER_FORWARD_DURATION_S:
                    # Toujours pas de ligne -> on retente un cycle recul/avance
                    maneuver_phase = "reverse"
                    maneuver_t0 = time.monotonic()

            with robot.state.lock:
                robot.state.driving = False

        time.sleep(interval)

    # ── Arrêt propre en fin de thread ─────────────────────────────
    robot.motor.stop()
    robot.head.steer_center()
    log.info("Thread arrêté")


def thread_buzzer(robot: Robot) -> None:
    """
    - Obstacle (emergency_stop)           -> sirène POLICE
    - Manœuvre de récupération (maneuver) -> son défini par LINE_LOST_SOUND
    - Roulage normal / à l'arrêt          -> silence
    """
    log = logger.get_logger("BUZZER")
    log.info("Thread démarré")

    def emergency_active() -> bool:
        with robot.state.lock:
            return robot.state.running and robot.state.emergency_stop

    def maneuver_active() -> bool:
        with robot.state.lock:
            return robot.state.running and robot.state.maneuver and not robot.state.emergency_stop

    while True:
        with robot.state.lock:
            running   = robot.state.running
            emergency = robot.state.emergency_stop
            maneuver  = robot.state.maneuver

        if not running:
            break

        if emergency:
            play(POLICE, emergency_active)
        elif maneuver and LINE_LOST_SOUND == "POLICE":
            play(POLICE, maneuver_active)
        elif maneuver and LINE_LOST_SOUND == "MII":
            play(MII, maneuver_active)
        else:
            time.sleep(SENSOR_INTERVAL_S)

    log.info("Thread arrêté")


# ═══════════════════════════════════════════════════════════════════
#  POINT D'ENTRÉE
# ═══════════════════════════════════════════════════════════════════

if __name__ == "__main__":
    args = parse_args()

    if args.debug:
        logging.getLogger().setLevel(logging.DEBUG)

    log = logger.get_logger("MAIN")
    log.info("╔══════════════════════════════════════════════╗")
    log.info("║  TEST manœuvre de récupération — Team C      ║")
    log.info("╚══════════════════════════════════════════════╝")

    robot = Robot(args)
    robot.init()

    threads = [
        threading.Thread(target=thread_ultrasonic, args=(robot, args.sensor_interval), name="US", daemon=True),
        threading.Thread(target=thread_line, args=(robot, args.sensor_interval), name="LINE", daemon=True),
        threading.Thread(target=thread_LED, args=(robot, args.sensor_interval), name="LED", daemon=True),
        threading.Thread(target=thread_controller, args=(robot, args.ctrl_interval), name="CTRL", daemon=True),
        threading.Thread(target=thread_buzzer, args=(robot,), name="BUZZER", daemon=True),
    ]

    for t in threads:
        t.start()
        log.info("Thread %-4s démarré (ident: %s)", t.name, t.ident)

    log.info("Tous les threads actifs. Appuyez sur Ctrl+C pour arrêter.\n")

    try:
        while True:
            time.sleep(0.5)

    except KeyboardInterrupt:
        log.info("Ctrl+C reçu — arrêt en cours…")

    finally:
        with robot.state.lock:
            robot.state.running = False

        for t in threads:
            t.join(timeout=3.0)
            if t.is_alive():
                log.warning("Thread %s ne s'est pas arrêté dans le délai", t.name)

        robot.shutdown()

    log.info("Programme terminé. Au revoir !")
    log.info("Program developed by Team C — MasterCamp SE 2026.")
