"""
╔════════════════════════════════════════════════════════════════════╗
║         Robot Line Follower — TEST manœuvre de récupération        ║
║         Team C — MasterCamp SE 2026                                ║
╠════════════════════════════════════════════════════════════════════╣
║  Variante de t11_line_following.py :                               ║
║    • Tolère les lignes pointillées (debounce avant manœuvre).      ║
║    • Manœuvre de récupération en plusieurs phases :                ║
║        - "corner"  : pivot serré côté du dernier virage (angle     ║
║                       droit) avant de tenter le recul.             ║
║        - "reverse" : recul, braqué du côté opposé au virage.       ║
║        - "forward" : ré-avance + réajustement du bon côté.         ║
║      Si la direction du virage est inconnue, alterne le côté       ║
║      testé (gauche/droite) à chaque cycle reverse/forward.         ║
║    • Buzzer : silence en roulage normal, POLICE uniquement         ║
║      pendant la phase de recul ("marche arrière").                 ║
║    • Marge de sécurité accrue sur le capteur d'obstacle.           ║
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
from t4_dc_motor import Direction, SPEED_BACKWARD, SPEED_TURNING_PCT, SPEED_NORMAL_PCT
from t6_line_tracking import LinePosition
from t11_buzzer_Sirene import POLICE, play

SENSOR_INTERVAL_S = 0.05   # s — période des threads capteurs

# ── Ligne perdue / lignes pointillées ───────────────────────────────
LINE_LOST_DEBOUNCE_S = 0.15   # tolérance avant de déclarer la ligne réellement perdue

# ── Manœuvre de récupération (ligne perdue en plein virage) ─────────
MANEUVER_CORNER_DURATION_S  = 0.5   # pivot serré côté virage (angle droit) avant de reculer
MANEUVER_REVERSE_DURATION_S = 0.4   # recul, braqué du côté opposé au virage
MANEUVER_FORWARD_DURATION_S = 0.6   # ré-avance + réajustement du bon côté

# ── Obstacle ─────────────────────────────────────────────────────────
# Marge augmentée : le temps de réaction du capteur + la rampe d'arrêt
# du moteur (200 ms) faisaient détecter le mur trop tard avec 150 mm.
OBSTACLE_MARGIN_MM = 250.0


# ── État partagé minimal entre thread_controller et thread_buzzer ──
class _ManeuverAudio:
    def __init__(self):
        self.lock = threading.Lock()
        self.reversing = False  # True uniquement pendant la phase "reverse" (marche arrière)


_maneuver_audio = _ManeuverAudio()


def _set_reversing(value: bool) -> None:
    with _maneuver_audio.lock:
        _maneuver_audio.reversing = value


# ═══════════════════════════════════════════════════════════════════
#  THREADS
# ═══════════════════════════════════════════════════════════════════

def thread_controller(robot: Robot, interval: float) -> None:
    """
    Boucle de décision : lit l'action synthétisée, décide et pilote les moteurs.

    Manœuvre de récupération quand la ligne est perdue :
      1. "corner"  (uniquement si le dernier virage était HARD, donc
                    probable angle droit) : pivot serré en avant, côté
                    du dernier virage.
      2. "reverse" : recul, braqué du côté opposé au virage.
      3. "forward" : ré-avance, braqué du côté du virage.
      4. Si toujours pas de ligne après "forward", on retente un cycle
         reverse/forward. Si la direction du virage était inconnue
         (last_turn == 0), le côté testé est inversé à chaque cycle.
    """
    log  = logger.get_logger("CTRL")
    log.info("Thread démarré (intervalle=%.3f s)", interval)

    last_action: Optional[LinePosition] = None

    last_turn = 0      # -1 = dernier virage à gauche, 0 = tout droit/inconnu, 1 = droite
    last_hard = False  # True si le dernier virage avant la perte de ligne était HARD

    line_lost_t0: Optional[float] = None  # debounce anti lignes pointillées

    maneuver_phase = "reverse"  # "corner" -> "reverse" -> "forward" -> ("reverse" -> "forward" ...)
    maneuver_t0 = 0.0
    side_test = 0  # 0/1 — côté testé en cycle quand last_turn == 0 (inconnu)

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
            _set_reversing(False)
            line_lost_t0 = None
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
                last_hard = False
                line_lost_t0 = None

            elif action == LinePosition.TURN_LEFT_SOFT:
                robot.head.steer_left(STEER_SOFT_DEG)
                robot.motor.drive(Direction.FORWARD, SPEED_TURNING_PCT, fast_accel=True)
                last_turn = -1
                last_hard = False
                line_lost_t0 = None

            elif action == LinePosition.TURN_RIGHT_SOFT:
                robot.head.steer_right(STEER_SOFT_DEG)
                robot.motor.drive(Direction.FORWARD, SPEED_TURNING_PCT, fast_accel=True)
                last_turn = 1
                last_hard = False
                line_lost_t0 = None

            elif action == LinePosition.TURN_LEFT_HARD:
                robot.head.steer_left(STEER_HARD_DEG)
                robot.motor.drive(Direction.FORWARD, SPEED_BACKWARD, fast_accel=True)
                last_turn = -1
                last_hard = True
                line_lost_t0 = None

            elif action == LinePosition.TURN_RIGHT_HARD:
                robot.head.steer_right(STEER_HARD_DEG)
                robot.motor.drive(Direction.FORWARD, SPEED_BACKWARD, fast_accel=True)
                last_turn = 1
                last_hard = True
                line_lost_t0 = None

            elif action == LinePosition.INTERSECTION:
                robot.head.steer_center()
                robot.motor.drive(Direction.FORWARD, SPEED_NORMAL_PCT, fast_accel=True)
                last_turn = 0
                last_hard = False
                line_lost_t0 = None

            else:  # LinePosition.LINE_LOST
                now = time.monotonic()
                if line_lost_t0 is None:
                    # Première détection : on patiente (ligne pointillée ?)
                    # sans toucher moteur/direction.
                    line_lost_t0 = now
                elif now - line_lost_t0 >= LINE_LOST_DEBOUNCE_S:
                    # Perte confirmée -> démarrage de la manœuvre de récupération
                    if last_hard:
                        maneuver_phase = "corner"
                        log.warning("Démarrage manœuvre — pivot angle droit côté %s",
                                     "gauche" if last_turn == -1 else "droite")
                    else:
                        maneuver_phase = "reverse"
                        log.warning("Démarrage manœuvre de récupération (dernier virage=%s)",
                                     {-1: "gauche", 0: "inconnu", 1: "droite"}[last_turn])
                    maneuver_t0 = now
                    line_lost_t0 = None
                    with robot.state.lock:
                        robot.state.maneuver = True
                # sinon : encore dans la fenêtre de tolérance, on ne change rien

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
                _set_reversing(False)
                continue

            elapsed = time.monotonic() - maneuver_t0

            if maneuver_phase == "corner":
                # Pivot serré en avant, côté du dernier virage (angle droit)
                if last_turn == -1:
                    robot.head.steer_left(STEER_HARD_DEG)
                elif last_turn == 1:
                    robot.head.steer_right(STEER_HARD_DEG)
                else:
                    robot.head.steer_center()
                robot.motor.drive(Direction.FORWARD, SPEED_BACKWARD, fast_accel=True)
                _set_reversing(False)

                if elapsed >= MANEUVER_CORNER_DURATION_S:
                    maneuver_phase = "reverse"
                    maneuver_t0 = time.monotonic()

            elif maneuver_phase == "reverse":
                # Recul, roue braquée du côté OPPOSÉ au virage en cours
                # (ou côté testé si la direction du virage est inconnue)
                if last_turn == -1:
                    robot.head.steer_right(STEER_HARD_DEG)
                elif last_turn == 1:
                    robot.head.steer_left(STEER_HARD_DEG)
                elif side_test == 0:
                    robot.head.steer_right(STEER_HARD_DEG)
                else:
                    robot.head.steer_left(STEER_HARD_DEG)
                robot.motor.drive(Direction.BACKWARD, SPEED_TURNING_PCT, fast_accel=True)
                _set_reversing(True)

                if elapsed >= MANEUVER_REVERSE_DURATION_S:
                    maneuver_phase = "forward"
                    maneuver_t0 = time.monotonic()

            else:  # "forward" — ré-avance + réajustement du bon côté
                if last_turn == -1:
                    robot.head.steer_left(STEER_HARD_DEG)
                elif last_turn == 1:
                    robot.head.steer_right(STEER_HARD_DEG)
                elif side_test == 0:
                    robot.head.steer_left(STEER_HARD_DEG)
                else:
                    robot.head.steer_right(STEER_HARD_DEG)
                robot.motor.drive(Direction.FORWARD, SPEED_BACKWARD, fast_accel=True)
                _set_reversing(False)

                if elapsed >= MANEUVER_FORWARD_DURATION_S:
                    # Toujours pas de ligne -> on retente un cycle recul/avance
                    maneuver_phase = "reverse"
                    maneuver_t0 = time.monotonic()
                    if last_turn == 0:
                        # Direction inconnue : on inverse le côté testé
                        side_test = 1 - side_test

            with robot.state.lock:
                robot.state.driving = False

        time.sleep(interval)

    # ── Arrêt propre en fin de thread ─────────────────────────────
    robot.motor.stop()
    robot.head.steer_center()
    log.info("Thread arrêté")


def thread_buzzer(robot: Robot) -> None:
    """
    - Obstacle (emergency_stop)              -> sirène POLICE
    - Manœuvre, phase "reverse" (marche AR)  -> sirène POLICE
    - Sinon (roulage normal, manœuvre avant, à l'arrêt) -> silence
    """
    log = logger.get_logger("BUZZER")
    log.info("Thread démarré")

    def emergency_active() -> bool:
        with robot.state.lock:
            return robot.state.running and robot.state.emergency_stop

    def reversing_active() -> bool:
        with robot.state.lock:
            if not (robot.state.running and robot.state.maneuver and not robot.state.emergency_stop):
                return False
        with _maneuver_audio.lock:
            return _maneuver_audio.reversing

    while True:
        with robot.state.lock:
            running   = robot.state.running
            emergency = robot.state.emergency_stop
            maneuver  = robot.state.maneuver

        if not running:
            break

        with _maneuver_audio.lock:
            reversing = _maneuver_audio.reversing

        if emergency:
            play(POLICE, emergency_active)
        elif maneuver and reversing:
            play(POLICE, reversing_active)
        else:
            time.sleep(SENSOR_INTERVAL_S)

    log.info("Thread arrêté")


# ═══════════════════════════════════════════════════════════════════
#  POINT D'ENTRÉE
# ═══════════════════════════════════════════════════════════════════

if __name__ == "__main__":
    args = parse_args()

    # Marge de sécurité accrue sur le capteur d'obstacle (cf. en-tête).
    args.obstacle_mm = max(args.obstacle_mm, OBSTACLE_MARGIN_MM)

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
