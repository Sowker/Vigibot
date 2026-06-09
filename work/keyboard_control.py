"""
╔════════════════════════════════════════════════════════════════════╗
║         Robot Keyboard controlled — Main Controller                ║
║         Team C — MasterCamp SE 2026                                ║
╠════════════════════════════════════════════════════════════════════╣
║  Architecture multithreads :                                       ║
║    • Thread CTRL  — Décision + commande moteur/direction           ║
║    • Thread MAIN  — Démarrage, supervision, arrêt propre           ║
╚════════════════════════════════════════════════════════════════════╝
"""

# ═══════════════════════════════════════════════════════════════════
#  IMPORTS
# ═══════════════════════════════════════════════════════════════════
import keyboard
import time
import threading
import argparse
from dataclasses import dataclass, field
from typing import Optional

from t6_line_tracking import PIN_LINE_LEFT, PIN_LINE_MIDDLE, PIN_LINE_RIGHT


from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685

import logging
import logger

from t3_servomotors import Head, STEER_HARD_DEG, STEER_SOFT_DEG
from t4_dc_motor import DCMotor, Direction, SPEED_SLOW_PCT, SPEED_TURNING_PCT, SPEED_NORMAL_PCT
from t5_ultrasonic_sensor import UltrasonicSensor, PIN_ULTRASONIC_ECHO, PIN_ULTRASONIC_TRIGGER
from work.t3_servomotors import CHANNEL_SERVO_VERTICAL

# ═══════════════════════════════════════════════════════════════════
#  CONSTANTES DE CONFIGURATION
# ═══════════════════════════════════════════════════════════════════

# ── PCA9685 ────────────────────────────────────────────────────────
PCA_ADDRESS        = 0x5F
PCA_FREQUENCY_HZ   = 50

# ── Contrôleur ─────────────────────────────────────────────────────
OBSTACLE_THRESHOLD_MM = 150.0  # mm — seuil d'arrêt d'urgence

CTRL_INTERVAL_S       = 0.05   # s — période du thread contrôleur
SENSOR_INTERVAL_S     = 0.05   # s — période des threads capteurs

# ═══════════════════════════════════════════════════════════════════
#  ÉTAT PARTAGÉ (thread-safe)
# ═══════════════════════════════════════════════════════════════════

@dataclass
class RobotState:
    """
    Source de vérité unique entre tous les threads.
    Chaque accès en lecture/écriture doit être protégé par self.lock.
    """
    lock: threading.Lock = field(default_factory=threading.Lock)

    # ── Données capteurs synthétisées ──────────────────────────────
    distance_mm: float = 9999.0

    # ── Commandes de supervision ──────────────────────────────────
    running:        bool = True    # False → tous les threads s'arrêtent
    emergency_stop: bool = False   # True  → obstacle détecté


# ═══════════════════════════════════════════════════════════════════
#  ROBOT — CLASSE DE HAUT NIVEAU
# ═══════════════════════════════════════════════════════════════════

class Robot:
    """Façade unique rassemblant tous les composants matériels."""

    def __init__(self, cfg: argparse.Namespace):
        self._log = logger.get_logger("ROBOT")
        self._cfg = cfg

        self._log.info("Initialisation du bus I²C et PCA9685 (addr=0x%02X)…", PCA_ADDRESS)
        self._i2c = busio.I2C(SCL, SDA)
        self._pca = PCA9685(self._i2c, address=PCA_ADDRESS)
        self._pca.frequency = PCA_FREQUENCY_HZ

        self.ultrasonic   = UltrasonicSensor(cfg.us_trigger, cfg.us_echo)
        self.motor        = DCMotor(self._pca)
        self.head         = Head(self._pca)

        self.state = RobotState()
        self._obstacle_threshold_mm = cfg.obstacle_mm

    def init(self) -> None:
        self._log.info("══ Mise à zéro initiale ══")
        self.motor.reset()
        self.head.reset()
        time.sleep(0.5)
        self._log.info("Robot prêt.")

    def shutdown(self) -> None:
        self._log.info("══ Shutdown — remise à zéro ══")
        self.motor.reset()
        self.head.shutdown()
        time.sleep(0.5)
        self._pca.deinit()
        self._log.info("PCA9685 désactivé. Bonne journée !")


# ═══════════════════════════════════════════════════════════════════
#  THREADS
# ═══════════════════════════════════════════════════════════════════
def thread_controller(robot: Robot, interval: float) -> None:
    """
    Boucle de décision : lit l'action synthétisée, décide et pilote les moteurs.
    """
    log  = logger.get_logger("CTRL")
    log.info("Thread démarré (intervalle=%.3f s)", interval)

    keys_state = {'z': False, 'q': False, 's':False, 'd':False, # z q s d to control the direction
                  'o': False, 'l': False, # o l to control the head up and down
                  }
    vertical_angle = 0

    while True:
        # ── Lecture atomique de l'état simplifié ──────────────────
        with robot.state.lock:
            if not robot.state.running:
                break
            emergency = robot.state.emergency_stop

        # ── Arrêt d'urgence obstacle (Priorité 1) ─────────────────
        if emergency:
            robot.motor.stop()
            robot.head.steer_center()
            log.warning("⚠ OBSTACLE détecté — arrêt d'urgence")
            time.sleep(interval)
            continue

        # s'il y a un changement de touche pour avancer ou reculer on stop le robot
        if keyboard.is_pressed('z') != keys_state['z'] or keyboard.is_pressed('s') != keys_state['s']:
            robot.motor.stop()
        # s'il y a un changement de touche pour tourner à droite ou à gauche on mets les roues au centre
        if keyboard.is_pressed('q') != keys_state['q'] or keyboard.is_pressed('d') != keys_state['d']:
            robot.head.steer_center()
        # s'il y a un changement de touche pour la tête haut/bas on la met au centre sur cet axe
        if keyboard.is_pressed('o') != keys_state['o'] or keyboard.is_pressed('l') != keys_state['l']:
            robot.head.set_angle_motor(CHANNEL_SERVO_VERTICAL, 0)

        # mise à jour des états de touches préssées
        keys_state = {'z': keyboard.is_pressed('z'), 'q': keyboard.is_pressed('q'), 's': keyboard.is_pressed('s'), 'd': keyboard.is_pressed('d'),  # z q s d to control the direction
                      'o': keyboard.is_pressed('o'), 'l': keyboard.is_pressed('l')  # o k l m to control the head
        }

        if keyboard.is_pressed('z') and not keys_state['z']:
            robot.motor.drive(Direction.FORWARD, SPEED_NORMAL_PCT)
        elif keyboard.is_pressed('s') and not keys_state['s']:
            robot.motor.drive(Direction.BACKWARD, SPEED_NORMAL_PCT)

        if keyboard.is_pressed('q') and not keys_state['q']:
            robot.head.steer_right(STEER_HARD_DEG)
        elif keyboard.is_pressed('d') and not keys_state['d']:
            robot.head.steer_left(STEER_HARD_DEG)

        if keyboard.is_pressed('o') and not keys_state['o']:
            vertical_angle += 5
        elif keyboard.is_pressed('l') and not keys_state['l']:
            vertical_angle -= 5

        robot.head.set_angle_motor(CHANNEL_SERVO_VERTICAL, vertical_angle)

        time.sleep(interval)

    # ── Arrêt propre en fin de thread ─────────────────────────────
    robot.motor.stop()
    robot.head.steer_center()
    log.info("Thread arrêté")


# ═══════════════════════════════════════════════════════════════════
#  ARGUMENTS CLI
# ═══════════════════════════════════════════════════════════════════

def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Robot Keyboard controlled — Team C — MasterCamp SE 2026",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )

    g_us = p.add_argument_group("Capteur ultrason")
    g_us.add_argument("--us-trigger",   type=int,   default=PIN_ULTRASONIC_TRIGGER)
    g_us.add_argument("--us-echo",      type=int,   default=PIN_ULTRASONIC_ECHO)
    g_us.add_argument("--obstacle-mm",  type=float, default=OBSTACLE_THRESHOLD_MM)

    g_line = p.add_argument_group("Capteurs de ligne")
    g_line.add_argument("--line-left",  type=int, default=PIN_LINE_LEFT)
    g_line.add_argument("--line-mid",   type=int, default=PIN_LINE_MIDDLE)
    g_line.add_argument("--line-right", type=int, default=PIN_LINE_RIGHT)

    g_timing = p.add_argument_group("Timing")
    g_timing.add_argument("--ctrl-interval",   type=float, default=CTRL_INTERVAL_S)
    g_timing.add_argument("--sensor-interval", type=float, default=SENSOR_INTERVAL_S)

    p.add_argument("--debug", action="store_true", help="Active les logs DEBUG")

    return p.parse_args()


# ═══════════════════════════════════════════════════════════════════
#  POINT D'ENTRÉE
# ═══════════════════════════════════════════════════════════════════

if __name__ == "__main__":
    args = parse_args()

    if args.debug:
        logging.getLogger().setLevel(logging.DEBUG)

    log = logger.get_logger("MAIN")
    log.info("╔══════════════════════════════════════════════╗")
    log.info("║ Robot Keyboard Controlled — Team C — SE 2026 ║")
    log.info("╚══════════════════════════════════════════════╝")

    robot = Robot(args)
    robot.init()

    threads = [
        threading.Thread(target=thread_controller, args=(robot, args.ctrl_interval), name="CTRL", daemon=True),
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
