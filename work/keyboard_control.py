"""
╔════════════════════════════════════════════════════════════════════╗
║         Robot Keyboard controlled — Main Controller                ║
║         Team C — MasterCamp SE 2026                                ║
╠════════════════════════════════════════════════════════════════════╣
║  Architecture simplifiée :                                         ║
║    • Saisie et exécution directes via input() standard             ║
╚════════════════════════════════════════════════════════════════════╝
"""

import time
import argparse

from t6_line_tracking import PIN_LINE_LEFT, PIN_LINE_MIDDLE, PIN_LINE_RIGHT

from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685

import logging
import logger

from t3_servomotors import Head, STEER_HARD_DEG, STEER_SOFT_DEG, CHANNEL_SERVO_VERTICAL
from t4_dc_motor import DCMotor, Direction, SPEED_SLOW_PCT, SPEED_TURNING_PCT, SPEED_NORMAL_PCT
from t5_ultrasonic_sensor import UltrasonicSensor, PIN_ULTRASONIC_ECHO, PIN_ULTRASONIC_TRIGGER

# ═══════════════════════════════════════════════════════════════════
#  CONSTANTES DE CONFIGURATION
# ═══════════════════════════════════════════════════════════════════

PCA_ADDRESS = 0x5F
PCA_FREQUENCY_HZ = 50
OBSTACLE_THRESHOLD_MM = 150.0


# ═══════════════════════════════════════════════════════════════════
#  ROBOT — CLASSE DE HAUT NIVEAU
# ═══════════════════════════════════════════════════════════════════

class Robot:
    def __init__(self, cfg: argparse.Namespace):
        self._log = logger.get_logger("ROBOT")
        self._cfg = cfg

        self._log.info("Initialisation du bus I²C et PCA9685 (addr=0x%02X)…", PCA_ADDRESS)
        self._i2c = busio.I2C(SCL, SDA)
        self._pca = PCA9685(self._i2c, address=PCA_ADDRESS)
        self._pca.frequency = PCA_FREQUENCY_HZ

        self.ultrasonic = UltrasonicSensor(cfg.us_trigger, cfg.us_echo)
        self.motor = DCMotor(self._pca)
        self.head = Head(self._pca)

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
#  ARGUMENTS CLI
# ═══════════════════════════════════════════════════════════════════

def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Robot Keyboard controlled — Team C — MasterCamp SE 2026",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    g_us = p.add_argument_group("Capteur ultrason")
    g_us.add_argument("--us-trigger", type=int, default=PIN_ULTRASONIC_TRIGGER)
    g_us.add_argument("--us-echo", type=int, default=PIN_ULTRASONIC_ECHO)
    g_us.add_argument("--obstacle-mm", type=float, default=OBSTACLE_THRESHOLD_MM)

    g_line = p.add_argument_group("Capteurs de ligne")
    g_line.add_argument("--line-left", type=int, default=PIN_LINE_LEFT)
    g_line.add_argument("--line-mid", type=int, default=PIN_LINE_MIDDLE)
    g_line.add_argument("--line-right", type=int, default=PIN_LINE_RIGHT)

    p.add_argument("--debug", action="store_true", help="Active les logs DEBUG")
    return p.parse_args()


# ═══════════════════════════════════════════════════════════════════
#  POINT D'ENTRÉE (Séquentiel classique)
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

    print("\n=== GUIDE DES COMMANDES ===")
    print("Moteurs   : z (Avancer), s (Reculer), a (Stopper Moteurs)")
    print("Direction : q (Braquer Gauche), d (Braquer Droite), c (Centrer Roues)")
    print("Tête      : o (Regarder Haut), l (Regarder Bas)")
    print("Quitter   : exit\n")
    print("👉 Tapez la lettre puis appuyez sur ENTREE pour valider.\n")

    vertical_angle = 90.0

    try:
        while True:
            # Saisie classique bloquante
            cmd = input("Commande > ").strip().lower()

            if cmd == "exit":
                break

            # Actionneurs Moteurs
            if cmd == 'z':
                log.info("Action : En avant")
                robot.motor.drive(Direction.FORWARD, SPEED_NORMAL_PCT)
            elif cmd == 's':
                log.info("Action : En arrière")
                robot.motor.drive(Direction.BACKWARD, SPEED_NORMAL_PCT)
            elif cmd == 'a':
                log.info("Action : Stop moteurs")
                robot.motor.stop()

            # Actionneurs Servos (Direction)
            elif cmd == 'q':
                log.info("Action : Direction Gauche")
                robot.head.steer_left(STEER_HARD_DEG)
            elif cmd == 'd':
                log.info("Action : Direction Droite")
                robot.head.steer_right(STEER_HARD_DEG)
            elif cmd == 'c':
                log.info("Action : Centre la direction")
                robot.head.steer_center()

            # Actionneurs Servo Vertical Tête
            elif cmd == 'o':
                vertical_angle = min(170.0, vertical_angle + 15.0)
                log.info("Action : Tête vers le haut (%.1f°)", vertical_angle)
                robot.head.set_angle_motor(CHANNEL_SERVO_VERTICAL, vertical_angle)
            elif cmd == 'l':
                vertical_angle = max(10.0, vertical_angle - 15.0)
                log.info("Action : Tête vers le bas (%.1f°)", vertical_angle)
                robot.head.set_angle_motor(CHANNEL_SERVO_VERTICAL, vertical_angle)

            elif cmd != "":
                print("Commande invalide. Utilisez : z, s, a, q, d, c, o, l ou exit")

    except KeyboardInterrupt:
        log.info("Arrêt par Ctrl+C…")

    finally:
        robot.shutdown()
        log.info("Programme terminé. Au revoir !")