"""
╔════════════════════════════════════════════════════════════════════╗
║         Robot Line Follower — Main Controller                      ║
║         Team C — MasterCamp SE 2026                                ║
╠════════════════════════════════════════════════════════════════════╣
║  Architecture multithreads :                                       ║
║    • Thread US    — Capteur ultrason (obstacle)                    ║
║    • Thread LINE  — Capteurs de ligne infrarouges avec read_action ║
║    • Thread CTRL  — Décision + commande moteur/direction           ║
║    • Thread MAIN  — Démarrage, supervision, arrêt propre           ║
╚════════════════════════════════════════════════════════════════════╝
"""

# ═══════════════════════════════════════════════════════════════════
#  IMPORTS
# ═══════════════════════════════════════════════════════════════════

import time
import threading
import argparse
import logging
from enum import IntEnum
from dataclasses import dataclass, field
from typing import Optional

from gpiozero import DistanceSensor, InputDevice
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import motor, servo as adafruit_servo


# ═══════════════════════════════════════════════════════════════════
#  LOGGING
# ═══════════════════════════════════════════════════════════════════

logging.basicConfig(
    level=logging.INFO,
    format="[%(asctime)s] [%(name)-5s] %(message)s",
    datefmt="%H:%M:%S"
)

def get_logger(name: str) -> logging.Logger:
    return logging.getLogger(name)


# ═══════════════════════════════════════════════════════════════════
#  CONSTANTES DE CONFIGURATION
# ═══════════════════════════════════════════════════════════════════

# ── PCA9685 ────────────────────────────────────────────────────────
PCA_ADDRESS        = 0x5F
PCA_FREQUENCY_HZ   = 50

# ── Canaux PCA9685 ─────────────────────────────────────────────────
CHANNEL_SERVO_WHEEL      = 0   # Roue directionnelle
CHANNEL_SERVO_HORIZONTAL = 1   # Tête pan (gauche/droite)
CHANNEL_SERVO_VERTICAL   = 2   # Tête tilt (haut/bas)
CHANNEL_MOTOR_IN1        = 15  # Moteur DC pôle +
CHANNEL_MOTOR_IN2        = 14  # Moteur DC pôle −

# ── GPIO capteurs ──────────────────────────────────────────────────
PIN_ULTRASONIC_TRIGGER = 23
PIN_ULTRASONIC_ECHO    = 24
PIN_LINE_LEFT          = 22
PIN_LINE_MIDDLE        = 27
PIN_LINE_RIGHT         = 17

# ── Servos ─────────────────────────────────────────────────────────
SERVO_MIN_PULSE_US  = 500    # µs — impulsion minimale
SERVO_MAX_PULSE_US  = 2400   # µs — impulsion maximale
SERVO_RANGE_DEG     = 180    # degrés d'actuation totale
SERVO_INIT_DELAY_S  = 0.3    # délai après positionnement initial

# Limites angulaires (mécaniques)
WHEEL_ANGLE_MIN     = 45     # degrés — braquage gauche max
WHEEL_ANGLE_CENTER  = 90     # degrés — tout droit
WHEEL_ANGLE_MAX     = 135    # degrés — braquage droite max

HEAD_ANGLE_MIN      = 10     # degrés
HEAD_ANGLE_CENTER   = 90     # degrés
HEAD_ANGLE_MAX      = 170    # degrés

# Pas et délai pour move_to()
SERVO_STEP_DEG      = 0.5    # degrés par pas (flottant)
SERVO_STEP_DELAY_S  = 0.008  # secondes entre chaque pas

# ── Moteur DC ──────────────────────────────────────────────────────
MOTOR_RAMP_MAX_TIME_S = 1.0  # durée max de la rampe d'accélération
MOTOR_RAMP_STEPS      = 10   # nombre de pas par seconde de rampe

# ── Contrôleur ─────────────────────────────────────────────────────
SPEED_NORMAL_PCT      = 60   # % puissance — ligne droite
SPEED_TURNING_PCT     = 50   # % puissance — virage doux
SPEED_SLOW_PCT        = 35   # % puissance — virage serré

STEER_SOFT_DEG        = 15   # degrés de braquage — correction légère
STEER_HARD_DEG        = 35   # degrés de braquage — virage serré

OBSTACLE_THRESHOLD_MM = 150.0  # mm — seuil d'arrêt d'urgence

CTRL_INTERVAL_S       = 0.05   # s — période du thread contrôleur
SENSOR_INTERVAL_S     = 0.05   # s — période des threads capteurs


# ═══════════════════════════════════════════════════════════════════
#  ÉNUMÉRATIONS
# ═══════════════════════════════════════════════════════════════════

class Direction(IntEnum):
    """Sens de marche du moteur DC."""
    FORWARD  =  1
    BACKWARD = -1


class LineAction(IntEnum):
    """Actions déduites des capteurs de ligne."""
    STRAIGHT        = 0
    TURN_LEFT_SOFT  = 1
    TURN_LEFT_HARD  = 2
    TURN_RIGHT_SOFT = 3
    TURN_RIGHT_HARD = 4
    INTERSECTION    = 5
    LINE_LOST       = 6


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
    line_action: LineAction = LineAction.LINE_LOST  # Stockage direct de l'action décodée

    # ── Commandes de supervision ──────────────────────────────────
    running:        bool = True    # False → tous les threads s'arrêtent
    emergency_stop: bool = False   # True  → obstacle détecté


# ═══════════════════════════════════════════════════════════════════
#  HARDWARE — CAPTEUR ULTRASON
# ═══════════════════════════════════════════════════════════════════

class UltrasonicSensor:
    """
    Capteur de distance HC-SR04 via gpiozero.DistanceSensor.
    Retourne la distance en millimètres.
    """

    def __init__(self,
                 trigger_pin: int   = PIN_ULTRASONIC_TRIGGER,
                 echo_pin:    int   = PIN_ULTRASONIC_ECHO,
                 max_dist_m:  float = 2.0):
        self._log = get_logger("US_HW")
        self._device = DistanceSensor(
            echo=echo_pin,
            trigger=trigger_pin,
            max_distance=max_dist_m
        )
        self.max_distance_mm = max_dist_m * 1000
        self._log.info("Capteur ultrason initialisé (trigger=%d, echo=%d)", trigger_pin, echo_pin)

    def read_mm(self) -> float:
        """Retourne la distance mesurée en millimètres."""
        return self._device.distance * 1000


# ═══════════════════════════════════════════════════════════════════
#  HARDWARE — CAPTEURS DE LIGNE
# ═══════════════════════════════════════════════════════════════════

class LineTracker:
    """
    Barrette de 3 capteurs infrarouges.
    Convention : 0 = ligne noire détectée, 1 = sol clair.
    """

    # Attribut de classe : partagé, alloué une seule fois en mémoire
    TRUTH_TABLE = {
        (1, 0, 1): LineAction.STRAIGHT,
        (0, 1, 1): LineAction.TURN_RIGHT_SOFT,
        (1, 1, 0): LineAction.TURN_LEFT_SOFT,
        (0, 0, 1): LineAction.TURN_RIGHT_HARD,
        (1, 0, 0): LineAction.TURN_LEFT_HARD,
        (0, 0, 0): LineAction.INTERSECTION,
        (1, 1, 1): LineAction.LINE_LOST,
    }

    def __init__(self,
                 pin_left:   int = PIN_LINE_LEFT,
                 pin_middle: int = PIN_LINE_MIDDLE,
                 pin_right:  int = PIN_LINE_RIGHT):
        self._log    = get_logger("LINE_HW")
        self._left   = InputDevice(pin=pin_left)
        self._middle = InputDevice(pin=pin_middle)
        self._right  = InputDevice(pin=pin_right)
        self._log.info("Capteurs de ligne initialisés (L=%d, M=%d, R=%d)",
                       pin_left, pin_middle, pin_right)

    def read(self) -> tuple[int, int, int]:
        """Retourne l'état brut instantané (gauche, milieu, droite)."""
        return self._left.value, self._middle.value, self._right.value

    def read_action(self) -> LineAction:
        """
        Méthode d'instance pratique.
        Lit les capteurs matériels et renvoie directement l'action décodée.
        """
        left, middle, right = self.read()
        return self.decode(left, middle, right)

    @staticmethod
    def decode(left: int, middle: int, right: int) -> LineAction:
        """Traduit les 3 valeurs binaires en une action de conduite."""
        pattern = (left, middle, right)
        return LineTracker.TRUTH_TABLE.get(pattern, LineAction.LINE_LOST)


# ═══════════════════════════════════════════════════════════════════
#  HARDWARE — SERVO-MOTEUR
# ═══════════════════════════════════════════════════════════════════

class ServoMotor:
    """Contrôle d'un servo PCA9685 avec suivi interne de l'angle (float)."""

    def __init__(self,
                 pca_instance:   PCA9685,
                 channel:        int,
                 angle_min:      float = 10.0,
                 angle_max:      float = 170.0,
                 default_angle:  float = 90.0,
                 name:           str   = "SERVO"):

        self._log          = get_logger(name)
        self._angle_min    = float(angle_min)
        self._angle_max    = float(angle_max)
        self._default      = float(default_angle)
        self._angle_f      = float(default_angle)   # état interne flottant

        self._hw = adafruit_servo.Servo(
            pca_instance.channels[channel],
            min_pulse=SERVO_MIN_PULSE_US,
            max_pulse=SERVO_MAX_PULSE_US,
            actuation_range=SERVO_RANGE_DEG
        )

        self._write_hw(self._default)
        time.sleep(SERVO_INIT_DELAY_S)
        self._log.info("Initialisé canal %d → %.1f° (min=%.1f°, max=%.1f°)",
                       channel, self._default, angle_min, angle_max)

    @property
    def angle(self) -> float:
        return self._angle_f

    def _clamp(self, value: float) -> float:
        return max(self._angle_min, min(self._angle_max, value))

    def _write_hw(self, angle: float) -> None:
        clamped = self._clamp(angle)
        self._hw.angle = clamped
        self._angle_f  = clamped

    def set_angle(self, angle: float) -> None:
        self._write_hw(angle)

    def move_to(self,
                target:     float,
                step:       float = SERVO_STEP_DEG,
                step_delay: float = SERVO_STEP_DELAY_S) -> None:
        target = self._clamp(target)
        if abs(target - self._angle_f) < step:
            self._write_hw(target)
            return

        direction = 1.0 if target > self._angle_f else -1.0
        while abs(target - self._angle_f) > step:
            self._write_hw(self._angle_f + direction * step)
            time.sleep(step_delay)

        self._write_hw(target)  # snap final précis

    def center(self) -> None:
        self.move_to(self._default)
        time.sleep(0.3)

    def reset(self) -> None:
        self._write_hw(self._default)
        self._log.debug("Reset → %.1f°", self._default)


# ═══════════════════════════════════════════════════════════════════
#  HARDWARE — MOTEUR DC
# ═══════════════════════════════════════════════════════════════════

class DCMotor:
    """Moteur de traction arrière via PCA9685 + adafruit_motor."""

    def __init__(self, pca_instance: PCA9685):
        self._log       = get_logger("MOTOR")
        self._motor     = motor.DCMotor(
            pca_instance.channels[CHANNEL_MOTOR_IN1],
            pca_instance.channels[CHANNEL_MOTOR_IN2]
        )
        self._motor.decay_mode = motor.SLOW_DECAY

        self._speed     = 0.0
        self._direction = Direction.FORWARD
        self._stopped   = True

        self._stop_immediate()
        self._log.info("Moteur DC initialisé (IN1=%d, IN2=%d)",
                       CHANNEL_MOTOR_IN1, CHANNEL_MOTOR_IN2)

    @staticmethod
    def _pct_to_throttle(speed_pct: float) -> float:
        return max(0.0, min(1.0, speed_pct / 100.0))

    def _apply_power(self, direction: Direction, speed_pct: float) -> None:
        throttle = self._pct_to_throttle(speed_pct)
        if direction == Direction.BACKWARD:
            throttle = -throttle
        self._motor.throttle = throttle

    def _ramp(self, target_speed: float, ramp_time: float = MOTOR_RAMP_MAX_TIME_S) -> None:
        ramp_time = min(ramp_time, MOTOR_RAMP_MAX_TIME_S)
        steps     = max(1, round(ramp_time * MOTOR_RAMP_STEPS))
        delta     = (target_speed - self._speed) / steps

        for _ in range(steps):
            self._speed += delta
            self._apply_power(self._direction, self._speed)
            time.sleep(1.0 / MOTOR_RAMP_STEPS)

        self._speed = target_speed
        self._apply_power(self._direction, self._speed)

    def _stop_immediate(self) -> None:
        self._speed            = 0.0
        self._motor.throttle   = 0
        self._stopped          = True

    def stop(self) -> None:
        self._ramp(0.0, ramp_time=0.2)
        self._stop_immediate()
        self._log.debug("Stop")

    def reset(self) -> None:
        self._stop_immediate()
        self._log.debug("Reset")

    def drive(self,
              direction:   Direction = Direction.FORWARD,
              speed_pct:   float     = SPEED_NORMAL_PCT,
              duration_s:  Optional[float] = None,
              slow:        bool      = False) -> None:
        if direction != self._direction:
            self.stop()
        self._direction = direction
        self._stopped   = False

        if slow:
            self._apply_power(self._direction, 25.0)
        elif duration_s is not None and duration_s < 1.0:
            self._ramp(speed_pct, ramp_time=duration_s)
            self.stop()
        else:
            self._ramp(speed_pct)


# ═══════════════════════════════════════════════════════════════════
#  HARDWARE — TÊTE (PAN / TILT / ROUE)
# ═══════════════════════════════════════════════════════════════════

class Head:
    """Tête du robot : 3 servos couplés montés sur PCA9685."""

    def __init__(self, pca_instance: PCA9685):
        self._log = get_logger("HEAD")

        self.wheel      = ServoMotor(pca_instance, CHANNEL_SERVO_WHEEL,
                                     WHEEL_ANGLE_MIN, WHEEL_ANGLE_MAX,
                                     WHEEL_ANGLE_CENTER, "WHEEL")
        self.horizontal = ServoMotor(pca_instance, CHANNEL_SERVO_HORIZONTAL,
                                     HEAD_ANGLE_MIN, HEAD_ANGLE_MAX,
                                     HEAD_ANGLE_CENTER, "PAN")
        self.vertical   = ServoMotor(pca_instance, CHANNEL_SERVO_VERTICAL,
                                     HEAD_ANGLE_MIN, HEAD_ANGLE_MAX,
                                     HEAD_ANGLE_CENTER, "TILT")

    def steer(self, angle: float) -> None:
        wheel_angle = max(WHEEL_ANGLE_MIN, min(WHEEL_ANGLE_MAX, angle))
        head_angle  = max(HEAD_ANGLE_MIN,  min(HEAD_ANGLE_MAX,  angle))
        self.wheel.set_angle(wheel_angle)
        self.horizontal.set_angle(head_angle)
        self._log.debug("steer → roue=%.1f° pan=%.1f°", wheel_angle, head_angle)

    def steer_left(self, intensity: float = STEER_SOFT_DEG) -> None:
        self.steer(WHEEL_ANGLE_CENTER - intensity)

    def steer_right(self, intensity: float = STEER_SOFT_DEG) -> None:
        self.steer(WHEEL_ANGLE_CENTER + intensity)

    def steer_center(self) -> None:
        self.steer(WHEEL_ANGLE_CENTER)

    def reset(self) -> None:
        self.wheel.reset()
        self.horizontal.reset()
        self.vertical.reset()
        self._log.info("Reset → tous les servos au centre")

    def shutdown(self) -> None:
        self._log.info("Shutdown — recentrage des servos…")
        self.horizontal.center()
        self.vertical.center()
        self.wheel.center()
        self.reset()


# ═══════════════════════════════════════════════════════════════════
#  ROBOT — CLASSE DE HAUT NIVEAU
# ═══════════════════════════════════════════════════════════════════

class Robot:
    """Façade unique rassemblant tous les composants matériels."""

    def __init__(self, cfg: argparse.Namespace):
        self._log = get_logger("ROBOT")
        self._cfg = cfg

        self._log.info("Initialisation du bus I²C et PCA9685 (addr=0x%02X)…", PCA_ADDRESS)
        self._i2c = busio.I2C(SCL, SDA)
        self._pca = PCA9685(self._i2c, address=PCA_ADDRESS)
        self._pca.frequency = PCA_FREQUENCY_HZ

        self.ultrasonic   = UltrasonicSensor(cfg.us_trigger, cfg.us_echo)
        self.line_tracker = LineTracker(cfg.line_left, cfg.line_mid, cfg.line_right)
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

def thread_ultrasonic(robot: Robot, interval: float) -> None:
    """Lit le capteur ultrason en boucle et met à jour RobotState."""
    log = get_logger("US")
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
    log = get_logger("LINE")
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


def thread_controller(robot: Robot, interval: float) -> None:
    """
    Boucle de décision : lit l'action synthétisée, décide et pilote les moteurs.
    """
    log  = get_logger("CTRL")
    log.info("Thread démarré (intervalle=%.3f s)", interval)

    last_action: Optional[LineAction] = None

    while True:
        # ── Lecture atomique de l'état simplifié ──────────────────
        with robot.state.lock:
            if not robot.state.running:
                break
            emergency = robot.state.emergency_stop
            action    = robot.state.line_action

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
            last_action = action

        if action == LineAction.STRAIGHT:
            robot.head.steer_center()
            robot.motor.drive(Direction.FORWARD, SPEED_NORMAL_PCT)

        elif action == LineAction.TURN_LEFT_SOFT:
            robot.head.steer_left(STEER_SOFT_DEG)
            robot.motor.drive(Direction.FORWARD, SPEED_TURNING_PCT)

        elif action == LineAction.TURN_RIGHT_SOFT:
            robot.head.steer_right(STEER_SOFT_DEG)
            robot.motor.drive(Direction.FORWARD, SPEED_TURNING_PCT)

        elif action == LineAction.TURN_LEFT_HARD:
            robot.head.steer_left(STEER_HARD_DEG)
            robot.motor.drive(Direction.FORWARD, SPEED_SLOW_PCT)

        elif action == LineAction.TURN_RIGHT_HARD:
            robot.head.steer_right(STEER_HARD_DEG)
            robot.motor.drive(Direction.FORWARD, SPEED_SLOW_PCT)

        elif action == LineAction.INTERSECTION:
            robot.head.steer_center()
            robot.motor.drive(Direction.FORWARD, SPEED_NORMAL_PCT)
            log.info("Intersection détectée — passage tout droit")

        else:  # LineAction.LINE_LOST
            robot.motor.stop()
            robot.head.steer_center()
            log.warning("Ligne perdue — recherche active / attente…")

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
        description="Robot Line Follower — Team C — MasterCamp SE 2026",
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

    log = get_logger("MAIN")
    log.info("╔══════════════════════════════════════════════╗")
    log.info("║  Robot Line Follower — Team C — SE 2026      ║")
    log.info("╚══════════════════════════════════════════════╝")

    robot = Robot(args)
    robot.init()

    threads = [
        threading.Thread(target=thread_ultrasonic, args=(robot, args.sensor_interval), name="US", daemon=True),
        threading.Thread(target=thread_line, args=(robot, args.sensor_interval), name="LINE", daemon=True),
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
