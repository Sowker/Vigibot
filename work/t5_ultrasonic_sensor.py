from gpiozero import DistanceSensor

import logger as log

# ── Constantes ──────────────────────────────────────────────────
PIN_ULTRASONIC_TRIGGER = 23
PIN_ULTRASONIC_ECHO    = 24

STEER_SOFT_DEG        = 15   # degrés de braquage — correction légère
STEER_HARD_DEG        = 35   # degrés de braquage — virage serré

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
        self._log = log.get_logger("US_HW")
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