from enum import IntEnum
from gpiozero import InputDevice

import logger as log

class LineAction(IntEnum):
    """Actions déduites des capteurs de ligne."""
    STRAIGHT        = 0
    TURN_LEFT_SOFT  = 1
    TURN_LEFT_HARD  = 2
    TURN_RIGHT_SOFT = 3
    TURN_RIGHT_HARD = 4
    INTERSECTION    = 5
    LINE_LOST       = 6


# ── Constantes ─────────────────────────────────────────────────

PIN_LINE_LEFT          = 22
PIN_LINE_MIDDLE        = 27
PIN_LINE_RIGHT         = 17

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
        self._log    = log.get_logger("LINE_HW")
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