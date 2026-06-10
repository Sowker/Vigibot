import argparse
import time
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
#  MATÉRIEL — CAPTEURS DE LIGNE
# ═══════════════════════════════════════════════════════════════════

class LineTracker:
    """
    Barrette de 3 capteurs infrarouges.
    Convention : 0 = ligne noire détectée, 1 = sol clair.
    """

    # Attribut de classe : partagé, alloué une seule fois en mémoire
    TRUTH_TABLE = {
        (1, 0, 1): LineAction.STRAIGHT,
        (0, 1, 1): LineAction.TURN_LEFT_SOFT,
        (1, 1, 0): LineAction.TURN_RIGHT_SOFT,
        (0, 0, 1): LineAction.TURN_LEFT_HARD,
        (1, 0, 0): LineAction.TURN_RIGHT_HARD,
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


def parse_arguments() -> argparse.Namespace:
    """Gère l'analyse des arguments de la ligne de commande pour le suivi de ligne.

    Retourne:
        argparse.Namespace: Les arguments de la ligne de commande analysés.
    """
    parser = argparse.ArgumentParser(
        description="Script d'automatisation du suivi de ligne pour barrette de capteurs infrarouges (0 = Ligne noire)."
    )
    parser.add_argument('--left', type=int, default=22, help="Broche GPIO pour le capteur Gauche (par défaut : 22)")
    parser.add_argument('--middle', type=int, default=27, help="Broche GPIO pour le capteur Milieu (par défaut : 27)")
    parser.add_argument('--right', type=int, default=17, help="Broche GPIO pour le capteur Droit (par défaut : 17)")
    return parser.parse_args()


if __name__ == '__main__':
    args = parse_arguments()
    line_tracking = LineTracker(pin_left=args.left, pin_middle=args.middle, pin_right=args.right)

    print("Démarrage du système de suivi de ligne... Appuyez sur Ctrl+C pour arrêter.\n")

    try:
        while True:
            status_left, status_middle, status_right = line_tracking.read()

            # Détermination de l'action basée sur les lectures des capteurs
            if status_left == 1 and status_middle == 0 and status_right == 1:
                action = "Tout droit"

            elif status_left == 0 and status_middle == 1 and status_right == 1:
                action = "Tourner à gauche ←"

            elif status_left == 1 and status_middle == 1 and status_right == 0:
                action = "Tourner à droite →"

            elif status_left == 0 and status_middle == 0 and status_right == 1:
                action = "Légèrement à gauche (Gauche + Milieu)"

            elif status_left == 1 and status_middle == 0 and status_right == 0:
                action = "Légèrement à droite (Milieu + Droite)"

            elif status_left == 0 and status_middle == 0 and status_right == 0:
                action = "Intersection / Croisement"

            else:
                action = "En attente de la ligne / Perdu..."

            # Affichage des données brutes et de l'action sur une seule ligne
            raw_data = f"(G:{status_left} M:{status_middle} D:{status_right})"
            print(f"{raw_data:<14} -> {action}")

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nProgramme interrompu. Au revoir !")
        print("Programme développé par l'Équipe C - MasterCamp SE 2026.")