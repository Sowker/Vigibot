"""
Script de debug pour les capteurs infrarouges (3 capteurs).
Affiche les valeurs brutes et l'action décodée.

Usage:
  sudo python3 work/test_ir_raw.py --left 22 --mid 27 --right 17 --interval 0.2 --pull-up

Couvrez un capteur avec le doigt: les valeurs correspondantes doivent changer.
"""

import time
import argparse

try:
    from gpiozero import InputDevice
except Exception as e:
    print("Erreur d'import gpiozero:", e)
    raise


def parse_args():
    p = argparse.ArgumentParser(description="Test capteurs IR")
    p.add_argument('--left', type=int, default=22)
    p.add_argument('--mid', type=int, default=27)
    p.add_argument('--right', type=int, default=17)
    p.add_argument('--interval', type=float, default=0.2)
    p.add_argument('--pull-up', action='store_true', help='Activer pull_up sur les entrées')
    p.add_argument('--invert', action='store_true', help='Inverser la logique 0/1 (utile si capteur actif-low)')
    return p.parse_args()


def main():
    args = parse_args()

    # Certains environnements de gpiozero acceptent pull_up, d'autres non;
    # gérer cela proprement pour compatibilité.
    def make_device(pin):
        try:
            return InputDevice(pin, pull_up=args.pull_up)
        except TypeError:
            # ancienne version: InputDevice(pin)
            return InputDevice(pin)

    left = make_device(args.left)
    mid = make_device(args.mid)
    right = make_device(args.right)

    print(f"Test IR: L={args.left} M={args.mid} R={args.right} | pull_up={args.pull_up} | invert={args.invert}")
    print("Couvrez un capteur ou approchez une bande noire — observez les changements")

    try:
        while True:
            l = int(left.value)
            m = int(mid.value)
            r = int(right.value)

            if args.invert:
                l, m, r = 1 - l, 1 - m, 1 - r

            pattern = (l, m, r)

            # Affichage compact
            print(f"(G:{l} M:{m} D:{r}) -> pattern={pattern}", end='\r')

            time.sleep(args.interval)

    except KeyboardInterrupt:
        print('\nFin du test')


if __name__ == '__main__':
    main()
