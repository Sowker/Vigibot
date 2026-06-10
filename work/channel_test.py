import time
from t3_servomotors import SERVO_PCA, ServoMotor

# Bouge un seul servo a la fois (canaux 0, 1, 2) pour identifier
# physiquement quel canal correspond a quel composant.
# Amplitude reduite (75-105) pour rester dans la plage mecanique de tous les servos.

CHANNELS_TO_TEST = [0, 1, 2]

print("Test canal par canal — observe quel composant bouge a chaque etape\n")

for ch in CHANNELS_TO_TEST:
    print(f"--- Canal {ch} ---")
    s = ServoMotor(SERVO_PCA, ch, angle_min=60, angle_max=120, default_angle=90, name=f"CH{ch}")
    time.sleep(0.5)

    print("  -> 75°")
    s.set_angle(75)
    time.sleep(1)

    print("  -> 105°")
    s.set_angle(105)
    time.sleep(1)

    print("  -> retour 90°")
    s.set_angle(90)
    time.sleep(1)

    input("  Note ce qui a bouge, puis Entree pour le canal suivant...\n")

print("Test termine.")
