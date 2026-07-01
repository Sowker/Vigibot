import time
from t11_argument_parser import parse_args
from t11_robot import Robot
from t4_dc_motor import Direction

# ── PARAMÈTRES DU ROBOT ──
SPEED_CRUISE = 40
SPEED_BACKUP = 35     # Vitesse réduite pour le recul de sécurité
STEER_CENTER = 90
THRESHOLD_STOP = 350
THRESHOLD_BACKUP = 200 # En mm : distance sous laquelle le robot va reculer
BACKUP_DURATION = 0.6  # Temps de recul en secondes
SCAN_START, SCAN_END, SCAN_STEP = 50, 130, 5
MOVE_DURATION = 0.5  # Durée de chaque impulsion d'avance
PAUSE_DURATION = 0.3 # Temps d'arrêt entre deux impulsions

def run_autonomy(robot: Robot):
    while True:
        with robot.state.lock:
            if not robot.state.running: 
                break
            
        # 1. SCAN (Balayage de 50 à 130° avec le moteur 1)
        dists = []
        angles = list(range(SCAN_START, SCAN_END + 1, SCAN_STEP))
        
        for angle in angles:
            robot.head.set_angle_motor(1, angle)
            time.sleep(0.05)
            dists.append(robot.ultrasonic.read_mm())
            
        # Repositionner le capteur au centre après le scan
        robot.head.set_angle_motor(1, STEER_CENTER)
        
        # Filtrer les fausses lectures (0)
        valid_dists = [d for d in dists if d > 0]
        min_dist = min(valid_dists) if valid_dists else 9999
        
        # 2. DÉCISION
        if min_dist > THRESHOLD_STOP:
            # ── IMPULSION D'AVANCE EN LIGNE DROITE ──
            robot.head.set_angle_motor(0, STEER_CENTER) # Roues droites (moteur 0)
            robot.motor.drive(Direction.FORWARD, SPEED_CRUISE, fast_accel=True)
            time.sleep(MOVE_DURATION)
            robot.motor.reset()
            
            # PAUSE D'OBSERVATION
            time.sleep(PAUSE_DURATION)
        else:
            # ── MANŒUVRE D'ÉVITEMENT AVEC SÉCURITÉ RECUL ──
            robot.motor.reset() # Arrêt d'urgence immédiat
            
            # Action bonus : Si trop proche, reculer d'abord
            if min_dist < THRESHOLD_BACKUP:
                print(f"⚠️ Obstacle critique à {min_dist}mm ! Recul de sécurité avant le S.")
                robot.head.set_angle_motor(0, STEER_CENTER) # Roues droites pour reculer proprement
                robot.motor.drive(Direction.BACKWARD, SPEED_BACKUP, fast_accel=True)
                time.sleep(BACKUP_DURATION)
                robot.motor.reset()
                time.sleep(0.1) # Légère pause pour stabiliser le châssis
            
            # Détermination de la direction d'évitement
            min_idx = dists.index(min_dist) if min_dist in dists else 0
            direction = -1 if min_idx > (len(dists) / 2) else 1
            
            # Allumage des moteurs de propulsion pour entamer le S
            robot.motor.drive(Direction.FORWARD, SPEED_CRUISE, fast_accel=True)
            
            # Séquence en S
            robot.head.set_angle_motor(0, STEER_CENTER + (direction * 45))
            time.sleep(0.8)
            
            robot.head.set_angle_motor(0, STEER_CENTER)
            time.sleep(0.2)
            
            robot.head.set_angle_motor(0, STEER_CENTER - (direction * 45))
            time.sleep(1.2)
            
            robot.head.set_angle_motor(0, STEER_CENTER + (direction * 45))
            time.sleep(0.5)
            
            robot.head.set_angle_motor(0, STEER_CENTER)
            time.sleep(0)
            
            # Fin de la manœuvre
            robot.motor.reset()

# ── CORPS PRINCIPAL ──
if __name__ == "__main__":
    args = parse_args()
    robot = Robot(args)
    robot.init()
    
    try:
        # Exécution directe sur le thread principal
        run_autonomy(robot)
    except KeyboardInterrupt:
        print("\nArrêt du robot...")
    finally:
        with robot.state.lock:
            robot.state.running = False
        robot.motor.reset()
        robot.shutdown()