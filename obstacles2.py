import time
import threading
from t11_argument_parser import parse_args
from t11_robot import Robot
from t4_dc_motor import Direction

# ── PARAMÈTRES DU ROBOT ──
SPEED_CRUISE = 40
SPEED_BACKUP = 35
STEER_CENTER = 90
THRESHOLD_STOP = 350
THRESHOLD_BACKUP = 180
THRESHOLD_CRITICAL = 100 # 10 cm : Sécurité absolue pendant le S
SCAN_START, SCAN_END, SCAN_STEP = 50, 130, 5
MOVE_DURATION = 0.5  
PAUSE_DURATION = 0.3 
BACKUP_DURATION = 0.6

# ── SÉCURITÉS ET THREADS ──
# Ce drapeau coupe TOUS les mouvements du robot instantanément si l'ultrason voit un danger proche
safety_abort = threading.Event()

state_lock = threading.Lock()
shared_min_dist = 9999
shared_direction = 1  

# ── THREAD 1 : MONITORING ULTRASON (AVEC RECONNAISSANCE DES CRASHS) ──
def thread_ultrasonic(robot: Robot):
    global shared_min_dist, shared_direction
    print("👁️ Thread Ultrason : Actif")
    head_angle = STEER_CENTER
    head_direction = 1
    
    while robot.state.running:
        robot.head.set_angle_motor(1, head_angle)
        time.sleep(0.05)
        
        dist = robot.ultrasonic.read_mm()
        if dist > 0:
            with state_lock:
                shared_min_dist = dist
                if dist < THRESHOLD_STOP:
                    # Détermine si l'obstacle est plutôt à gauche ou à droite
                    shared_direction = -1 if head_angle > STEER_CENTER else 1
            
            # 🚨 SÉCURITÉ CRITIQUE : Si un obstacle surgit à < 10cm pendant qu'on roule
            if dist < THRESHOLD_CRITICAL:
                safety_abort.set()
        
        # Balayage de la tête
        head_angle += head_direction * 15
        if head_angle >= SCAN_END:
            head_angle = SCAN_END
            head_direction = -1
        elif head_angle <= SCAN_START:
            head_angle = SCAN_START
            head_direction = 1

# ── THREAD 2 (PRINCIPAL) : LOGIQUE DE TRAJECTOIRE ──
def smart_sleep(duration_target: float) -> bool:
    """
    Attend le temps imparti. Coupe court instantanément si 'safety_abort' est activé.
    Retourne True si une urgence est survenue, False si le temps est écoulé normalement.
    """
    return safety_abort.wait(timeout=duration_target)

def handle_emergency(robot: Robot):
    """Gère le recul d'urgence en cas d'approche critique d'un obstacle."""
    with state_lock:
        current_dist = shared_min_dist

    print(f"🚨 Obstacle trop proche ({current_dist}mm) ! Recul d'urgence.")
    robot.motor.reset()
    robot.head.set_angle_motor(0, STEER_CENTER)
    
    # Recul de dégagement roues droites
    robot.motor.drive(Direction.BACKWARD, SPEED_BACKUP, fast_accel=True)
    time.sleep(0.4)

    # Reset des moteurs et du drapeau d'alerte
    robot.motor.reset()
    safety_abort.clear()

def run_autonomy(robot: Robot):
    print("🚀 Boucle de mouvement démarrée (Mode Ultrason Seul).")
    
    while robot.state.running:
        if safety_abort.is_set():
            handle_emergency(robot)
            continue
            
        with state_lock:
            current_dist = shared_min_dist
            evt_direction = shared_direction
            
        # ── INTERPRÉTATION DES DONNÉES ──
        if current_dist > THRESHOLD_STOP:
            # Ligne droite nominale
            robot.head.set_angle_motor(0, STEER_CENTER)
            robot.motor.drive(Direction.FORWARD, SPEED_CRUISE, fast_accel=True)
            
            if smart_sleep(MOVE_DURATION):
                handle_emergency(robot)
                continue
                
            robot.motor.reset()
            smart_sleep(PAUSE_DURATION)
            
        else:
            # Évitement en S
            robot.motor.reset()
            
            # Recul préparatoire si le robot s'est arrêté trop près avant d'engager le S
            if current_dist < THRESHOLD_BACKUP:
                robot.head.set_angle_motor(0, STEER_CENTER)
                robot.motor.drive(Direction.BACKWARD, SPEED_BACKUP, fast_accel=True)
                if smart_sleep(BACKUP_DURATION):
                    handle_emergency(robot)
                    continue
                robot.motor.reset()
            
            # Séquence de contournement en S
            steps = [
                (STEER_CENTER + (evt_direction * 45), 0.9),
                (STEER_CENTER, 0.2),
                (STEER_CENTER - (evt_direction * 45), 1.3),
                (STEER_CENTER + (evt_direction * 45), 0.5)
            ]
            
            robot.motor.drive(Direction.FORWARD, SPEED_CRUISE, fast_accel=True)
            
            s_interrupted = False
            for steer, step_duration in steps:
                robot.head.set_angle_motor(0, steer)
                
                # Le smart_sleep surveille la distance critique en continu pendant les virages
                if smart_sleep(step_duration):
                    s_interrupted = True
                    break
            
            robot.motor.reset()
            robot.head.set_angle_motor(0, STEER_CENTER)
            
            if s_interrupted:
                handle_emergency(robot)

# ── DISPATCHING DES THREADS ──
if __name__ == "__main__":
    args = parse_args()
    robot = Robot(args)
    robot.init()
    
    # On ne lance que le thread de l'œil ultrason
    t_ultra = threading.Thread(target=thread_ultrasonic, args=(robot,), daemon=True)
    t_ultra.start()
    
    try:
        run_autonomy(robot)
    except KeyboardInterrupt:
        print("\nArrêt du robot...")
    finally:
        with robot.state.lock:
            robot.state.running = False
        robot.motor.reset()
        robot.shutdown()