import cv2
import numpy as np
import time
import logger 
from t11_robot import Robot

cap = cv2.VideoCapture(0) 
def thread_arrow(robot,interval) : 
    log = logger.get_logger("ARROW") 
    while True : 
        with robot.state.lock : 
            if not robot.state.running : 
                break
        ret, frame = cap.read()
        if not ret : 
            time.sleep(interval)
            continue

        #Détection de la fleche a faire
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5,5),0)
        _, thresh = cv2.threshold(blurred, 100, 255, cv2.THRESH_BINARY_INV) 
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) 
        contours = [c for c in contours if cv2.contourArea(c) > 500] 
        if not contours : 
            time.sleep(interval) 
            continue 

        arrow = max(contours, key=cv2.contourArea)
        M = cv2.moments(arrow)

        if M["m00"] == 0: 
            continue
        cx = int(M["m10"] / M["m00"]) 

        leftmost = tuple(arrow[arrow[:, :, 0].argmin()][0])
        rightmost = tuple(arrow[arrow[:, :, 0].argmax()][0])

        dist_left = cx - leftmost[0] 
        dist_right = rightmost[0] - cx 

        if dist_right > dist_left : 
            direction = 1
        elif dist_left > dist_right : 
            direction = -1
        else : 
            direction = 0

        with robot.state.lock : 
            robot.state.arrow_direction = direction
        log.info("Direciton flèche : %s", {1 : "droite", -1 : "gauche", 0 : "inconnu"}[direction])

        #retour video pour tests
        cv2.imshow("Seuillage",thresh)
        cv2.waitKey(1)

        time.sleep(interval)
    cap.release()
    cv2.destroyAllWindows()
    log.info("Thread arrêté") 
    
