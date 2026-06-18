import cv2
import numpy as np
import time
import logger
# from t11_robot import Robot
from picamera2 import Picamera2

picam = Picamera2()
picam.configure(picam.create_preview_configuration(
    main={"format": "RGB888", "size": (640, 480)}
))
picam.start()


def _detect_direction(thresh):
    """Détecte la direction de la flèche dans la zone centrale de l'image.
    Retourne 1 (droite), -1 (gauche) ou 0 (inconnu).
    """
    h, w = thresh.shape[:2]

    # ROI : quart central de l'image — ignore l'environnement autour
    roi_x1, roi_x2 = w // 4, 3 * w // 4
    roi_y1, roi_y2 = h // 4, 3 * h // 4
    thresh_roi = thresh[roi_y1:roi_y2, roi_x1:roi_x2]

    contours_roi, _ = cv2.findContours(thresh_roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Décaler les contours en coordonnées image complète + filtrer bruit et fond
    contours = []
    for c in contours_roi:
        c_shifted = c + np.array([roi_x1, roi_y1])
        if 2000 < cv2.contourArea(c_shifted) < (h * w * 0.5):
            contours.append(c_shifted)

    if not contours:
        return 0

    arrow = max(contours, key=cv2.contourArea)
    bx, _, bw, _ = cv2.boundingRect(arrow)
    bbox_cx = bx + bw // 2

    # Masque rempli : compter pixels gauche vs droite
    mask = np.zeros(thresh.shape, dtype=np.uint8)
    cv2.drawContours(mask, [arrow], -1, 255, thickness=cv2.FILLED)
    left_pixels  = cv2.countNonZero(mask[:, bx:bbox_cx])
    right_pixels = cv2.countNonZero(mask[:, bbox_cx:bx + bw])

    diff = left_pixels - right_pixels
    if abs(diff) < 500:
        return 0
    return 1 if left_pixels > right_pixels else -1


def thread_arrow(robot, interval):
    log = logger.get_logger("ARROW")
    while True:
        with robot.state.lock:
            if not robot.state.running:
                break
        frame = picam.capture_array()
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        gray    = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        _, thresh = cv2.threshold(blurred, 100, 255, cv2.THRESH_BINARY_INV)

        direction = _detect_direction(thresh)

        with robot.state.lock:
            robot.state.arrow_direction = direction
        log.info("Direction flèche : %s", {1: "droite", -1: "gauche", 0: "inconnu"}[direction])

        # Sauvegarde image pour debug (visible via scp /tmp/arrow_*.jpg)
        cv2.imwrite("/tmp/arrow_thresh.jpg", thresh)
        cv2.imwrite("/tmp/arrow_frame.jpg", frame)

        time.sleep(interval)

    picam.stop()
    log.info("Thread arrêté")


if __name__ == "__main__":
    print("Démarrage — Ctrl+C pour arrêter.")
    labels = {1: "DROITE ->", -1: "<- GAUCHE", 0: "inconnu"}
    last_dir = None
    try:
        while True:
            frame = picam.capture_array()
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            gray    = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(gray, (5, 5), 0)
            _, thresh = cv2.threshold(blurred, 100, 255, cv2.THRESH_BINARY_INV)

            direction = _detect_direction(thresh)

            cv2.imwrite("/tmp/arrow_frame.jpg", frame)
            cv2.imwrite("/tmp/arrow_thresh.jpg", thresh)

            if direction != last_dir:
                print(f"Direction : {labels[direction]}")
                last_dir = direction

            time.sleep(0.05)
    except KeyboardInterrupt:
        pass

    picam.stop()
    print("Arrêté.")
