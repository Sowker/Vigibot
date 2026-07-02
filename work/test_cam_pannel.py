import cv2
import numpy as np
import time
from pannel_test import get_color_mask

# ── PC : webcam ──────────────────────────────────────────────────────
cap = cv2.VideoCapture(0)

# ── Pi : remplace les 2 lignes ci-dessus par : ───────────────────────
# from picamera2 import Picamera2
# picam = Picamera2()
# picam.configure(picam.create_preview_configuration(main={"format":"RGB888","size":(640,480)}))
# picam.start()

last_print_t = 0
last_panneau = None

try:
    while True:
        # ── PC ──────────────────────────────────────────────────────
        ret, frame = cap.read()
        if not ret:
            break
        # ── Pi : remplace les 2 lignes ci-dessus par : ──────────────
        # frame = picam.capture_array()
        # frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        h, w  = frame.shape[:2]
        hsv   = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        masks = get_color_mask(hsv)

        panneau_detecte = None

        for couleur, mask in masks.items():
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            contours = [c for c in contours if cv2.contourArea(c) > 500]
            if not contours:
                continue

            c     = max(contours, key=cv2.contourArea)
            ratio = cv2.contourArea(c) / (h * w)

            epsilon = 0.04 * cv2.arcLength(c, True)
            approx  = cv2.approxPolyDP(c, epsilon, True)
            n       = len(approx)

            forme = "triangle" if n == 3 else "rectangle" if n == 4 else f"{n} sommets"

            if   couleur == "bleu"  and n == 4: panneau = "TUNNEL"
            elif couleur == "jaune" and n == 3: panneau = "TRAVAUX"
            else:                               panneau = None

            if panneau:
                panneau_detecte = panneau

            # Dessin sur la frame
            color_draw = (255, 100, 0) if couleur == "bleu" else (0, 200, 255)
            cv2.drawContours(frame, [c], -1, color_draw, 2)
            label = f"{couleur} | {forme} | {ratio*100:.1f}%"
            if panneau:
                label += f"  → {panneau}"
            x, y, _, _ = cv2.boundingRect(c)
            cv2.putText(frame, label, (x, max(y - 8, 15)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_draw, 2)

        # Résultat en grand en haut de la frame
        if panneau_detecte:
            cv2.putText(frame, panneau_detecte, (20, 45),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.4, (0, 255, 0), 3)

        # Console : affiche seulement si ça change, max 1 fois/sec
        now = time.monotonic()
        if panneau_detecte != last_panneau or now - last_print_t > 1.0:
            print(f"→ {panneau_detecte or 'rien'}")
            last_panneau = panneau_detecte
            last_print_t = now

        cv2.imshow("Camera",       frame)
        cv2.imshow("Masque bleu",  masks["bleu"])
        cv2.imshow("Masque jaune", masks["jaune"])

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    pass

cap.release()
cv2.destroyAllWindows()
