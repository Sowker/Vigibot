import cv2
import numpy as np
from picamera2 import Picamera2
from pannel_test import get_color_mask

picam = Picamera2()
picam.configure(picam.create_preview_configuration(main={"format": "RGB888", "size": (640, 480)}))
picam.start()

try:
    while True:
        frame = picam.capture_array()
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        h, w  = frame.shape[:2]
        hsv   = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        masks = get_color_mask(hsv)

        found = False
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

            print(f"couleur={couleur:6s}  forme={forme:12s}  occupation={ratio*100:.1f}%  → {panneau or '---'}")

            color_draw = (255, 100, 0) if couleur == "bleu" else (0, 200, 255)
            cv2.drawContours(frame, [c], -1, color_draw, 2)
            label = f"{couleur} | {forme} | {ratio*100:.1f}%"
            if panneau:
                label += f" → {panneau}"
            x, y, _, _ = cv2.boundingRect(c)
            cv2.putText(frame, label, (x, y - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color_draw, 2)
            found = True

        if not found:
            print("---  rien détecté")

        cv2.imwrite("/tmp/pannel_frame.jpg", frame)
        cv2.imwrite("/tmp/pannel_bleu.jpg",  masks["bleu"])
        cv2.imwrite("/tmp/pannel_jaune.jpg", masks["jaune"])

except KeyboardInterrupt:
    pass

picam.stop()
print("Arrêté.")
