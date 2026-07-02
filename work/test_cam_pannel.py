import cv2
import numpy as np
from pannel_test import get_color_mask

cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    h, w = frame.shape[:2]
    hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    masks = get_color_mask(hsv)

    found = False
    for couleur, mask in masks.items():
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = [c for c in contours if cv2.contourArea(c) > 500]
        if not contours:
            continue

        c     = max(contours, key=cv2.contourArea)
        area  = cv2.contourArea(c)
        ratio = area / (h * w)

        epsilon = 0.04 * cv2.arcLength(c, True)
        approx  = cv2.approxPolyDP(c, epsilon, True)
        n       = len(approx)

        forme = "triangle" if n == 3 else "rectangle" if n == 4 else f"{n} sommets"

        # Détermine le panneau détecté
        if   couleur == "bleu"  and n == 4: panneau = "TUNNEL"
        elif couleur == "jaune" and n == 3: panneau = "TRAVAUX"
        else:                               panneau = None

        # Affichage console
        print(f"couleur={couleur:6s}  forme={forme:12s}  occupation={ratio*100:.1f}%  → {panneau or '---'}")

        # Dessin sur la frame
        color_draw = (255, 100, 0) if couleur == "bleu" else (0, 200, 255)
        cv2.drawContours(frame, [c], -1, color_draw, 2)
        label = f"{couleur} | {forme} | {ratio*100:.1f}%"
        if panneau:
            label += f" → {panneau}"
        x, y, _, _ = cv2.boundingRect(c)
        cv2.putText(frame, label, (x, y - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    color_draw, 2)
        found = True

    if not found:
        print("---  rien détecté")

    cv2.imshow("Camera", frame)
    cv2.imshow("Masque bleu",  masks["bleu"])
    cv2.imshow("Masque jaune", masks["jaune"])

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
