import os
from pathlib import Path

# Running headless: force Qt to use offscreen platform to avoid X/Qt errors
# Remove any existing setting then set to offscreen.
os.environ.pop("QT_QPA_PLATFORM", None)
os.environ["QT_QPA_PLATFORM"] = "offscreen"

import cv2
import numpy as np
import time

BLUE_LABEL = "Tunnel"
YELLOW_LABEL = "Travaux"


def apply_mask(hsv_image, lower_bound, upper_bound):
    mask = cv2.inRange(hsv_image, lower_bound, upper_bound)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask


def classify_sign(mask_blue, mask_yellow):
    blue_score = cv2.countNonZero(mask_blue)
    yellow_score = cv2.countNonZero(mask_yellow)

    if blue_score > yellow_score and blue_score > 500:
        print(f"{BLUE_LABEL} detecté")
        return f"{BLUE_LABEL} detecte"
    if yellow_score > blue_score and yellow_score > 500:
        print(f"{YELLOW_LABEL} detecté")
        return f"{YELLOW_LABEL} detecte"
    return "Aucun panneau detecte"


def add_noise(frame, sigma=10):
    noise = np.random.normal(0, sigma, frame.shape).astype(np.int16)
    noisy_frame = frame.astype(np.int16) + noise
    return np.clip(noisy_frame, 0, 255).astype(np.uint8)


if __name__ == "__main__":
    import argparse

    # Paramètres de couleur
    lower_blue = (90, 80, 40)
    upper_blue = (140, 255, 255)
    lower_yellow = (15, 80, 80)
    upper_yellow = (40, 255, 255)

    parser = argparse.ArgumentParser(description="Détection de panneaux (bleu / jaune)")
    parser.add_argument("--headless", action="store_true", help="Mode sans GUI — imprime les labels dans le terminal")
    parser.add_argument("--power-pin", type=int, default=17, help="GPIO BCM pin pour alimenter la caméra (gpiozero OutputDevice)")
    args = parser.parse_args()

    headless = args.headless
    POWER_PIN = args.power_pin

    cam_power = None
    picam = None
    camera = None

    # Detect availability of Picamera2 and gpiozero
    use_picamera2 = False
    have_gpio = False
    try:
        from picamera2 import Picamera2
        use_picamera2 = True
    except Exception:
        use_picamera2 = False

    try:
        from gpiozero import OutputDevice
        have_gpio = True
    except Exception:
        have_gpio = False

    try:
        if use_picamera2:
            if have_gpio:
                cam_power = OutputDevice(POWER_PIN, active_high=True, initial_value=False)
                print(f"Activation de la caméra via GPIO {POWER_PIN}")
                cam_power.on()
                time.sleep(1.0)  # laisser le temps à l'alimentation de se stabiliser

            picam = Picamera2()
            config = picam.create_video_configuration(main={"size": (640, 480)})
            picam.configure(config)
            picam.start()
            time.sleep(0.5)  # attente après l'initialisation

            label_prev = None
            while True:
                try:
                    frame = picam.capture_array()
                except RuntimeError as e:
                    print(f"Failed to capture frame: {e}")
                    break

                # Picamera2 retourne un tableau en RGB
                hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

                mask_blue = apply_mask(hsv_frame, lower_blue, upper_blue)
                mask_yellow = apply_mask(hsv_frame, lower_yellow, upper_yellow)

                label = classify_sign(mask_blue, mask_yellow)

                if headless:
                    if label != label_prev:
                        print(f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {label}")
                        label_prev = label
                else:
                    display_frame = frame.copy()
                    cv2.putText(
                        display_frame,
                        label,
                        (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1.0,
                        (0, 255, 0),
                        2,
                        cv2.LINE_AA,
                    )
                    # OpenCV expects BGR for display; convert if needed
                    display_bgr = cv2.cvtColor(display_frame, cv2.COLOR_RGB2BGR)
                    cv2.imshow("Panneaux", display_bgr)
                    key = cv2.waitKey(1) & 0xFF
                    if key in (27, ord("q")):
                        break

        else:
            # Fallback vers webcam OpenCV (PC ou USB webcam)
            camera = cv2.VideoCapture(0)
            if not camera.isOpened():
                raise RuntimeError("Impossible d'ouvrir la camera.")

            label_prev = None
            while True:
                success, frame = camera.read()
                if not success:
                    raise RuntimeError("Impossible de lire une image depuis la camera.")

                hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

                mask_blue = apply_mask(hsv_frame, lower_blue, upper_blue)
                mask_yellow = apply_mask(hsv_frame, lower_yellow, upper_yellow)

                label = classify_sign(mask_blue, mask_yellow)

                if headless:
                    if label != label_prev:
                        print(f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {label}")
                        label_prev = label
                else:
                    display_frame = frame.copy()
                    cv2.putText(
                        display_frame,
                        label,
                        (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1.0,
                        (0, 255, 0),
                        2,
                        cv2.LINE_AA,
                    )
                    cv2.imshow("Panneaux", display_frame)
                    key = cv2.waitKey(1) & 0xFF
                    if key in (27, ord("q")):
                        break

    except KeyboardInterrupt:
        print("Arrêt demandé — nettoyage...")
    finally:
        try:
            if picam is not None:
                picam.stop()
        except Exception:
            pass
        try:
            if camera is not None:
                camera.release()
        except Exception:
            pass
        try:
            if cam_power is not None:
                cam_power.off()
        except Exception:
            pass
        if not headless:
            cv2.destroyAllWindows()
