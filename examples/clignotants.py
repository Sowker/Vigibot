import time
import threading
import importlib.util
import os

_here = os.path.dirname(os.path.abspath(__file__))


def _load_module(name, filename):
    path = os.path.join(_here, filename)
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


front = _load_module("test_leds_t1", "Test_LEDs_T1.py")
_spi_module = _load_module("spi_ws2812", "06_Spi_WS2812.py")
Adeept_SPI_LedPixel = _spi_module.Adeept_SPI_LedPixel

ORANGE_RGB = [255, 128, 0] 

WS_LEFT = [2, 3, 4, 11, 12, 13]
WS_RIGHT = [5, 6, 7, 8, 9, 10]

strip = None


def set_strip(indices, on):
    color = ORANGE_RGB if on else [0, 0, 0]
    for i in indices:
        strip.set_led_rgb_data(i, color)
    strip.show()


def stop_strip():
    set_strip(WS_LEFT, False)
    set_strip(WS_RIGHT, False)


def strip_worker():

    blink_period = 0.5
    while True:
        with front.blink_lock:
            state = front.blink_state

        if state == 'left':
            set_strip(WS_LEFT, True)
            time.sleep(blink_period)
            set_strip(WS_LEFT, False)
            time.sleep(blink_period)
        elif state == 'right':
            set_strip(WS_RIGHT, True)
            time.sleep(blink_period)
            set_strip(WS_RIGHT, False)
            time.sleep(blink_period)
        elif state == 'warning':
            set_strip(WS_LEFT, True)
            set_strip(WS_RIGHT, True)
            time.sleep(blink_period)
            set_strip(WS_LEFT, False)
            set_strip(WS_RIGHT, False)
            time.sleep(blink_period)
        else:
            stop_strip()
            time.sleep(0.1)


if __name__ == "__main__":
    front.setup()

    strip = Adeept_SPI_LedPixel(14, 255)
    if strip.check_spi_state() == 0:
        print("Attention : bus SPI indisponible, le ruban WS2812 ne s'allumera pas")
    strip.set_all_led_color(0, 0, 0)

    threading.Thread(target=front.blink_worker, daemon=True).start()
    threading.Thread(target=strip_worker, daemon=True).start()

    print("=== Clignotants avant (LED RGB) + arriere (ruban WS2812) ===")
    print("Commandes identiques a Test_LEDs_T1 :")
    print("  cl  : active/desactive le clignotant gauche")
    print("  cr  : active/desactive le clignotant droit")
    print("  war : active/desactive les warnings (gauche + droite)")
    print("  ap  : appel de phares")
    print("  q   : quitter")

    try:
        while True:
            code = input()
            if code == 'q':
                break
            elif code == 'cl':
                front.set_blink('left')
                continue
            elif code == 'cr':
                front.set_blink('right')
                continue
            elif code == 'war':
                front.set_blink('warning')
                continue
            elif code == 'ap':
                front.appel_phares()
                continue

            try:
                code = int(code)
            except ValueError:
                print('Wrong Command: Example--11 pour allumer LED1, cl/cr/war pour les clignotants')
                continue

            front.cancel_blink()

            action = code // 10
            num = code % 10

            if action == 1:
                front.switch(num, 1)
            elif action == 2:
                front.switch(num, 0)
            else:
                print('Wrong Command: Example--11 pour allumer LED1')
    finally:
        with front.blink_lock:
            front.blink_state = None
        front.set_all_switch_off()
        stop_strip()
        strip.led_close()
