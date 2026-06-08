from gpiozero import LED, PWMOutputDevice
import time
import threading

# Ratio vert/rouge pour obtenir un orange plutot qu'un jaune (R a fond, G attenue)
ORANGE_GREEN_LEVEL = 0.45

# 3 LED onboard (HAT V3.1) - logique normale : GPIO=1 -> allumee
LED1 = 9
LED2 = 25
LED3 = 11

# 6 LED RGB des feux avant - logique inversee : GPIO=0 -> allumee
# Mapping verifie empiriquement (cf. tests 14/15/16 et 17/18/19) :
# cote gauche, le cablage est "tourne" par rapport aux noms des broches
Left_R = 13
Left_G = 19
Left_B = 0
Right_R = 1
Right_G = 5
Right_B = 6

leds = {}

# Etat du clignotant : None, 'left', 'right' ou 'warning'
blink_state = None
blink_lock = threading.Lock()


def setup():
    leds[1] = LED(LED1)
    leds[2] = LED(LED2)
    leds[3] = LED(LED3)
    leds[4] = PWMOutputDevice(Left_R, active_high=False)
    leds[5] = PWMOutputDevice(Left_G, active_high=False)
    leds[6] = PWMOutputDevice(Left_B, active_high=False)
    leds[7] = PWMOutputDevice(Right_R, active_high=False)
    leds[8] = PWMOutputDevice(Right_G, active_high=False)
    leds[9] = PWMOutputDevice(Right_B, active_high=False)


def switch(num, status):
    if num not in leds:
        print('Wrong Command: num must be between 1 and 9')
        return
    if status == 1:
        leds[num].on()
    elif status == 0:
        leds[num].off()
    else:
        print('Wrong Command: status must be 0 or 1')


def set_all_switch_off():
    for num in leds:
        switch(num, 0)


def stop_blinkers():
    for num in (4, 5, 7, 8):
        switch(num, 0)


def appel_phares():
    global blink_state
    previous_state = blink_state
    with blink_lock:
        blink_state = None
    stop_blinkers()

    flash_duration = 0.08
    print("Appel de phares")
    for _ in range(3):
        for num in (4, 5, 6, 7, 8, 9):
            leds[num].value = 1.0
        time.sleep(flash_duration)
        for num in (4, 5, 6, 7, 8, 9):
            leds[num].value = 0.0
        time.sleep(flash_duration)

    with blink_lock:
        blink_state = previous_state


def set_orange(r_num, g_num, on):
    if on:
        leds[r_num].value = 1.0
        leds[g_num].value = ORANGE_GREEN_LEVEL
    else:
        leds[r_num].value = 0.0
        leds[g_num].value = 0.0


def cancel_blink():
    global blink_state
    with blink_lock:
        if blink_state is None:
            return
        blink_state = None
    stop_blinkers()
    print("Clignotant coupe (commande manuelle recue)")


def set_blink(mode):
    global blink_state
    with blink_lock:
        # Retaper la meme commande desactive le clignotant en cours
        blink_state = None if blink_state == mode else mode
        new_state = blink_state

    if new_state is None:
        stop_blinkers()
        print("Clignotants eteints")
    elif new_state == 'left':
        print("Clignotant gauche active")
    elif new_state == 'right':
        print("Clignotant droit active")
    elif new_state == 'warning':
        print("Warnings actives")


def blink_worker():
    blink_period = 0.5
    while True:
        with blink_lock:
            state = blink_state

        if state == 'left':
            set_orange(4, 5, True)
            time.sleep(blink_period)
            set_orange(4, 5, False)
            time.sleep(blink_period)
        elif state == 'right':
            set_orange(7, 8, True)
            time.sleep(blink_period)
            set_orange(7, 8, False)
            time.sleep(blink_period)
        elif state == 'warning':
            set_orange(4, 5, True)
            set_orange(7, 8, True)
            time.sleep(blink_period)
            set_orange(4, 5, False)
            set_orange(7, 8, False)
            time.sleep(blink_period)
        else:
            time.sleep(0.1)


if __name__ == "__main__":
    setup()
    threading.Thread(target=blink_worker, daemon=True).start()

    try:
        while True:
            code = input()
            if code == 'q':
                break
            elif code == 'cl':
                set_blink('left')
                continue
            elif code == 'cr':
                set_blink('right')
                continue
            elif code == 'war':
                set_blink('warning')
                continue
            elif code == 'ap':
                appel_phares()
                continue

            try:
                code = int(code)
            except ValueError:
                print('Wrong Command: Example--11 pour allumer LED1, cl/cr/war pour les clignotants')
                continue

            cancel_blink()

            action = code // 10
            num = code % 10

            if action == 1:
                switch(num, 1)
            elif action == 2:
                switch(num, 0)
            else:
                print('Wrong Command: Example--11 pour allumer LED1')
    finally:
        with blink_lock:
            blink_state = None
        set_all_switch_off()
