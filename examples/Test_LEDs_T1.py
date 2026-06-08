from gpiozero import LED
import time

# 3 LED onboard (HAT V3.1) - logique normale : GPIO=1 -> allumee
LED1 = 9
LED2 = 25
LED3 = 11

# 6 LED RGB des feux avant - logique inversee : GPIO=0 -> allumee
Left_R = 19
Left_G = 0
Left_B = 13
Right_R = 1
Right_G = 5
Right_B = 6

leds = {}


def setup():
    leds[1] = LED(LED1)
    leds[2] = LED(LED2)
    leds[3] = LED(LED3)
    leds[4] = LED(Left_R, active_high=False)
    leds[5] = LED(Left_G, active_high=False)
    leds[6] = LED(Left_B, active_high=False)
    leds[7] = LED(Right_R, active_high=False)
    leds[8] = LED(Right_G, active_high=False)
    leds[9] = LED(Right_B, active_high=False)


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


if __name__ == "__main__":
    setup()
    try:
        while True:
            code = input()
            if code == 'q':
                break

            code = int(code)
            action = code // 10 
            num = code % 10 

            if action == 1:
                switch(num, 1)
            elif action == 2:
                switch(num, 0)
            else:
                print('Wrong Command: Example--11 pour allumer LED1')
    finally:
        set_all_switch_off()
