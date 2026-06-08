import time
import board
import busio
from adafruit_bus_device.i2c_device import I2CDevice

i2c = busio.I2C(board.SCL, board.SDA)
# ADS7830 adress 0x48
device = I2CDevice(i2c, 0x48)

# Define constants
Vref = 8.4
WarningThreshold = 6.75
R15 = 3000
R17 = 1000
DivisionRatio = R17 / (R15 + R17)

PlausibleMin = WarningThreshold - 1.0
PlausibleMax = Vref + 0.5

# Define the ADC channel and command.
cmd = 0x84
channel = 0
control_byte = cmd | (((channel << 2 | channel >> 1) & 0x07) << 4)


def read_battery_voltage():
    buffer = [1]
    device.write_then_readinto(bytes([control_byte]), buffer)
    adcValue = buffer[0]
    A0Voltage = (adcValue / 255) * 5
    return A0Voltage / DivisionRatio


def battery_percentage(voltage):
    if voltage < PlausibleMin or voltage > PlausibleMax:
        return None

    percentage = (voltage - WarningThreshold) / (Vref - WarningThreshold) * 100
    return max(0, min(100, percentage))


if __name__ == "__main__":
    while True:
        voltage = read_battery_voltage()
        percentage = battery_percentage(voltage)

        if percentage is None:
            print(f"Tension mesuree: {voltage:.2f} V -> hors plage plausible "
                  f"[{PlausibleMin:.2f} V, {PlausibleMax:.2f} V], verifier le "
                  f"branchement de la batterie et le cablage du capteur")
        else:
            print(f"Tension batterie: {voltage:.2f} V | Niveau: {percentage:.2f} %")
            if percentage < 20:
                print("Warning! The battery level is too low. Please charge in time!")

        time.sleep(0.5)
