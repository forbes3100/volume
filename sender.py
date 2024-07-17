"""Volume-Knob-Encoder Bluetooth Sender for Pico W Board"""

__copyright__ = "Copyright 2024, Scott Forbes"
__license__ = "GPL"

from machine import Pin
import bluetooth
import time
import ble_simple_peripheral

GPIO_PIN1 = 11
GPIO_PIN2 = 13

# set up GPIO pins
pin1 = Pin(GPIO_PIN1, Pin.IN)   # encoder data A
pin2 = Pin(GPIO_PIN2, Pin.IN)   # encoder data B

def sender():
    ble = bluetooth.BLE()
    p = ble_simple_peripheral.BLESimplePeripheral(ble)

    last_state = 4

    while True:
        state = (pin1.value() << 1) | pin2.value()

        if p.is_connected():
            # send volume knob state out Bluetooth whenever it changes
            if state != last_state:
                # print for debug
                print(f"{state=:02b}")
                p.send(bytes([state]))
                last_state = state
        time.sleep_ms(10)

if __name__ == "__main__":
    sender()
