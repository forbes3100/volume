"""Volume-Knob-Encoder Bluetooth Receiver for Pico W Board"""

__copyright__ = "Copyright 2024, Scott Forbes"
__license__ = "GPL"

import bluetooth
import time
from machine import Pin
import ble_simple_central

GPIO_PIN1 = 11
GPIO_PIN2 = 13

# set up GPIO pins as outputs
pin1 = Pin(GPIO_PIN1, Pin.OUT)   # encoder data A to amp
pin2 = Pin(GPIO_PIN2, Pin.OUT)   # encoder data B to amp

def receiver():
    ble = bluetooth.BLE()
    central = ble_simple_central.BLESimpleCentral(ble)

    not_found = False

    def on_scan(addr_type, addr, name):
        if addr_type is not None:
            print("Found peripheral:", addr_type, addr, name)
            central.connect()
        else:
            nonlocal not_found
            not_found = True
            print("No peripheral found.")

    central.scan(callback=on_scan)

    # wait for connection...
    while not central.is_connected():
        time.sleep_ms(100)
        if not_found:
            return

    print("Connected")

    def on_rx(v):
        # received a state change over Bluetooth
        state = v[0]
        # print for debug
        print(f"RX {state:02b}")
        # update volume knob state going to amp
        pin1.value(state & 0x01)
        pin2.value((state >> 1) & 0x01)

    central.on_notify(on_rx)

    while central.is_connected():
        time.sleep_ms(400)

    print("Disconnected")

if __name__ == "__main__":
    receiver()
