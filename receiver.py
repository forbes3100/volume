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

def on_scan(addr_type, addr, name, found, central):
    if addr_type is not None:
        found[0] = True
        addr_hex = hex(int.from_bytes(addr, 'big'))
        print("Found peripheral:", addr_type, addr_hex, name)
        central.connect()

def on_rx(v, pin1, pin2):
    # received a state change over Bluetooth
    state = v[0]
    # print for debug
    print(f"RX {state:02b}")
    # update volume knob state going to amp
    pin1.value(state & 0x01)
    pin2.value((state >> 1) & 0x01)

def receiver():
    ble = bluetooth.BLE()
    central = ble_simple_central.BLESimpleCentral(ble)
    found = [False]  # Use a list to maintain mutable state

    while True:
        central.scan(lambda at, ad, n: on_scan(at, ad, n, found, central))

        # wait for connection...
        while not (central.is_connected() and found[0]):
            time.sleep_ms(100)

        print("Connected")
        central.on_notify(lambda v: on_rx(v, pin1, pin2))

        while central.is_connected():
            time.sleep_ms(400)

        print("Disconnected")
        found[0] = False  # Reset found status

if __name__ == "__main__":
    receiver()
