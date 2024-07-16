# Volume-Knob-Encoder Bluetooth Sender for Pico W Board

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

    last_state1 = pin1.value()
    last_state2 = pin1.value()

    while True:
        current_state1 = pin1.value()
        current_state2 = pin2.value()

        if p.is_connected():
            # send volume knob state out Bluetooth whenever it changes
            if current_state1 != last_state1 or current_state2 != last_state2:
                # print for debug
                print(f"State: {current_state1}{current_state2}")
                state = (current_state1 << 1) | current_state2
                p.send(bytes([state]))
                last_state1 = current_state1
                last_state2 = current_state2
        time.sleep_ms(10)

if __name__ == "__main__":
    sender()
