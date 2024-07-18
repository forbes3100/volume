"""Volume-Knob-Encoder Bluetooth Sender for Pico W Board"""

__copyright__ = "Copyright 2024, Scott Forbes"
__license__ = "GPL"

from machine import Pin
import bluetooth
import rp2
import time
import ble_simple_peripheral

GPIO_PIN1 = 11
GPIO_PIN2 = 13

# set up GPIO pins
pin1 = Pin(GPIO_PIN1, Pin.IN)   # encoder data A from knob
pin2 = Pin(GPIO_PIN2, Pin.IN)   # encoder data B from knob

# PIO program to wait for a pin change
@rp2.asm_pio()
def wait_pin_change():
    wrap_target()

    wait(0, pin, 0)
    irq(block, rel(0))
    wait(1, pin, 0)
    irq(block, rel(0))

    wrap()

# IRQ handler to send the state via Bluetooth
def handler(sm):
    # read the state of both pins
    state = (pin1.value() << 1) | pin2.value()
    if p.is_connected():
        print(f"{state=:02b}")
        p.send(bytes([state]))
    else:
        print(f"nc {state=:02b}")

def sender():
    global p
    ble = bluetooth.BLE()
    p = ble_simple_peripheral.BLESimplePeripheral(ble)

    # instantiate a StateMachine with wait_pin_change program on each pin
    sm0 = rp2.StateMachine(0, wait_pin_change, in_base=pin1)
    sm0.irq(handler)
    sm1 = rp2.StateMachine(1, wait_pin_change, in_base=pin2)
    sm1.irq(handler)

    # start the StateMachines running
    sm0.active(1)
    sm1.active(1)

    # keep the main thread alive
    while True:
        time.sleep(1)

if __name__ == "__main__":
    sender()
