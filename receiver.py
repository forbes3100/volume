"""Volume-Knob-Encoder Bluetooth Receiver for Pico W Board"""

__copyright__ = "Copyright 2024, Scott Forbes"
__license__ = "GPL"

# Based on micropython/tree/master/examples/bluetooth/ble_simple_central.py

import bluetooth
import random
import struct
import time
import micropython
from ble_advertising import decode_services, decode_name
from micropython import const
from machine import Pin

GPIO_PIN1 = 11
GPIO_PIN_CONNECTED_LED = 12
GPIO_PIN2 = 13

# set up GPIO pins as outputs
pin1 = Pin(GPIO_PIN1, Pin.OUT)   # encoder data A to amp
pin2 = Pin(GPIO_PIN2, Pin.OUT)   # encoder data B to amp
pin_connected_led = Pin(GPIO_PIN_CONNECTED_LED, Pin.OUT)   # 'Connected' LED (active low)


_IRQ_SCAN_RESULT = const(5)
_IRQ_SCAN_DONE = const(6)
_IRQ_PERIPHERAL_CONNECT = const(7)
_IRQ_PERIPHERAL_DISCONNECT = const(8)
_IRQ_GATTC_SERVICE_RESULT = const(9)
_IRQ_GATTC_SERVICE_DONE = const(10)
_IRQ_GATTC_CHARACTERISTIC_RESULT = const(11)
_IRQ_GATTC_CHARACTERISTIC_DONE = const(12)
_IRQ_GATTC_WRITE_DONE = const(17)
_IRQ_GATTC_NOTIFY = const(18)

_ADV_IND = const(0x00)
_ADV_DIRECT_IND = const(0x01)

_UART_SERVICE_UUID = bluetooth.UUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E")
_UART_RX_CHAR_UUID = bluetooth.UUID("6E400002-B5A3-F393-E0A9-E50E24DCCA9E")
_UART_TX_CHAR_UUID = bluetooth.UUID("6E400003-B5A3-F393-E0A9-E50E24DCCA9E")


class BLESimpleCentral:
    def __init__(self, ble):
        self._ble = ble
        self._ble.active(True)
        self._ble.irq(self._irq)

        self._reset()
        self._continue_scanning = False  # Flag to control scanning
        self._scanning = False

    def _reset(self):
        print("central._reset")
        # Cached name and address from a successful scan.
        self._name = None
        self._addr_type = None
        self._addr = None

        # Callbacks for completion of various operations.
        # These reset back to None after being invoked.
        self._conn_callback = None
        self._read_callback = None

        # Persistent callback for when new data is notified from the device.
        self._notify_callback = None

        # Connected device.
        self._conn_handle = None
        self._start_handle = None
        self._end_handle = None
        self._tx_handle = None
        self._rx_handle = None

    def _irq(self, event, data):
        if event == _IRQ_SCAN_RESULT:
            addr_type, addr, adv_type, rssi, adv_data = data
            if adv_type in (_ADV_IND, _ADV_DIRECT_IND) and _UART_SERVICE_UUID in decode_services(
                adv_data
            ):
                # Found a potential device, remember it and stop scanning.
                self._addr_type = addr_type
                self._addr = bytes(
                    addr
                )  # Note: addr buffer is owned by caller so need to copy it.
                self._name = decode_name(adv_data) or "?"
                self._ble.gap_scan(None)

        elif event == _IRQ_SCAN_DONE:
            if self._scanning:
                if self._addr:
                    # Found a device during the scan (and the scan was explicitly stopped).
                    self._scan_callback(self._addr_type, self._addr, self._name)
                    self._scanning = False
                else:
                    # Scan timed out or no device was found
                    ##self._scan_callback(None, None, None)
                    print("Scan timed out or no device found.")
                    self._continue_scanning = True  # Set flag to true to indicate rescan needed

        elif event == _IRQ_PERIPHERAL_CONNECT:
            # Connect successful.
            conn_handle, addr_type, addr = data
            if addr_type == self._addr_type and addr == self._addr:
                self._conn_handle = conn_handle
                print(f"central._irq _IRQ_PERIPHERAL_CONNECT: {conn_handle=}")
                self._ble.gattc_discover_services(self._conn_handle)

        elif event == _IRQ_PERIPHERAL_DISCONNECT:
            # Disconnect (either initiated by us or the remote end).
            conn_handle, _, _ = data
            print(f"central._irq _IRQ_PERIPHERAL_DISCONNECT: {conn_handle=}")
            if conn_handle == self._conn_handle:
                # If it was initiated by us, it'll already be reset.
                self._reset()
                self._continue_scanning = True  # Signal to start scanning again

        elif event == _IRQ_GATTC_SERVICE_RESULT:
            # Connected device returned a service.
            conn_handle, start_handle, end_handle, uuid = data
            print("service", data)
            if conn_handle == self._conn_handle and uuid == _UART_SERVICE_UUID:
                self._start_handle, self._end_handle = start_handle, end_handle

        elif event == _IRQ_GATTC_SERVICE_DONE:
            # Service query complete.
            if self._start_handle and self._end_handle:
                self._ble.gattc_discover_characteristics(
                    self._conn_handle, self._start_handle, self._end_handle
                )
            else:
                print("Failed to find uart service.")

        elif event == _IRQ_GATTC_CHARACTERISTIC_RESULT:
            # Connected device returned a characteristic.
            conn_handle, def_handle, value_handle, properties, uuid = data
            if conn_handle == self._conn_handle and uuid == _UART_RX_CHAR_UUID:
                self._rx_handle = value_handle
            if conn_handle == self._conn_handle and uuid == _UART_TX_CHAR_UUID:
                self._tx_handle = value_handle

        elif event == _IRQ_GATTC_CHARACTERISTIC_DONE:
            # Characteristic query complete.
            if self._tx_handle is not None and self._rx_handle is not None:
                # We've finished connecting and discovering device, fire the connect callback.
                if self._conn_callback:
                    self._conn_callback()
            else:
                print("Failed to find uart rx characteristic.")

        elif event == _IRQ_GATTC_WRITE_DONE:
            conn_handle, value_handle, status = data
            print("TX complete")

        elif event == _IRQ_GATTC_NOTIFY:
            conn_handle, value_handle, notify_data = data
            if conn_handle == self._conn_handle and value_handle == self._tx_handle:
                if self._notify_callback:
                    self._notify_callback(notify_data)

        elif event == _IRQ_ERROR or event == _IRQ_UNEXPECTED_STATE:
            print("Error or unexpected state encountered.")
            self.disconnect()
            self._continue_scanning = True

        else:
            print(f"_irq: unknown event {event}")

    # Returns true if we've successfully connected and discovered characteristics.
    def is_connected(self):
        return (
            self._conn_handle is not None
            and self._tx_handle is not None
            and self._rx_handle is not None
        )

    # Find a device advertising the environmental sensor service.
    def scan(self, callback=None):
        print(f"central.scan({callback=})")
        self._addr_type = None
        self._addr = None
        self._scan_callback = callback
        self._scanning = True
        self._ble.gap_scan(2000, 30000, 30000)
        print("  scan done.")

    def manage_scanning(self):
        if self._continue_scanning:
            print("Restarting scan based on flag.")
            self.scan(self._scan_callback)
            self._continue_scanning = False  # Reset the flag after restarting the scan

    # Connect to the specified device (otherwise use cached address from a scan).
    def connect(self, addr_type=None, addr=None, callback=None):
        self._addr_type = addr_type or self._addr_type
        self._addr = addr or self._addr
        self._conn_callback = callback
        if self._addr_type is None or self._addr is None:
            print("No device address available, retrying scan...")
            self._continue_scanning = True
            return False
        # handle failures
        try:
            self._ble.gap_connect(self._addr_type, self._addr)
            return True
        except Exception as e:
            print(f"Error connecting: {e}")
            self._continue_scanning = True
            return False

    # Disconnect from current device.
    def disconnect(self):
        print("central.disconnect")
        if self._conn_handle is None:
            print("  no conn_handle!")
            return
        self._ble.gap_disconnect(self._conn_handle)
        self._reset()
        print("  disconnect done.")

    # Send data over the UART
    def write(self, v, response=False):
        if not self.is_connected():
            return
        self._ble.gattc_write(self._conn_handle, self._rx_handle, v, 1 if response else 0)

    # Set handler for when data is received over the UART.
    def on_notify(self, callback):
        self._notify_callback = callback


def on_scan(addr_type, addr, name, found, central):
    if addr_type is not None:
        found[0] = True
        addr_hex = hex(int.from_bytes(addr, 'big'))
        print("Found peripheral:", addr_type, addr_hex, name)
        success = central.connect()
        if not success:
            print("Connection attempt failed, resuming scan...")
            central.scan(lambda at, ad, n: on_scan(at, ad, n, found, central))

def on_rx(v, pin1, pin2):
    # received a state change over Bluetooth
    state = v[0]
    # print for debug
    print(f"RX {state:02b}")
    # update volume knob state going to amp
    pin1.value(state & 0x01)
    pin2.value((state >> 1) & 0x01)

last_rx_time = time.time()

def monitor_connection(central, timeout=5):
    global last_rx_time
    print("monitor_connection")
    if time.time() - last_rx_time > timeout:
        print("Connection timeout, resetting...")
        last_rx_time = time.time()
        central.disconnect()
        central.continue_scanning = True

def receiver():
    global last_rx_time
    ble = bluetooth.BLE()
    central = BLESimpleCentral(ble)
    found = [False]  # Use a list to maintain mutable state
    pin_connected_led.value(True)
    shown_connected = True

    while True:
        if not central.is_connected() and not found[0]:
            if shown_connected:
                print("Disconnected, initiating scan...")
                pin_connected_led.value(not False)
                shown_connected = False
            central.scan(lambda at, ad, n: on_scan(at, ad, n, found, central))

        central.manage_scanning()  # Check and handle the scanning flag
        time.sleep_ms(100)  # Prevent tight loop, adjust time as necessary

        if central.is_connected():
            last_rx_time = time.time()  # Reset the timer whenever a packet is received
            if found[0]:
                if not shown_connected:
                    print("Connected")
                    pin_connected_led.value(not True)
                    shown_connected = True
                central.on_notify(lambda v: on_rx(v, pin1, pin2))
        else:
            monitor_connection(central)  # Check if we should consider it disconnected

if __name__ == "__main__":
    receiver()
