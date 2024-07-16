# volume
Volume-knob encoder sent over Bluetooth using Raspberry Pi Pico W boards

Install MicroPython for the Pico W from https://rpf.io/pico-w-firmware, dropping the downloaded RPI_PICO_W .uf2 file into your RPI-RP2 folder that shows up when it's connected via USB.

Download and install mpremote:

    pip install mpremote

Download from https://github.com/micropython/micropython/tree/master/examples/bluetooth

    ble_advertising.py
    ble_simple_central.py
    ble_simple_peripheral.py
    
Reconnect each Pico W board one-at-a-time and list the USB device names to see where the board's USB serial is located. In the following example the sender board is at /dev/tty.usbmodem11401, and the receiver at /dev/tty.usbmodem11101.

For the sender board:

	mpremote connect /dev/tty.usbmodem11401 fs cp ble_advertising.py :ble_advertising.py
 	mpremote connect /dev/tty.usbmodem11401 fs cp ble_simple_peripheral.py :ble_simple_peripheral.py
 	mpremote connect /dev/tty.usbmodem11401 fs cp sender.py :sender.py
 	mpremote connect /dev/tty.usbmodem11401 run sender.py

And the receiver board:

	mpremote connect /dev/tty.usbmodem11101 fs cp ble_advertising.py :ble_advertising.py  
	mpremote connect /dev/tty.usbmodem11101 fs cp ble_simple_central.py :ble_simple_central.py
	mpremote connect /dev/tty.usbmodem11101 fs cp receiver.py :receiver.py
 	mpremote connect /dev/tty.usbmodem11101 run receiver.py

Finally, copy sender.py and receiver.py to main.py on their respective boards so that the code runs on power-up.