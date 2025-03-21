import bluetooth
from micropython import const
import struct

ble = bluetooth.BLE()
ble.active(True)

# Start advertising
def advertise():
    ble.gap_advertise(100, b"\x02\x01\x06\x03\x03\xD8\xFE")
    print("Advertising...")

advertise()
