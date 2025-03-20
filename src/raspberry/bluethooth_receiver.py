import bluetooth
from machine import Pin
from micropython import const
import time

# Initialize Bluetooth
ble = bluetooth.BLE()
ble.active(True)

# Define a simple service with one characteristic
SERVICE_UUID = bluetooth.UUID("12345678-1234-5678-1234-56789abcdef0")
CHAR_UUID = bluetooth.UUID("87654321-4321-6789-4321-abcdef012345")
SERVICE = (SERVICE_UUID, (CHAR_UUID, bluetooth.FLAG_READ | bluetooth.FLAG_WRITE),)

# Advertise the service
ble.gatts_register_services([SERVICE])
ble.gap_advertise(100, b'\x02\x01\x06')

print("Bluetooth LE Started, Advertising...")

def on_rx(event):
    print("Received:", event)

# Set a callback for incoming data
ble.gatts_set_buffer(0, 100, True)
ble.gatts_write(0, "Hello from Pico W 2!")

while True:
    time.sleep(1)
