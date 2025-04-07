import json
import os

# In MicroPython, we use simpler path handling
# Directly specify the config file in the same directory
config_path = "parameters/config.json"

# Load JSON data
with open(config_path, "r") as f:
    config = json.load(f)

# Assign variables
MOTOR_CONFIG = config["MOTOR_CONFIG"]
MPU_CONFIG = config["MPU_CONFIG"]
PID_CONFIG = config["PID_CONFIG"]
ENCODER_CONFIG = config["ENCODER_CONFIG"]
COMMANDS = config["COMMANDS"]

ROBOT_SERVICE_UUID = config["ROBOT_SERVICE_UUID"]
CONTROL_CHAR_UUID = config["CONTROL_CHAR_UUID"]
TELEMETRY_CHAR_UUID = config["TELEMETRY_CHAR_UUID"]
PICO_NAME = config["PICO_NAME"]
PICO_ADRESS = config["PICO_ADRESS"]