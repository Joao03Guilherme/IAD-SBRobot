import json
import os

# Get the directory where this script is located
script_dir = os.path.dirname(os.path.abspath(__file__))
# Build the path to config.json
config_path = os.path.join(script_dir, "config.json")

# Load JSON data with absolute path
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