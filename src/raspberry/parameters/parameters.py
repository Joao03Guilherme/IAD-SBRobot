from parameters.parameters_aux import data

config_path = "parameters/config.json"
config = data

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
