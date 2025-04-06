"""
Global parameters for the self-balancing robot project.
All constants used throughout the codebase should be defined here.
"""

# ============================
# MOTOR SETTINGS
# ============================
MOTOR_CONFIG = {
    "IN1": 2,
    "IN2": 3,
    "IN3": 4,
    "IN4": 5,
    "ENA": 6,
    "ENB": 7,
    "PWM_FREQ": 1000,
}

# ============================
# MPU6050 SETTINGS
# ============================
MPU_CONFIG = {
    "i2c_id": 1,  # I2C bus ID
    "sda_pin": 26,  # SDA pin number
    "scl_pin": 27,  # SCL pin number
    "address": 0x68,  # I2C address of the MPU6050
    "accel_xout_h": 0x3B,  # Register addresses for accelerometer and gyroscope data
    "accel_xout_l": 0x3C,
    "accel_yout_h": 0x3D,
    "accel_yout_l": 0x3E,
    "accel_zout_h": 0x3F,
    "accel_zout_l": 0x40,
    "gyro_xout_h": 0x43,
    "gyro_xout_l": 0x44,
    "gyro_yout_h": 0x45,
    "gyro_yout_l": 0x46,
    "gyro_zout_h": 0x47,
    "gyro_zout_l": 0x48,
    "bias_x": -1064.831,  # Bias values for accelerometer and gyroscope
    "bias_y": 224.058,
    "bias_z": -1357.831,
    "gyro_scale_factor": 131.0,  # Default is ±250 deg/s range, which gives 131 LSB/(deg/s)
    "filter_time_constant": 200,  # Time constant for complementary filter (in seconds)
}

# ============================
# PID CONTROLLER SETTINGS
# ============================
PID_CONFIG = {
    "sample_time": 0.0001,  # Time between PID updates (in seconds)
    "kp": 50,  # Proportional gain
    "ki": 30,  # Integral gain
    "kd": 90,  # Derivative gain
    "max_safe_tilt": 100,  # Maximum safe tilt angle (in degrees)
    "max_acceleration": 10,  # Maximum acceleration in units/second
    "acceleration_smoothing": 0.2,  # Lower values = smoother acceleration (0-1)
    "k_acceleration": 2.0,  # Coefficient for acceleration/deceleration response
    "k_torque_per_pw": 0.5,  # Models how torque requirements change with speed
    "drag": 1.0,  # Coefficient for overcoming friction/drag forces
    "alpha": 0.97,  # Filter constant to combine gyro and accelerometer data (1 = only gyro)
}


# ============================
# ENCODER SETTINGS
# ============================
ENCODER_CONFIG = {
    "pin_a": 22,
    "pin_b": 28,
    "pulses_per_rev": 20,
    "wheel_diameter": 0.067,
    "buffer_size": 5,
    "measurement_interval_ms": 70,
}


# ============================
# BLUETHOOTH SETTINGS
# ============================
COMMANDS = {
    "1": {"action": "START", "description": "Start balancing"},
    "2": {"action": "STOP", "description": "Stop all motors"},
    "3": {"action": "DRIVE", "description": "Drive with speed & turn (e.g., 3 20 5)"},
    "4": {"action": "TURN", "description": "Turn by angle (e.g., 4 90 1)"},
    "5": {"action": "CONFIG", "description": "Update configuration"},
    "6": {"action": "CALIBRATE", "description": "Start calibration procedure"},
    "7": {"action": "HELP", "description": "Show this help"},
    "8": {"action": "STATUS", "description": "Show current robot status"},
    "9": {"action": "RESET", "description": "Reset the robot"},
}

ROBOT_SERVICE_UUID = "19B10000-E8F2-537E-4F6C-D104768A1214"
CONTROL_CHAR_UUID = "19B10001-E8F2-537E-4F6C-D104768A1214"
TELEMETRY_CHAR_UUID = "19B10002-E8F2-537E-4F6C-D104768A1214"
PICO_NAME = "PicoBot"
PICO_ADRESS = "33844EF1-F428-5A3B-DF48-83DFC12F9890"

# TODO: Add telemetry and logging settings