![Project Banner](assets/banner.png)


## Table of Contents

- [Project Overview](#-project-overview)
- [Hardware Components](#-hardware-components)
- [Pinout Configuration](#-pinout-configuration)
- Software Architecture
- Setup and Configuration
- Usage Instructions
- Parameter Tuning
- Calibration
- Troubleshooting

## 🚀 Project Overview

This self balancing robot is powered by the Raspberry Pi Pico 2W. Designed for learning, experimentation, and fun, it brings together real-time control, multiple sensors and wireless communication in a compact robotics package.

**Key Features:**
- **Real-Time Balancing:** Uses an MPU6050 IMU and a robust PID controller to keep the robot upright.
- **Modular Architecture:** Clean separation of motor, sensor, control, and communication logic for easy extension and maintenance.
- **Bluetooth Remote Control:** Drive, tune, and calibrate your robot wirelessly via a simple command interface.
- **Live Parameter Tuning:** Adjust PID and balance parameters on the fly for rapid experimentation.
- **Sound & Feedback:** Play melodies (like the Star Wars theme!) and R2D2-style sounds with the onboard buzzer for interactive feedback.
- **Educational Focus:** Ideal for exploring robotics, control systems, and embedded programming.


## 🛠️ Hardware Components

- **Controller**: Raspberry Pi Pico 2 W (with Wi-Fi and Bluetooth)
- **Sensors**:
  - MPU6050 6-axis gyroscope and accelerometer (MPU9250 recommended)
  - 2 Wheel encoders 
- **Actuators**:
  - L298N motor driver 
  - 2 DC motors with gearboxes
  - Passive buzzer
- **Power**:
  - 2 9V Batteries 

## 🔌 Pinout Configuration

![Pinout Configuration](assets/pinout.png)

### Motor Controller Pins
```
Motor A (Left):
- IN1: GPIO 2
- IN2: GPIO 3
- ENA (PWM): GPIO 6

Motor B (Right):
- IN3: GPIO 4
- IN4: GPIO 5
- ENB (PWM): GPIO 7
```

### MPU6050 Sensor
```
- SDA: GPIO 26
- SCL: GPIO 27
```

### Wheel Encoders
```
- Left Encoder: GPIO 22
- Right Encoder: GPIO 28
```

### Status Indicator
```
- LED: Onboard LED (Pico)
```

## Software Architecture

The codebase is organized into several modules:

- [`src/raspberry/main.py`](src/raspberry/main.py): Entry point and overall robot control
- [`controllers/`](src/raspberry/controllers):
  - [`gyroscope_controller.py`](src/raspberry/controllers/gyroscope_controller.py): MPU6050 interface and sensor fusion
  - [`motor_controller.py`](src/raspberry/controllers/motor_controller.py): Motor control
  - [`balance_controller.py`](src/raspberry/controllers/balance_controller.py): PID control and driving logic
  - [`encoder_controller.py`](src/raspberry/controllers/encoder_controller.py): Speed measurement
- `bluetooth/`:
  - [`BLEReceiver.py`](src/raspberry/bluethooth/BLEReceiver.py): Bluetooth communication
- [`parameters/`](src/raspberry/parameters):
  - `parameters.py`: All system constants and configuration
- `training/`:
  - [`robot_interface.py`](src/raspberry/training/robot_interface.py): Interface for reinforcement learning

## Setup and Configuration

1. **Hardware Assembly**:
   - Connect components according to the pinout configuration
   - Mount the MPU6050 at the center of gravity, with X axis aligned to the forward direction

2. **Software Installation**:
   - Install MicroPython on the Raspberry Pi Pico
   - Upload all project files to the Pico

3. **Initial Configuration**:
   - Customize [`parameters/parameters.py`](src/raspberry/parameters/parameters.py) as needed for your specific hardware
   - Initial calibration is required before first use

## Usage Instructions

### Basic Operation

1. **Power On**: Turn on the robot in a balanced position
2. **Calibration**: Run the calibration procedure before first use
3. **Balancing**: Start the balancing mode with the START command
4. **Control**: Send movement commands (DRIVE) to move the robot

### Command Interface

Commands can be sent via Bluetooth:

- `1`: START - Begin balancing
- `2`: STOP - Stop motors
- `3 <speed> <turn>`: DRIVE - Move with speed (-100 to 100) and turn (-100 to 100)
- `5 <param> <value>`: CONFIG - Update parameter values
- `6`: CALIBRATE - Calibrate sensors

### Telemetry Data

The robot outputs telemetry in this format:
```
A:<angle>,B:<balance_angle>,S:<speed>,T:<turn>,R:<running>,L:<left_power>,R:<right_power>
```

## Parameter Tuning

### PID Controller Parameters

- `KP`: Proportional gain (40-50 recommended)
- `KI`: Integral gain (10-30 recommended)
- `KD`: Derivative gain (30-90 recommended)
- `K_DAMPING`: Dampens oscillations (0.8-1.2 recommended)

### Balance Parameters

- `MAX_SAFE_TILT`: Maximum allowed tilt angle before emergency stop
- `BALANCE2OFFSET`: Fine adjustment for balance point
- `MAX_CORRECTION_TILT`: Maximum correction angle

### Speed Control Parameters

- `K_ACCELERATION`: Controls acceleration response
- `K_TORQUE_PER_PW`: Models how torque requirements change with speed
- `DRAG`: Coefficient for friction compensation

## Calibration

1. Place the robot on a level surface
2. Keep it completely still
3. Run the CALIBRATE command
4. Wait for calibration to complete (typically ~5 seconds)
5. The robot will report calibration values

The calibration process:
- Determines gyroscope and accelerometer biases
- Calculates the natural balance point

## Troubleshooting

### Robot Falls Forward/Backward

- Adjust the `BALANCE2OFFSET` parameter
- Verify MPU6050 mounting orientation
- Re-calibrate on a level surface

### Oscillations

- Decrease `KP` and `KI` parameters
- Increase `KD` parameter
- Adjust `K_DAMPING` higher

### Motors Not Responding

- Check motor connections
- Verify battery voltage (>7.2V recommended)
- Ensure motor driver is receiving power

### Poor Turning Performance

- Adjust turning parameters in the PID controller
- Check for mechanical issues in wheels

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgments

- Based on inverted pendulum control theory
- Uses complementary filter for sensor fusion
- Implements PID control for balance




