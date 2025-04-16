![Project Banner](assets/banner.png)


## 🗂️ Table of Contents

- [🚀 Project Overview](#-project-overview)
- [🛠️ Hardware Components](#-hardware-components)
- [🔌 Pinout Configuration](#-pinout-configuration)
- [🧠 PID Control Theory](#-pid-control-theory)
- [🧩 Software Architecture](#-software-architecture)
- [⚙️ Setup and Configuration](#-setup-and-configuration)
- [🎬 Demo](#-demo)

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

<img src="assets/pinout.png" alt="Pinout Configuration" width="800"/>

<div align="center">

| Component         | Signal/Function   | Pico GPIO |
|-------------------|-------------------|-----------|
| Motor A (Left)    | IN1               | 2         |
| Motor A (Left)    | IN2               | 3         |
| Motor A (Left)    | ENA (PWM)         | 6         |
| Motor B (Right)   | IN3               | 4         |
| Motor B (Right)   | IN4               | 5         |
| Motor B (Right)   | ENB (PWM)         | 7         |
| MPU6050           | SDA               | 26        |
| MPU6050           | SCL               | 27        |
| Left Encoder      | Signal            | 22        |
| Right Encoder     | Signal            | 28        |
| Buzzer            | Signal            | 0         |

</div>

## 🧠 PID Control Theory

A PID (Proportional-Integral-Derivative) controller is a feedback mechanism widely used in control systems. It continuously calculates an error value as the difference between a desired setpoint and a measured process variable, applying corrections based on proportional, integral, and derivative terms:

- **Proportional (P):** Reacts to the current error. Higher values increase responsiveness but can cause overshoot.
- **Integral (I):** Reacts to the accumulation of past errors. Helps eliminate steady-state error but can introduce instability if too high.
- **Derivative (D):** Reacts to the rate of change of the error. Helps dampen oscillations and improve stability.

In this robot, the PID controller processes the tilt angle from the IMU sensor to keep the robot balanced by adjusting the motor outputs in real time. Proper tuning of the PID gains is essential for stable and responsive balancing.

## 🧩 Software Architecture

The codebase is organized into several modules, each with a clear responsibility to ensure modularity, maintainability, and ease of extension:

- [`src/raspberry/main.py`](src/raspberry/main.py):
  - **Entry point and main control loop.**
  - Initializes all controllers, handles Bluetooth commands, telemetry, and orchestrates the robot's operation.

- [`controllers/`](src/raspberry/controllers):
  - [`gyroscope_controller.py`](src/raspberry/controllers/gyroscope_controller.py):
    - Interfaces with the MPU6050 IMU to read angles and acceleration.
    - Provides sensor fusion and filtering for accurate orientation estimation.
  - [`motor_controller.py`](src/raspberry/controllers/motor_controller.py):
    - Low-level control of the DC motors via the L298N driver.
    - Supports PWM speed control and direction management.
  - [`balance_controller.py`](src/raspberry/controllers/balance_controller.py):
    - Implements the PID control loop for self-balancing.
    - Contains driving logic for speed and turning, and safety checks for tilt.
  - [`encoder_controller.py`](src/raspberry/controllers/encoder_controller.py):
    - Reads wheel encoder pulses to measure speed and distance.
    - Enables closed-loop speed control and odometry.
  - [`buzzer_controller.py`](src/raspberry/controllers/buzzer_controller.py):
    - Controls the passive buzzer for sound feedback.
    - Supports playing melodies and sound effects asynchronously.

- [`bluetooth/`](src/raspberry/bluethooth):
  - [`BLEReceiver.py`](src/raspberry/bluethooth/BLEReceiver.py):
    - Handles Bluetooth Low Energy (BLE) communication.
    - Receives commands and sends telemetry to a remote device.

- [`parameters/`](src/raspberry/parameters):
  - [`parameters.py`](src/raspberry/parameters/parameters.py):
    - Central location for all system constants and configuration values.
    - Includes PID gains, hardware pin mappings, and calibration data.

- [`training/`](src/raspberry/training):
  - [`robot_interface.py`](src/raspberry/training/robot_interface.py):
    - Provides an interface for reinforcement learning experiments.
    - Allows external agents to interact with the robot for training and evaluation.

**Design Principles:**
- **Modularity:** Each hardware component and control function is encapsulated in its own module.
- **Asynchronous Operation:** Uses `asyncio` for concurrent tasks (e.g., BLE communication, telemetry, and control loops).
- **Extensibility:** New sensors, actuators, or control strategies can be added with minimal changes to the main logic.
- **Separation of Concerns:** Hardware abstraction, control logic, and communication are clearly separated for clarity and testability.

**Typical Data Flow:**
1. Sensor data (IMU, encoders) is read by controller modules.
2. The balance controller computes motor commands using PID logic.
3. Motor controller actuates the motors accordingly.
4. BLEReceiver handles incoming commands and outgoing telemetry.
5. Buzzer controller provides sound feedback for events and status.

This architecture enables robust, real-time control while remaining easy to understand and modify for experimentation or educational purposes.

## ⚙️ Setup and Configuration

1. **Hardware Assembly**:
   - Connect components according to the pinout configuration
   - Mount the MPU6050 at the center of gravity, with X axis aligned to the forward direction

2. **Software Installation**:
   - Install MicroPython on the Raspberry Pi Pico
   - Upload all project files to the Pico

3. **Initial Configuration**:
   - Customize [`parameters/parameters.py`](src/raspberry/parameters/parameters.py) as needed for your specific hardware
   - Initial calibration is required before first use

## 🎬 Demo

<div align="center">

https://github.com/user-attachments/assets/34d4b190-f3cc-48b9-b1b2-641a364e285a

<div style="display: flex; justify-content: center; gap: 20px;">
  <img src="assets/full_bot.jpeg" alt="Full view of the robot" width="350" style="margin:10px;"/>
  <img src="assets/inside_bot.jpeg" alt="Internal components of the robot" width="350" style="margin:10px;"/>
  <img src="assets/plot.png" alt="Plot of the robot's angle" width="350" style="margin:10px;"/>
</div>

</div>



