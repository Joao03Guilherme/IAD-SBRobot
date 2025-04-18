from machine import Pin, PWM
import math


class MotorController:
    def __init__(self, config: dict) -> None:
        """
        Initialize the MotorController with the given configuration.

        Args:
            config (dict): Dictionary containing motor pin configuration and PWM frequency.
        Returns:
            None
        """
        # Setup motor pins
        self.IN1 = Pin(config["IN1"], Pin.OUT)
        self.IN2 = Pin(config["IN2"], Pin.OUT)
        self.IN3 = Pin(config["IN3"], Pin.OUT)
        self.IN4 = Pin(config["IN4"], Pin.OUT)

        # Setup PWM for speed control
        self.ENA = PWM(Pin(config["ENA"]))
        self.ENB = PWM(Pin(config["ENB"]))

        # Set PWM frequency
        self.ENA.freq(config["PWM_FREQ"])
        self.ENB.freq(config["PWM_FREQ"])

        # Initialize motors to stopped state
        self.stop()

    def motor_a(self, power: int) -> None:
        """
        Set the power for motor A.

        Args:
            power (int): Power value (-100 to 100). Positive for forward, negative for backward.
        Returns:
            None
        """
        duty = int(abs(power) * 65535 / 100)

        if power > 0:
            self.IN1.value(1)
            self.IN2.value(0)
        elif power < 0:
            self.IN1.value(0)
            self.IN2.value(1)
        else:
            self.IN1.value(0)
            self.IN2.value(0)

        self.ENA.duty_u16(duty)

    def motor_b(self, power: int) -> None:
        """
        Set the power for motor B.

        Args:
            power (int): Power value (-100 to 100). Positive for forward, negative for backward.
        Returns:
            None
        """
        duty = int(abs(power) * 65535 / 100)

        if power > 0:
            self.IN3.value(1)
            self.IN4.value(0)
        elif power < 0:
            self.IN3.value(0)
            self.IN4.value(1)
        else:
            self.IN3.value(0)
            self.IN4.value(0)

        self.ENB.duty_u16(duty)

    def set_speeds(self, left_speed, right_speed):
        """Set both motor speeds (-255 to 255 scale)"""
        # Convert from Arduino's -255 to 255 range to -100 to 100 percentage
        left_scaled = (left_speed * 100) // 255
        right_scaled = (right_speed * 100) // 255

        self.motor_a(left_scaled)
        self.motor_b(right_scaled)

    def stop(self) -> None:
        """
        Stop both motors by setting their power to zero.

        Returns:
            None
        """
        self.motor_a(0)
        self.motor_b(0)
