from machine import Pin
import asyncio
import time
import math
import parameters.parameters as params


class WheelEncoder:
    def __init__(self, encoder_pin=19, pulses_per_rev=20, wheel_diameter=0.067):
        self.ENCODER_PIN = encoder_pin
        self.PULSES_PER_REV = pulses_per_rev
        self.WHEEL_DIAMETER = wheel_diameter

        self.last_pulse_count = 0
        self.pulse_count = 0
        self.motor_direction = 0
        self.encoder = None

        # Buffers for storing historical data (last 50 points)
        self.rpm_buffer = [0] * params.data["ENCODER_CONFIG"]["buffer_size"]
        self.time_buffer = [0] * params.data["ENCODER_CONFIG"]["buffer_size"]
        self.is_running = False
        self.last_measurement_time = 0
        self.measurement_interval_ms = params.data["ENCODER_CONFIG"][
            "measurement_interval_ms"
        ]

        # Initialize the encoder
        self.initialize_encoder()

    def update_parameters(self):
        self.rpm_buffer = [0] * params.data["ENCODER_CONFIG"]["buffer_size"]
        self.time_buffer = [0] * params.data["ENCODER_CONFIG"]["buffer_size"]
        self.measurement_interval_ms = params.data["ENCODER_CONFIG"][
            "measurement_interval_ms"
        ]

    def initialize_encoder(self):
        # Configure the encoder pin as input with internal pull-up resistor
        self.encoder = Pin(self.ENCODER_PIN, Pin.IN, Pin.PULL_UP)
        # Attach an interrupt on the rising edge
        self.encoder.irq(trigger=Pin.IRQ_RISING, handler=self._irq_handler)

    def _irq_handler(self, pin):
        self.pulse_count += self.motor_direction

    async def run_background_measurement(self):
        """Continuously measures RPM and maintains the data buffer."""
        self.is_running = True
        while self.is_running:
            current_time = time.ticks_ms()

            # Only take measurement if enough time has passed
            if (
                time.ticks_diff(current_time, self.last_measurement_time)
                >= self.measurement_interval_ms
            ):
                # Get current RPM
                count = self.pulse_count - self.last_pulse_count
                self.pulse_count = slef.last_pulse_count

                # Calculate instant RPM
                elapsed_ms = time.ticks_diff(current_time, self.last_measurement_time)
                rpm = (
                    (count / self.PULSES_PER_REV) * (60000 / elapsed_ms)
                    if elapsed_ms > 0
                    else 0
                )

                # Update buffers (shift values and add new one at the end)
                self.rpm_buffer.pop(0)
                self.rpm_buffer.append(rpm)
                self.time_buffer.pop(0)
                self.time_buffer.append(current_time)

                # Update last measurement time
                self.last_measurement_time = current_time

            # Small sleep to prevent CPU overload
            await asyncio.sleep(0.01)

    def start(self):
        """Start the background measurement task."""
        asyncio.create_task(self.run_background_measurement())

    def stop(self):
        """Stop the background measurement task."""
        self.is_running = False

    def get_rpm(self):
        """Returns the latest RPM value from the buffer."""
        return self.rpm_buffer[-1] if self.rpm_buffer else 0

    def get_rpm_buffer(self):
        """Returns the entire RPM buffer."""
        return self.rpm_buffer

    def get_speed(self):
        """Calculate speed based on the latest RPM."""
        rpm_mean = self.get_rpm()
        wheel_circumference = math.pi * self.WHEEL_DIAMETER
        # Speed in m/s = (RPM * circumference) / 60
        speed_m_s = (rpm_mean * wheel_circumference) / 60
        # Adjust speed based on direction
        return speed_m_s

    def get_absolute_acceleration(self):
        """Calculate acceleration based on the buffered RPM data."""
        rpms = self.rpm_buffer
        times = [t / 1000 for t in self.time_buffer]  # Convert to seconds
        wheel_circumference = math.pi * self.WHEEL_DIAMETER

        # Filter out zero times (avoid issues in the calculation)
        valid_data = [(t, r) for t, r in zip(times, rpms) if t > 0]

        if len(valid_data) < 2:
            return 0

        times = [t for t, _ in valid_data]
        rpms = [r for _, r in valid_data]

        # Calculate the slope (acceleration) using linear regression
        n = len(rpms)
        sum_x = sum(times)
        sum_y = sum(rpms)
        sum_xy = sum(x * y for x, y in zip(times, rpms))
        sum_x_squared = sum(x**2 for x in times)

        # Avoid division by zero
        if (n * sum_x_squared - sum_x**2) == 0:
            return 0

        slope = (n * sum_xy - sum_x * sum_y) / (n * sum_x_squared - sum_x**2)
        # Calculate the acceleration in m/s²
        acceleration = (slope * wheel_circumference) / 60
        # Adjust acceleration based on direction
        return acceleration
