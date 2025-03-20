from machine import Pin, PWM
import time

testpin = Pin(15, Pin.OUT)

# Define motor direction pins as digital outputs
IN1 = Pin(10, Pin.OUT)
IN2 = Pin(11, Pin.OUT)
IN3 = Pin(12, Pin.OUT)
IN4 = Pin(13, Pin.OUT)

# Define ENA and ENB as PWM for speed control
ENA = PWM(Pin(14))  # Motor A Speed
ENB = PWM(Pin(15))  # Motor B Speed

# Set PWM frequency (1 kHz is good for motors)
FREQ = 1000
ENA.freq(FREQ)
ENB.freq(FREQ)

# Function to control Motor A
def motorA(speed):
    """ Controls Motor A:
        speed > 0 → Forward
        speed < 0 → Backward
        speed = 0 → Stop
    """
    duty = int(abs(speed) * 65535 / 100)  # Convert speed (0-100%) to PWM range
    
    if speed > 0:
        IN1.value(1)
        IN2.value(0)
    elif speed < 0:
        IN1.value(0)
        IN2.value(1)
    else:
        IN1.value(0)
        IN2.value(0)  # Stop motor

    ENA.duty_u16(duty)  # Set speed via ENA PWM

# Function to control Motor B
def motorB(speed):
    """ Controls Motor B:
        speed > 0 → Forward
        speed < 0 → Backward
        speed = 0 → Stop
    """
    duty = int(abs(speed) * 65535 / 100)

    if speed > 0:
        IN3.value(1)
        IN4.value(0)
    elif speed < 0:
        IN3.value(0)
        IN4.value(1)
    else:
        IN3.value(0)
        IN4.value(0)  # Stop motor

    ENB.duty_u16(duty)  # Set speed via ENB PWM

# 🚗 **Full Motor Test Sequence**
if __name__ == "__main__":
    print("Starting pin setup...")
    testpin.value(1)
    time.sleep(10)
    testpin.value(0)

    print("Starting motor B test")

    # Test Motor B
    motorB(100)  # Full speed forward
    time.sleep(2)
    motorB(50)  # Half speed forward
    time.sleep(2)
    motorB(0)   # Stop
    time.sleep(2)
    motorB(-50) # Half speed backward
    time.sleep(2)
    motorB(-100) # Full speed backward
    time.sleep(2)
    motorB(0)   # Stop
    time.sleep(2)

    
