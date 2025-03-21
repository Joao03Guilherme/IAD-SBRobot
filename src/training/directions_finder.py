import sys
import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Access sensor code
sys.path.append('..')
from raspberry.accelerometer import mpu_init, read_accel, read_gyro

# Initialize sensor, wait for stability
mpu_init()
time.sleep(1)

# Arrays for storing sensor data
timestamps = []
acc_x_data, acc_y_data, acc_z_data = [], [], []
gyro_x_data, gyro_y_data, gyro_z_data = [], [], []

plt.style.use('ggplot')
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10))
fig.suptitle('MPU6050 Sensor Direction Test')

start_time = time.time()

def test_directions():
    """Perform ordered tests to identify sensor axes."""
    input("Press Enter to begin the test...")
    do_test(5, "Tilt Forward")
    do_test(5, "Tilt Backward")
    do_test(5, "Tilt Left")
    do_test(5, "Tilt Right")
    do_test(5, "Rotate Clockwise")
    do_test(5, "Rotate CounterClockwise")
    print("Tests complete.")

def do_test(duration, label):
    """Collect and store sensor data for a certain movement."""
    timestamps.clear()
    acc_x_data.clear(), acc_y_data.clear(), acc_z_data.clear()
    gyro_x_data.clear(), gyro_y_data.clear(), gyro_z_data.clear()
    start = time.time()
    while (time.time() - start) < duration:
        t_now = time.time() - start
        timestamps.append(t_now)
        ax, ay, az = read_accel()
        gx, gy, gz = read_gyro()
        acc_x_data.append(ax)
        acc_y_data.append(ay)
        acc_z_data.append(az)
        gyro_x_data.append(gx)
        gyro_y_data.append(gy)
        gyro_z_data.append(gz)
        time.sleep(0.05)
    plot_test_data(label)

def plot_test_data(label):
    """Plot both accelerometer and gyroscope data."""
    ax1.clear()
    ax2.clear()
    ax1.plot(timestamps, acc_x_data, 'r-', label='AccX')
    ax1.plot(timestamps, acc_y_data, 'g-', label='AccY')
    ax1.plot(timestamps, acc_z_data, 'b-', label='AccZ')
    ax1.set_title(f'Accelerometer - {label}')
    ax1.legend(); ax1.grid(True)
    ax2.plot(timestamps, gyro_x_data, 'r-', label='GyroX')
    ax2.plot(timestamps, gyro_y_data, 'g-', label='GyroY')
    ax2.plot(timestamps, gyro_z_data, 'b-', label='GyroZ')
    ax2.set_title(f'Gyroscope - {label}')
    ax2.legend(); ax2.grid(True)
    plt.tight_layout()
    plt.savefig(f'sensor_test_{label.replace(" ", "_").lower()}.png')
    plt.pause(0.1)

def analyze_results():
    """Print basic instructions to interpret the plots."""
    print("Analyze plots to see which axis changes for each movement.")

if __name__ == "__main__":
    try:
        test_directions()
        analyze_results()
    except KeyboardInterrupt:
        print("Interrupted.")
    finally:
        plt.show()