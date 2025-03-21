import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit
import sys
import time
import math
from collections import deque

sys.path.append('..')
from raspberry.motor_controller import MotorController
from raspberry.accelerometer import mpu_init, read_accel, read_gyro

ALPHA = 0.98  # Complementary filter weight (gyro)
DT = 0.01     # Timestep (s)

# Motor configuration for all test functions
MOTOR_CONFIG = {
    "IN1": 2,
    "IN2": 3,
    "IN3": 4,
    "IN4": 5,
    "ENA": 6,
    "ENB": 7,
    "PWM_FREQ": 1000
}

class DataRecorder:
    """Stores and saves angle & motor power data."""
    def __init__(self, max_size=1000):
        self.time = deque(maxlen=max_size)
        self.angles = deque(maxlen=max_size)
        self.gyro_angles = deque(maxlen=max_size)
        self.acc_angles = deque(maxlen=max_size)
        self.motor_powers = deque(maxlen=max_size)
        self.start_time = time.time()
    
    def record(self, angle, gyro_angle, acc_angle, motor_power):
        self.time.append(time.time() - self.start_time)
        self.angles.append(angle)
        self.gyro_angles.append(gyro_angle)
        self.acc_angles.append(acc_angle)
        self.motor_powers.append(motor_power)
    
    def save(self, filename):
        """Save collected data to a .npz file."""
        np.savez(filename,
                 time=np.array(self.time),
                 angles=np.array(self.angles),
                 gyro_angles=np.array(self.gyro_angles),
                 acc_angles=np.array(self.acc_angles),
                 motor_powers=np.array(self.motor_powers))
        print(f"Data saved: {filename}")

class AngleSensor:
    """Combines gyro and accel data to get an estimated angle."""
    def __init__(self):
        mpu_init()
        self.last_time = time.time()
        self.current_angle = 0
        self.gyro_angle = 0
        self.calibrate()
    
    def calibrate(self, samples=100):
        """Collect bias data for gyro and accel."""
        gyro_bias = [0, 0, 0]
        acc_bias = [0, 0, 0]
        for _ in range(samples):
            ax, ay, az = read_accel()
            gx, gy, gz = read_gyro()
            gyro_bias[0] += gx
            gyro_bias[1] += gy
            gyro_bias[2] += gz
            acc_bias[0] += ax
            acc_bias[1] += ay
            acc_bias[2] += az
            time.sleep(0.01)
        self.gyro_bias = [x/samples for x in gyro_bias]
        self.acc_bias = [x/samples for x in acc_bias]
        self.acc_bias[2] = 0  # Z is assumed close to gravity
        time.sleep(1)
    
    def update(self):
        """Returns (combined_angle, gyro_angle, raw_acc_angle)."""
        ax, ay, az = read_accel()
        gx, gy, gz = read_gyro()
        gx -= self.gyro_bias[0]
        gy -= self.gyro_bias[1]
        gz -= self.gyro_bias[2]
        ax -= self.acc_bias[0]
        ay -= self.acc_bias[1]
        dt = time.time() - self.last_time
        self.last_time = time.time()
        acc_angle = math.atan2(ay, az) * 180.0 / math.pi
        gyro_rate = gx / 131.0
        gyro_delta = gyro_rate * dt
        self.gyro_angle += gyro_delta
        self.current_angle = (ALPHA * (self.current_angle + gyro_delta)
                              + (1 - ALPHA) * acc_angle)
        return self.current_angle, self.gyro_angle, acc_angle

def test_motor_power_for_angle(angle_sensor, target_angle, current_power=0, step=2):
    """Attempt to find motor power required for a specific target angle."""
    print(f"Target angle: {target_angle:.2f}°")
    
    # Initialize motor controller
    motors = MotorController(MOTOR_CONFIG)
    
    Kp, Ki, Kd = 1.2, 0.1, 0.05
    angle_error_sum = 0
    last_error = 0
    max_iterations = 100
    stable_count = 0
    stable_threshold = 5
    angle_tolerance = 1.0
    recorder = DataRecorder()
    for _ in range(1, max_iterations + 1):
        current_angle, gyro_angle, acc_angle = angle_sensor.update()
        angle_error = target_angle - current_angle
        angle_error_sum = max(-300, min(300, angle_error_sum + angle_error))
        p_term = Kp * angle_error
        i_term = Ki * angle_error_sum * DT
        d_term = Kd * (angle_error - last_error) / DT
        recorder.record(current_angle, gyro_angle, acc_angle, current_power)
        if abs(angle_error) < angle_tolerance:
            stable_count += 1
            if stable_count >= stable_threshold:
                print(f"Stable at {current_power}% (Angle: {current_angle:.2f}°)")
                recorder.save(f"angle_test_{target_angle:.1f}_deg.npz")
                visualize_test(recorder, target_angle)
                motors.stop()
                return current_angle, current_power
        else:
            stable_count = 0
        adjustment = p_term + i_term + d_term
        new_power = current_power + adjustment
        if 0 < abs(adjustment) < step:
            new_power = current_power + (step if adjustment > 0 else -step)
        new_power = max(-100, min(100, new_power))
        if new_power != current_power:
            current_power = new_power
            motors.motor_a(current_power)
            motors.motor_b(current_power)
            time.sleep(0.1)
        last_error = angle_error
    print(f"No stable power found for {target_angle:.2f}°")
    motors.stop()
    recorder.save(f"angle_test_{target_angle:.1f}_deg_unstable.npz")
    return None, None

def visualize_test(recorder, target_angle):
    """Plot angle and motor power during the test."""
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
    ax1.plot(recorder.time, recorder.angles, label='Combined')
    ax1.plot(recorder.time, recorder.gyro_angles, alpha=0.5, label='Gyro')
    ax1.plot(recorder.time, recorder.acc_angles, alpha=0.5, label='Acc')
    ax1.axhline(y=target_angle, color='k', linestyle='--', label='Target')
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Angle (°)')
    ax1.legend()
    ax1.grid(True)
    ax2.plot(recorder.time, recorder.motor_powers, 'r-')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Motor Power (%)')
    ax2.grid(True)
    plt.tight_layout()
    plt.savefig(f'angle_test_{target_angle:.1f}_deg.png')
    plt.show()

def run_experiment():
    """Perform tests at multiple angles and analyze results."""
    angle_sensor = AngleSensor()
    time.sleep(1)
    tests = [1, 3, 5, 8, 10, 12, 15]
    results = {'angles': [], 'powers': []}
    
    # Initialize motor controller once for all tests
    motors = MotorController(MOTOR_CONFIG)
    
    for angle in tests:
        a, p = test_motor_power_for_angle(angle_sensor, angle)
        if a is not None and p is not None:
            results['angles'].append(a)
            results['powers'].append(p)
            motors.stop()
            time.sleep(1)
    
    # Make sure motors are stopped at the end
    motors.stop()
    
    if results['angles']:
        np.savez('angle_power_results.npz', angles=results['angles'], powers=results['powers'])
        analyze_results(results['angles'], results['powers'])
    return results

def analyze_results(angles, powers):
    """Fit curves to angle-power data and plot results."""
    if len(angles) < 2:
        print("Not enough data.")
        return
    angles, powers = np.array(angles), np.array(powers)
    models = {
        'linear': (lambda x, a, b: a*x + b, 'y = {:.2f}x + {:.2f}'),
        'quadratic': (lambda x, a, b, c: a*x**2 + b*x + c, 'y = {:.2f}x² + {:.2f}x + {:.2f}'),
        'cubic': (lambda x, a, b, c, d: a*x**3 + b*x**2 + c*x + d, 'y = {:.2f}x³ + {:.2f}x² + {:.2f}x + {:.2f}')
    }
    plt.figure(figsize=(12, 8))
    plt.scatter(angles, powers, color='blue', label='Data')
    x_smooth = np.linspace(min(angles), max(angles), 100)
    for name, (f_model, lbl_fmt) in models.items():
        try:
            params, _ = curve_fit(f_model, angles, powers)
            y_pred = f_model(x_smooth, *params)
            label_text = lbl_fmt.format(*params)
            plt.plot(x_smooth, y_pred, label=f"{name}: {label_text}")
            y_mean = np.mean(powers)
            ss_tot = np.sum((powers - y_mean)**2)
            ss_res = np.sum((powers - f_model(angles, *params))**2)
            r2 = 1 - (ss_res / ss_tot)
            print(f"{name.capitalize()} R²: {r2:.4f}")
            np.savez(f'angle_power_{name}_model.npz', params=params, r_squared=r2)
        except:
            pass
    plt.xlabel('Angle (°)')
    plt.ylabel('Motor Power (%)')
    plt.title('Angle vs. Motor Power')
    plt.grid(True)
    plt.legend()
    plt.savefig('angle_vs_power_analysis.png')
    plt.show()

if __name__ == "__main__":
    try:
        try:
            data = np.load('angle_power_results.npz')
            print("1: Analyze existing data\n2: Run new experiment")
            choice = input("Choice (1/2): ")
            if choice == '1':
                analyze_results(data['angles'], data['powers'])
                sys.exit(0)
        except:
            pass
        print("Starting new experiment...")
        run_experiment()
    except Exception as e:
        print(f"Error: {e}")