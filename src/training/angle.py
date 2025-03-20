import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit
import sys
import time
import math

sys.path.append('..')
from motor import motorA, motorB
from accelerometer import mpu_init, read_accel, read_gyro

angles = []
motor_powers = []

def calculate_angle(ax, ay, az):
    """Get pitch angle from accelerometer."""
    return math.atan2(ay, az) * 180.0 / math.pi

def test_motor_power_for_angle(target_angle, current_power=0, step=2):
    """Find motor power to hold a target angle."""
    print(f"\n-- Target angle: {target_angle:.2f}° --")
    Kp, Ki = 1.0, 0.1
    angle_error_sum = 0
    max_iterations = 50
    iterations = 0
    stable_count = 0
    stable_threshold = 3
    angle_tolerance = 1.0
    previous_angles = []
    max_history = 5

    while iterations < max_iterations:
        iterations += 1
        ax, ay, az = read_accel()
        current_angle = calculate_angle(ax, ay, az)
        previous_angles.append(current_angle)
        if len(previous_angles) > max_history:
            previous_angles.pop(0)

        angle_error = target_angle - current_angle
        angle_error_sum += angle_error

        print(f"Iteration {iterations}: Angle: {current_angle:.2f}°, Target: {target_angle:.2f}°, Power: {current_power}%")

        if abs(angle_error) < angle_tolerance:
            if len(previous_angles) >= 3:
                angle_std = np.std(previous_angles)
                if angle_std < 0.5:
                    stable_count += 1
                else:
                    stable_count = 0

            if stable_count >= stable_threshold:
                print(f"Stable at {current_power}% for {target_angle:.2f}°")
                angles.append(current_angle)
                motor_powers.append(current_power)
                return current_power
        else:
            stable_count = 0

        adjustment = (Kp * angle_error) + (Ki * angle_error_sum)
        new_power = current_power + adjustment

        if abs(new_power - current_power) < step:
            if adjustment > 0:
                new_power = current_power + step
            elif adjustment < 0:
                new_power = current_power - step

        new_power = max(-100, min(100, new_power))

        if new_power != current_power:
            current_power = new_power
            motorA(current_power)
            motorB(current_power)
            time.sleep(0.5)

    print(f"No stable power found for {target_angle:.2f}°")
    motorA(0)
    motorB(0)
    return None

def linear_model(x, m, b):
    return m * x + b

def quadratic_model(x, a, b, c):
    return a * x**2 + b * x + c

def cubic_model(x, a, b, c, d):
    return a * x**3 + b * x**2 + c * x + d

def odd_cubic_model(x, a, c):
    return a * x**3 + c * x

def run_experiment():
    """Collect angle vs. motor power data."""
    mpu_init()
    time.sleep(1)
    print("=== Angle vs Motor Power Test ===")

    test_angles = [2, 5, 8, 10, 12, 15, 18, 20, 25]
    last_power = 0

    try:
        for target in test_angles:
            print(f"\nTesting {target}°")
            result = test_motor_power_for_angle(target, last_power)
            if result is not None:
                last_power = result
                motorA(0)
                motorB(0)
                time.sleep(1)
            else:
                print(f"Failed at {target}°. Stopping.")
                break
    except KeyboardInterrupt:
        print("Stopped by user.")
    finally:
        motorA(0)
        motorB(0)

    return angles, motor_powers

def analyze_and_plot(angles, motor_powers):
    """Fit data and create plots."""
    if len(angles) < 2:
        print("Not enough data.")
        return

    angles_array = np.array(angles)
    powers_array = np.array(motor_powers)

    popt_linear, _ = curve_fit(linear_model, angles_array, powers_array)
    m, b = popt_linear
    x_smooth = np.linspace(min(angles_array), max(angles_array), 100)
    y_linear = linear_model(x_smooth, m, b)

    plt.figure(figsize=(10, 6))
    plt.scatter(angles_array, powers_array, color='blue', label='Data')
    plt.plot(x_smooth, y_linear, 'r-', label=f'Linear: y={m:.2f}x+{b:.2f}')

    if len(angles) >= 3:
        popt_quad, _ = curve_fit(quadratic_model, angles_array, powers_array)
        a, bq, c = popt_quad
        y_quad = quadratic_model(x_smooth, a, bq, c)
        plt.plot(x_smooth, y_quad, 'g-', label='Quadratic')

        try:
            popt_odd_cubic, _ = curve_fit(odd_cubic_model, angles_array, powers_array)
            a_cubic, c_cubic = popt_odd_cubic
            y_odd_cubic = odd_cubic_model(x_smooth, a_cubic, c_cubic)
            plt.plot(x_smooth, y_odd_cubic, 'm-', label='Odd Cubic')
        except:
            print("Odd cubic fit failed.")

        if len(angles) >= 4:
            try:
                popt_cubic, _ = curve_fit(cubic_model, angles_array, powers_array)
                ac, bc, cc, dc = popt_cubic
                y_cubic = cubic_model(x_smooth, ac, bc, cc, dc)
                plt.plot(x_smooth, y_cubic, 'y-', label='Cubic')
            except:
                print("Full cubic fit failed.")

    plt.xlabel('Angle (°)')
    plt.ylabel('Motor Power (%)')
    plt.title('Angle vs Motor Power')
    plt.grid(True)
    plt.legend()
    plt.savefig('angle_vs_motor_power.png')
    print("Saved plot as 'angle_vs_motor_power.png'")
    np.savez('angle_motor_data.npz', angles=angles_array, powers=powers_array)
    print("Saved data as 'angle_motor_data.npz'")
    plt.show()
    return popt_linear

def predict_power_for_angle(angle, model_params):
    """Use linear model to predict power."""
    m, b = model_params
    return m * angle + b

if __name__ == "__main__":
    try:
        data = np.load('angle_motor_data.npz')
        print("Existing data found.")
        print("1: Use it")
        print("2: Collect new")
        choice = input("Choice (1/2): ")

        if choice == '1':
            angles = data['angles']
            motor_powers = data['powers']
            model_params = analyze_and_plot(angles, motor_powers)
            while True:
                try:
                    test_angle = float(input("Angle to predict (q to quit): "))
                    predicted_power = predict_power_for_angle(test_angle, model_params)
                    print(f"{test_angle:.2f}° -> {predicted_power:.2f}%")
                except ValueError:
                    break
            sys.exit(0)
    except (FileNotFoundError, IOError):
        pass

    print("Starting fresh data collection...")
    angles, motor_powers = run_experiment()
    if len(angles) > 0:
        model_params = analyze_and_plot(angles, motor_powers)
        print("Done!")