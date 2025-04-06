from controllers.balance_controller import Driving
import time


def test_balance():
    """Simple test of the balancing behavior."""
    driver = Driving()
    print("Starting balance test...")
    print("Use Ctrl+C to stop")

    try:
        while True:
            angle, left, right, speed, accel = driver.forward()
            print(
                f"Angle: {angle:6.2f}° | Left: {left:4.0f}% | Right: {right:4.0f}% | Speed: {speed:.2f} m/s | Accel: {accel:.2f} m/s²"
            )
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        driver.stop()


if __name__ == "__main__":
    test_balance()
