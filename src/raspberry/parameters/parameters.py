import json
from typing import Any

config_path: str = "parameters/config.json"
data: dict[str, Any] = {}
with open(config_path, "r") as f:
    data = json.load(f)

def save_config() -> None:
    """
    Save the current state of the data dictionary to the config.json file.
    
    Args:
        None
    Returns:
        None
    """
    with open(config_path, "w") as f:
        json.dump(data, f)

"""
FOR EVERY CHANGE FUNCTION, THERE MUST BE A CHANGE IN DATA DICT AND SAVE CONFIG
"""

def change_kp(kp: float) -> bool:
    """
    Change the kp value in the config.json file.

    Args:
        kp (float): The new proportional gain value.
    Returns:
        bool: True if the value was updated successfully.
    Raises:
        ValueError: If kp is not a float or is negative.
    """
    if not isinstance(kp, float):
        raise ValueError("kp must be a float")
    if kp < 0:
        raise ValueError("kp must be greater than or equal to 0")
    data["PID_CONFIG"]["kp"] = kp
    save_config()
    return True

def change_ki(ki: float) -> bool:
    """
    Change the ki value in the config.json file.

    Args:
        ki (float): The new integral gain value.
    Returns:
        bool: True if the value was updated successfully.
    Raises:
        ValueError: If ki is not a float or is negative.
    """
    if not isinstance(ki, float):
        raise ValueError("ki must be a float")
    if ki < 0:
        raise ValueError("ki must be greater than or equal to 0")
    data["PID_CONFIG"]["ki"] = ki
    save_config()
    return True

def change_kd(kd: float) -> bool:
    """
    Change the kd value in the config.json file.

    Args:
        kd (float): The new derivative gain value.
    Returns:
        bool: True if the value was updated successfully.
    Raises:
        ValueError: If kd is not a float or is negative.
    """
    if not isinstance(kd, float):
        raise ValueError("kd must be a float")
    if kd < 0:
        raise ValueError("kd must be greater than or equal to 0")
    data["PID_CONFIG"]["kd"] = kd
    save_config()
    return True

def change_k_damping(k_damping: float) -> bool:
    """
    Change the k_damping value in the config.json file.

    Args:
        k_damping (float): The new damping coefficient.
    Returns:
        bool: True if the value was updated successfully.
    Raises:
        ValueError: If k_damping is not a float or is negative.
    """
    if not isinstance(k_damping, float):
        raise ValueError("k_damping must be a float")
    if k_damping < 0:
        raise ValueError("k_damping must be greater than or equal to 0")
    data["PID_CONFIG"]["k_damping"] = k_damping
    save_config()
    return True

def change_balance2offset(balance2offset: float) -> bool:
    """
    Change the balance2offset value in the config.json file.

    Args:
        balance2offset (float): The new balance offset value.
    Returns:
        bool: True if the value was updated successfully.
    Raises:
        ValueError: If balance2offset is not a float.
    """
    if not isinstance(balance2offset, float):
        raise ValueError("balance2offset must be a float")
    data["PID_CONFIG"]["balance2offset"] = balance2offset
    save_config()
    return True

def change_sample_time(sample_time: float) -> bool:
    """
    Change the sample_time value in the config.json file.

    Args:
        sample_time (float): The new sample time value.
    Returns:
        bool: True if the value was updated successfully.
    Raises:
        ValueError: If sample_time is not a float or is not positive.
    """
    if not isinstance(sample_time, float):
        raise ValueError("sample_time must be a float")
    if sample_time <= 0:
        raise ValueError("sample_time must be greater than 0")
    data["PID_CONFIG"]["sample_time"] = sample_time
    save_config()
    return True

def change_alpha(alpha: float) -> bool:
    """
    Change the alpha value in the config.json file.

    Args:
        alpha (float): The new alpha value for the complementary filter.
    Returns:
        bool: True if the value was updated successfully.
    Raises:
        ValueError: If alpha is not a float or is negative.
    """
    if not isinstance(alpha, float):
        raise ValueError("alpha must be a float")
    if alpha < 0:
        raise ValueError("alpha must be greater than or equal to 0")
    data["PID_CONFIG"]["alpha"] = alpha
    save_config()
    return True

def change_max_correction_tilt(max_correction_tilt: float) -> bool:
    """
    Change the max_correction_tilt value in the config.json file.

    Args:
        max_correction_tilt (float): The new maximum correction tilt value.
    Returns:
        bool: True if the value was updated successfully.
    Raises:
        ValueError: If max_correction_tilt is not a float or is negative.
    """
    if not isinstance(max_correction_tilt, float):
        raise ValueError("max_correction_tilt must be a float")
    if max_correction_tilt < 0:
        raise ValueError("max_correction_tilt must be greater than or equal to 0")
    data["MAX_CORRECTION_TILT"] = max_correction_tilt
    save_config()
    return True

def change_max_safe_tilt(max_safe_tilt: float) -> bool:
    """
    Change the max_safe_tilt value in the config.json file.

    Args:
        max_safe_tilt (float): The new maximum safe tilt value.
    Returns:
        bool: True if the value was updated successfully.
    Raises:
        ValueError: If max_safe_tilt is not a float or is negative.
    """
    if not isinstance(max_safe_tilt, float):
        raise ValueError("max_safe_tilt must be a float")
    if max_safe_tilt < 0:
        raise ValueError("max_safe_tilt must be greater than or equal to 0")
    data["MAX_SAFE_TILT"] = max_safe_tilt
    save_config()
    return True

def change_gyro_bias(bias_gx: float, bias_gy: float, bias_gz: float) -> None:
    """
    Change the gyro bias values in the config.json file.

    Args:
        bias_gx (float): Gyroscope X-axis bias.
        bias_gy (float): Gyroscope Y-axis bias.
        bias_gz (float): Gyroscope Z-axis bias.
    Returns:
        None
    Raises:
        ValueError: If any bias value is not a float.
    """
    if not isinstance(bias_gx, float):
        raise ValueError("bias_x must be a float")
    if not isinstance(bias_gy, float):
        raise ValueError("bias_y must be a float")
    if not isinstance(bias_gz, float):
        raise ValueError("bias_z must be a float")
    data["MPU_CONFIG"]["bias_gx"] = bias_gx
    data["MPU_CONFIG"]["bias_gy"] = bias_gy
    data["MPU_CONFIG"]["bias_gz"] = bias_gz
    save_config()

def change_accel_bias(bias_ax: float, bias_ay: float, bias_az: float) -> None:
    """
    Change the accelerometer bias values in the config.json file.

    Args:
        bias_ax (float): Accelerometer X-axis bias.
        bias_ay (float): Accelerometer Y-axis bias.
        bias_az (float): Accelerometer Z-axis bias.
    Returns:
        None
    Raises:
        ValueError: If any bias value is not a float.
    """
    if not isinstance(bias_ax, float):
        raise ValueError("bias_ax must be a float")
    if not isinstance(bias_ay, float):
        raise ValueError("bias_ay must be a float")
    data["MPU_CONFIG"]["bias_ax"] = bias_ax
    data["MPU_CONFIG"]["bias_ay"] = bias_ay
    data["MPU_CONFIG"]["bias_az"] = bias_az
    save_config()
