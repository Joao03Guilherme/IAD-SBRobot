import json
import parameters.parameters as parameters

config_path = "parameters/config.json"
data = {}
with open(config_path, "r") as f:
    data = json.load(f)


def save_config():
    """
    Save the config.json file.
    """
    with open(config_path, "w") as f:
        json.dump(data, f, indent=4)


def change_kp(kp):
    """
    Change the kp value in the config.json file.
    """

    if not isinstance(kp, float):
        raise ValueError("kp must be a float")

    if kp < 0:
        raise ValueError("kp must be greater than or equal to 0")

    data["PID_CONFIG"]["kp"] = kp
    parameters.PID_CONFIG["kp"] = kp

    save_config()


def change_ki(ki):
    """
    Change the ki value in the config.json file.
    """

    if not isinstance(ki, float):
        raise ValueError("ki must be a float")

    if ki < 0:
        raise ValueError("ki must be greater than or equal to 0")

    data["PID_CONFIG"]["ki"] = ki
    parameters.PID_CONFIG["ki"] = ki

    save_config()


def change_kd(kd):
    """
    Change the kd value in the config.json file.
    """

    if not isinstance(kd, float):
        raise ValueError("kd must be a float")

    if kd < 0:
        raise ValueError("kd must be greater than or equal to 0")

    data["PID_CONFIG"]["kd"] = kd
    parameters.PID_CONFIG["kd"] = kd

    save_config()


def change_sample_time(sample_time):
    """
    Change the sample_time value in the config.json file.
    """

    if not isinstance(sample_time, float):
        raise ValueError("sample_time must be a float")

    if sample_time <= 0:
        raise ValueError("sample_time must be greater than 0")

    data["PID_CONFIG"]["sample_time"] = sample_time
    parameters.PID_CONFIG["sample_time"] = sample_time

    save_config()


def change_max_safe_tilt(max_safe_tilt):
    """
    Change the max_tilt value in the config.json file.
    """

    if not isinstance(max_safe_tilt, float):
        raise ValueError("max_safe_tilt must be a float")

    if max_safe_tilt < 0:
        raise ValueError("max_safe_tilt must be greater than or equal to 0")

    data["MAX_SAFE_TILT"] = max_safe_tilt
    parameters.MAX_SAFE_TILT = max_safe_tilt

    save_config()
