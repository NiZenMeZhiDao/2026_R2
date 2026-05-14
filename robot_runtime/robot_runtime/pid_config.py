import json
import os
from functools import lru_cache

from robot_runtime.libraries.pid import PidGains


PID_CONFIG_ENV = 'ROBOT_RUNTIME_PID_CONFIG'


_DEFAULT_CONFIG = {
    'chassis_pose': {
        'x': {
            'kp': 0.8,
            'ki': 0.0,
            'kd': 0.05,
            'output_limit': 0.5,
            'integral_limit': 0.5,
        },
        'y': {
            'kp': 0.8,
            'ki': 0.0,
            'kd': 0.05,
            'output_limit': 0.5,
            'integral_limit': 0.5,
        },
        'yaw': {
            'kp': 1.2,
            'ki': 0.0,
            'kd': 0.08,
            'output_limit': 1.2,
            'integral_limit': 0.5,
        },
    },
    'step_climb': {
        'y': {
            'kp': 0.8,
            'ki': 0.0,
            'kd': 0.05,
            'output_limit': 0.5,
            'integral_limit': 0.5,
        },
        'wz': {
            'kp': 1.2,
            'ki': 0.0,
            'kd': 0.08,
            'output_limit': 1.2,
            'integral_limit': 0.5,
        },
    },
    'chassis_deadzone': {
        'xy': 0.12,
        'wz': 0.15,
    },
}


def pid_gains(group, axis):
    """Return PID gains from the runtime config file."""
    config = load_pid_config()
    try:
        values = config[group][axis]
    except KeyError as exc:
        raise KeyError('missing PID config for %s.%s' % (group, axis)) from exc
    return _pid_gains_from_mapping(values)


def chassis_deadzone():
    """Return chassis feed-forward deadzone compensation values."""
    config = load_pid_config()
    values = config.get('chassis_deadzone', {})
    return {
        'xy': float(values.get('xy', 0.0)),
        'wz': float(values.get('wz', 0.0)),
    }


@lru_cache(maxsize=1)
def load_pid_config():
    config = _deep_copy(_DEFAULT_CONFIG)
    path = _find_pid_config_path()
    if path is None:
        return config
    with open(path, 'r', encoding='utf-8') as stream:
        loaded = json.load(stream)
    if not isinstance(loaded, dict):
        raise ValueError('PID config root must be an object')
    _merge_dict(config, loaded)
    return config


def reload_pid_config():
    """Clear the PID config cache; useful for tests and live tuning scripts."""
    load_pid_config.cache_clear()


def _find_pid_config_path():
    env_path = os.environ.get(PID_CONFIG_ENV)
    if env_path:
        return env_path

    candidates = [
        os.path.join(_package_share_dir(), 'config', 'pid.json'),
        os.path.join(_source_package_dir(), '..', 'config', 'pid.json'),
    ]
    for path in candidates:
        normalized = os.path.abspath(path)
        if os.path.exists(normalized):
            return normalized
    return None


def _package_share_dir():
    try:
        from ament_index_python.packages import get_package_share_directory
    except ImportError:
        return ''
    try:
        return get_package_share_directory('robot_runtime')
    except Exception:
        return ''


def _source_package_dir():
    return os.path.dirname(os.path.abspath(__file__))


def _pid_gains_from_mapping(values):
    return PidGains(
        kp=float(values['kp']),
        ki=float(values['ki']),
        kd=float(values['kd']),
        output_limit=float(values['output_limit']),
        integral_limit=float(values.get('integral_limit', 0.0)),
    )


def _merge_dict(target, source):
    for key, value in source.items():
        if isinstance(value, dict) and isinstance(target.get(key), dict):
            _merge_dict(target[key], value)
        else:
            target[key] = value


def _deep_copy(value):
    return json.loads(json.dumps(value))
