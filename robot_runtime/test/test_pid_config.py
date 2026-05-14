import json
from types import SimpleNamespace

from robot_runtime.pid_config import (
    PID_CONFIG_ENV,
    chassis_deadzone,
    pid_gains,
    reload_pid_config,
)
from robot_runtime.tasks.chassis_pid_task import ChassisPidTask
from robot_runtime.tasks.step_climb_task import StepClimbConfig


def test_pid_config_loads_step_climb_gains_from_env_file(tmp_path, monkeypatch):
    path = tmp_path / 'pid.json'
    path.write_text(
        json.dumps({
            'step_climb': {
                'y': {
                    'kp': 2.0,
                    'ki': 0.1,
                    'kd': 0.2,
                    'output_limit': 0.3,
                    'integral_limit': 0.4,
                }
            }
        }),
        encoding='utf-8',
    )
    monkeypatch.setenv(PID_CONFIG_ENV, str(path))
    reload_pid_config()

    gains = pid_gains('step_climb', 'y')

    assert gains.kp == 2.0
    assert gains.ki == 0.1
    assert gains.kd == 0.2
    assert gains.output_limit == 0.3
    assert gains.integral_limit == 0.4


def test_step_climb_config_uses_pid_config_defaults():
    reload_pid_config()

    config = StepClimbConfig.forward()

    assert config.y_gains.kp == pid_gains('step_climb', 'y').kp
    assert config.wz_gains.kp == pid_gains('step_climb', 'wz').kp


def test_chassis_deadzone_loads_from_config_file(tmp_path, monkeypatch):
    path = tmp_path / 'pid.json'
    path.write_text(
        json.dumps({
            'chassis_deadzone': {
                'xy': 0.2,
                'wz': 0.3,
            }
        }),
        encoding='utf-8',
    )
    monkeypatch.setenv(PID_CONFIG_ENV, str(path))
    reload_pid_config()

    assert chassis_deadzone() == {'xy': 0.2, 'wz': 0.3}


def test_chassis_pid_task_accepts_explicit_test_gains():
    gains = SimpleNamespace(
        kp=1.0,
        ki=0.0,
        kd=0.0,
        output_limit=10.0,
        integral_limit=0.0,
    )
    task = ChassisPidTask(x_gains=gains, y_gains=gains, yaw_gains=gains)

    cmd = task.compute_cmd((1.0, -2.0, 0.5), now=1.0)

    assert cmd.linear.x == 1.0
    assert cmd.linear.y == -2.0
    assert cmd.angular.z == 0.5
