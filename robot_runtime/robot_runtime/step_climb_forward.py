import time

from robot_runtime.libraries.pid import wrap_angle
from robot_runtime.tasks import StepClimbConfig, StepClimbTask


MOVE_TIMEOUT = 10.0


def main(args=None):
    import rclpy

    from robot_runtime.runtime_node import RobotRuntimeNode

    rclpy.init(args=args)
    node = RobotRuntimeNode()
    try:
        run_task(node.core)
    except KeyboardInterrupt:
        pass
    finally:
        node.core.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def run_task(core):
    """Top-level task example: edit this function into the real field routine."""
    _wait_until_ready(core, timeout=30.0)
    if not _localization_ready(core):
        _log_info(core, 'Skip routine: no localization within timeout')
        core.stop()
        return

    core.set_stepmode(False)
    core.set_height(30.0)

    # Move to the first waypoint, then climb forward.
    if not _move_to_or_stop(core, x=1.00, y=0.00, theta=0.00):
        return
    climb_step(core, StepClimbConfig.forward(speed=0.5))
    core.set_stepmode(False)
    core.set_height(30.0)

    # Move to the second waypoint, then climb forward again.
    if not _move_to_or_stop(core, x=1.20, y=0.00, theta=1.5708):
        return
    climb_step(core, StepClimbConfig.forward(speed=0.5))
    core.set_stepmode(False)
    core.set_height(30.0)

    # Move to the third waypoint, then climb forward again.
    if not _move_to_or_stop(core, x=1.20, y=1.20, theta=0):
        return
    climb_step(core, StepClimbConfig.forward(speed=0.5))
    core.set_stepmode(False)
    core.set_height(30.0)

    core.stop()


def climb_step(core, config):
    task = StepClimbTask(core, config)
    task.reset()
    reference_pose = _reference_pose_if_ready(core)
    deadline = time.monotonic() + float(config.timeout)
    try:
        while not task.is_done():
            _spin_core_once(core)
            if reference_pose is None:
                reference_pose = _reference_pose_if_ready(core)
            result = task.tick(
                _pose_error_from_reference(core, reference_pose, config.move_direction),
                now=time.monotonic(),
            )
            _log_phase(core, result)
            if time.monotonic() >= deadline:
                raise TimeoutError('step climb timed out')
            time.sleep(config.control_period)
    finally:
        task.stop()


def _current_pose_xytheta(core):
    map_ready = bool(getattr(core.context, 'map_ready', False))
    localization_ready = bool(getattr(core.context, 'localization_ready', False))
    pose = list(getattr(core.context, 'robot_pose_map_xytheta', []))
    if (map_ready or localization_ready) and len(pose) >= 3:
        return float(pose[0]), float(pose[1]), float(pose[2])
    odom_pose = list(getattr(core.context, 'robot_pose_odom_xytheta', []))
    if len(odom_pose) >= 3:
        return float(odom_pose[0]), float(odom_pose[1]), float(odom_pose[2])
    return (
        float(getattr(core.context, 'odom_x', 0.0)),
        float(getattr(core.context, 'odom_y', 0.0)),
        float(getattr(core.context, 'odom_theta', 0.0)),
    )


def _pose_error_from_reference(core, reference_pose, move_direction):
    if reference_pose is None:
        return 0.0, 0.0, 0.0

    current = _current_pose_xytheta(core)
    dx = reference_pose[0] - current[0]
    dy = reference_pose[1] - current[1]
    dtheta = wrap_angle(reference_pose[2] - current[2])
    cos_heading = _cos(current[2])
    sin_heading = _sin(current[2])
    forward_error = cos_heading * dx + sin_heading * dy
    left_error = -sin_heading * dx + cos_heading * dy
    return forward_error, left_error, dtheta


def _reference_pose_if_ready(core):
    if not _localization_ready(core):
        return None
    return _current_pose_xytheta(core)


def _log_phase(core, result):
    phase = result['suspension']['phase']
    previous_phase = getattr(core, '_step_climb_phase', None)
    if phase != previous_phase:
        node = getattr(core, '_node', None)
        if node is not None:
            node.get_logger().info('Suspension phase: %s' % phase.name)
        core._step_climb_phase = phase


def _move_to_or_stop(core, *, x, y, theta, frame=None, timeout=MOVE_TIMEOUT):
    try:
        core.move_to(x=x, y=y, theta=theta, timeout=timeout, frame=frame)
        return True
    except TimeoutError as exc:
        _log_info(core, 'Stop task: %s' % exc)
        core.stop()
        return False


def _wait_until_ready(core, timeout=30.0):
    import rclpy

    deadline = time.monotonic() + float(timeout)
    while not _localization_ready(core):
        if time.monotonic() >= deadline:
            _log_info(core, 'Wait for localization timed out')
            return
        if hasattr(rclpy, 'ok') and not rclpy.ok():
            return
        _spin_core_once(core)
        time.sleep(0.1)


def _localization_ready(core):
    context = getattr(core, 'context', None)
    if context is None:
        return False
    return bool(context.map_ready or context.localization_ready or context.odom_ready)


def _log_info(core, message):
    node = getattr(core, '_node', None)
    if node is not None:
        node.get_logger().info(str(message))


def _spin_core_once(core):
    node = getattr(core, '_node', None)
    if node is None:
        return
    try:
        import rclpy
    except ImportError:
        return
    if hasattr(rclpy, 'ok') and not rclpy.ok():
        return
    # Process a batch of pending callbacks so the suspension timer is not
    # starved by high-frequency sensor subscriptions competing for spin_once.
    for _ in range(6):
        rclpy.spin_once(node, timeout_sec=0.0)


def _cos(value):
    import math

    return math.cos(float(value))


def _sin(value):
    import math

    return math.sin(float(value))


if __name__ == '__main__':
    main()
