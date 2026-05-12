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
    # Go to the forward stair start point, then climb forward.
    core.move_to(x=1.00, y=0.00, theta=0.00, timeout=MOVE_TIMEOUT)
    climb_step(core, StepClimbConfig.forward(speed=0.5))

    # Example of a simple mode switch between navigation segments.
    core.set_stepmode(False)
    core.set_height(30.0)

    # Go to the left stair start point, then climb left.
    core.move_to(x=1.00, y=0.60, theta=1.5708, timeout=MOVE_TIMEOUT)
    climb_step(core, StepClimbConfig.left(speed=0.35, base_vx=0.0))

    # Go to the right stair start point, then climb right.
    core.move_to(x=1.00, y=-0.60, theta=-1.5708, timeout=MOVE_TIMEOUT)
    climb_step(core, StepClimbConfig.right(speed=0.35, base_vx=0.0))

    core.stop()


def climb_step(core, config):
    task = StepClimbTask(core, config)
    task.reset()
    reference_pose = _current_pose_xytheta(core)
    deadline = time.monotonic() + float(config.timeout)
    try:
        while not task.is_done():
            _spin_core_once(core)
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


def _pose_error(core):
    values = core.context.relative_pose_error
    if len(values) >= 3:
        return values[:3]
    return 0.0, 0.0, 0.0


def _current_pose_xytheta(core):
    pose = list(getattr(core.context, 'robot_pose_map_xytheta', []))
    if len(pose) >= 3:
        return float(pose[0]), float(pose[1]), float(pose[2])
    return (
        float(getattr(core.context, 'robot_x', 0.0)),
        float(getattr(core.context, 'robot_y', 0.0)),
        float(getattr(core.context, 'robot_theta', 0.0)),
    )


def _pose_error_from_reference(core, reference_pose, move_direction):
    current = _current_pose_xytheta(core)
    dx = reference_pose[0] - current[0]
    dy = reference_pose[1] - current[1]
    dtheta = wrap_angle(reference_pose[2] - current[2])

    if int(move_direction) == 0:
        return 0.0, dy, dtheta

    return 0.0, dx, dtheta


def _log_phase(core, result):
    phase = result['suspension']['phase']
    previous_phase = getattr(core, '_step_climb_phase', None)
    if phase != previous_phase:
        node = getattr(core, '_node', None)
        if node is not None:
            node.get_logger().info('Suspension phase: %s' % phase.name)
        core._step_climb_phase = phase


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
    rclpy.spin_once(node, timeout_sec=0.0)


if __name__ == '__main__':
    main()
