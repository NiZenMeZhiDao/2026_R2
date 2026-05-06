import time

import rclpy

from robot_runtime.runtime_node import RobotRuntimeNode
from robot_runtime.tasks import StepClimbConfig, StepClimbTask


STEP_CLIMB_CONFIG = StepClimbConfig.forward(
    speed=0.5,
    control_period=0.01,
)


def main(args=None):
    rclpy.init(args=args)
    node = RobotRuntimeNode()
    task = StepClimbTask(node.core, STEP_CLIMB_CONFIG)
    task.reset()

    node.get_logger().info('Step climb forward task started.')
    try:
        while rclpy.ok() and not task.is_done():
            now = time.monotonic()
            rclpy.spin_once(node, timeout_sec=0.0)
            result = task.tick(_pose_error(node), now=now)
            _log_phase_change(node, result)
            time.sleep(task.config.control_period)

        node.get_logger().info('Step climb forward task complete.')
    except KeyboardInterrupt:
        pass
    finally:
        task.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def _pose_error(node):
    values = node.context.relative_pose_error
    if len(values) >= 3:
        return values[:3]
    return 0.0, 0.0, 0.0


def _log_phase_change(node, result):
    phase = result['suspension']['phase']
    previous_phase = getattr(node, '_step_climb_phase', None)
    if phase != previous_phase:
        node.get_logger().info('Suspension phase: %s' % phase.name)
        node._step_climb_phase = phase


if __name__ == '__main__':
    main()
