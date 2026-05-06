import time

import rclpy

from robot_runtime.runtime_node import RobotRuntimeNode
from robot_runtime.tasks import StepClimbConfig, StepClimbTask


DEFAULT_SPEED = 0.12
CONTROL_PERIOD = 0.01


class StepClimbForwardNode(RobotRuntimeNode):
    """Run the default forward step-climb task."""

    def __init__(self):
        super().__init__()
        self._task = StepClimbTask(
            self.core,
            StepClimbConfig.forward(speed=DEFAULT_SPEED),
        )
        self._task.reset()
        self._last_phase = None
        self._has_started_sequence = False
        self._timer = self.create_timer(CONTROL_PERIOD, self._tick)
        self.get_logger().info('Step climb forward task started.')

    def _tick(self):
        result = self._task.tick(self._pose_error(), now=time.monotonic())
        phase = result['suspension']['phase']

        if phase.name != 'IDLE':
            self._has_started_sequence = True
        if phase != self._last_phase:
            self.get_logger().info('Suspension phase: %s' % phase.name)
            self._last_phase = phase

        if phase.name == 'IDLE' and self._has_started_sequence:
            self.get_logger().info('Step climb forward task complete.')
            self._finish()

    def _pose_error(self):
        values = self.context.relative_pose_error
        if len(values) >= 3:
            return values[:3]
        return 0.0, 0.0, 0.0

    def _finish(self):
        self._timer.cancel()
        self.core.stop_all()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = StepClimbForwardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.core.stop_all()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
