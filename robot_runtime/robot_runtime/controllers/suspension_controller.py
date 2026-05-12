import time

from std_msgs.msg import Float32MultiArray

from robot_runtime.controllers.exclusive_controller import ExclusiveController


class SuspensionController(ExclusiveController):
    """Bottom-layer suspension command publisher."""

    def __init__(self, node, context):
        super().__init__('suspension')
        self._node = node
        self._context = context
        self._pub_action = node.create_publisher(
            Float32MultiArray,
            't0x0102_action',
            10,
        )

    def stop(self, owner):
        self.require_owner(owner)
        if self._context.wheel_heights:
            return self._publish_heights(self._context.wheel_heights[:4], owner)
        return None

    def set_all_height(self, height, owner=None):
        return self._publish_heights([height, height, height, height], owner)

    def set_wheel_heights(self, heights, owner=None):
        return self._publish_heights(heights, owner)

    def set_front_height(self, height, owner=None):
        self.require_owner(owner)
        targets = self._current_or_default()
        targets[0] = float(height)
        targets[1] = float(height)
        return self._publish_heights(targets, owner)

    def set_rear_height(self, height, owner=None):
        self.require_owner(owner)
        targets = self._current_or_default()
        targets[2] = float(height)
        targets[3] = float(height)
        return self._publish_heights(targets, owner)

    def wait_height_reached(self, target, tolerance=20.0, timeout_sec=2.0):
        deadline = time.monotonic() + timeout_sec
        targets = _target_list(target)
        while time.monotonic() < deadline:
            heights = self._context.wheel_heights[:4]
            if len(heights) >= 4:
                reached = all(
                    abs(float(heights[i]) - targets[i]) <= tolerance
                    for i in range(4)
                )
                if reached:
                    return True
            time.sleep(0.01)
        return False

    def _current_or_default(self):
        if len(self._context.wheel_heights) >= 4:
            return [float(v) for v in self._context.wheel_heights[:4]]
        return [30.0, 30.0, 30.0, 30.0]

    def _publish_heights(self, heights, owner):
        self.require_owner(owner)
        msg = Float32MultiArray()
        msg.data = [float(height) for height in heights[:4]]
        self._context.suspension_target = list(msg.data)
        self._pub_action.publish(msg)
        return msg


def _target_list(target):
    if isinstance(target, (int, float)):
        value = float(target)
        return [value, value, value, value]
    values = [float(v) for v in target]
    if len(values) < 4:
        raise ValueError('target must be a scalar or four wheel heights')
    return values[:4]
