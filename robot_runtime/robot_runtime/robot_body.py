from robot_runtime.controllers.chassis_controller import ChassisController
from robot_runtime.controllers.suspension_controller import SuspensionController


class RobotBody:
    """Unified bottom-layer hardware entry point."""

    def __init__(self, node, context):
        self.chassis = ChassisController(node)
        self.suspension = SuspensionController(node, context)

    def stop_all(self, owner):
        self.chassis.stop(owner)
        self.suspension.stop(owner)
