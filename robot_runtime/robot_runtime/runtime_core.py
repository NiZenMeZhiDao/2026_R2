import collections
import math

from robot_runtime.libraries.suspension_math import SuspensionMath, SuspensionPhase
from robot_runtime.robot_body import RobotBody
from robot_runtime.robot_context import RobotContext
from robot_runtime.tasks.chassis_pid_task import ChassisPidTask


class RuntimeCore:
    """Code-call API used by the middle layer."""

    def __init__(self, node):
        self.context = RobotContext()
        self.body = RobotBody(node, self.context)
        self.owner = 'runtime_core'
        self.chassis_pid = ChassisPidTask()
        self.suspension_math = SuspensionMath()
        self._distance_buffers = [collections.deque(maxlen=5) for _ in range(8)]
        self._pe_debounce_counters = [0] * 4
        self._pe_last_states = [0] * 4

    def update_distances(self, values):
        self.context.distances = [float(value) for value in values]
        for index, value in enumerate(self.context.distances[:8]):
            self._distance_buffers[index].append(value)
        self.context.filtered_distances = [
            sum(buffer) / len(buffer) if buffer else float('inf')
            for buffer in self._distance_buffers
        ]
        step_detected, step_height = self.suspension_math.detect_step(self.context)
        self.context.step_detected = step_detected
        self.context.step_height = step_height

    def update_lower_machine(self, values):
        data = [float(value) for value in values]
        self.context.lower_machine_raw = data
        if len(data) >= 4:
            self.context.pe_switches = [int(value) for value in data[:4]]
            self.context.filtered_pe_switches = _debounce_pe(
                self.context.pe_switches,
                self._pe_last_states,
                self._pe_debounce_counters,
            )
        if len(data) >= 8:
            self.context.wheel_heights = [float(value) for value in data[4:8]]
        if (
            self.context.control_direction_by_lower_machine and
            self.suspension_math.state.phase == SuspensionPhase.IDLE and
            len(data) >= 12
        ):
            if data[11] > 0:
                self.update_direction(1)
            elif data[11] < 0:
                self.update_direction(-1)
            else:
                self.update_direction(0)

    def update_direction(self, value):
        self.context.move_direction = int(value)
        self.suspension_math.set_direction(self.context.move_direction)

    def set_direction_control_by_lower_machine(self, enabled):
        self.context.control_direction_by_lower_machine = bool(enabled)

    def update_suspension_state(self, value):
        self.context.suspension_state = int(value)

    def update_suspension_status(self, value):
        self.context.suspension_status = str(value)

    def update_robot_pose(self, pose):
        self.context.robot_pose = pose

    def update_imu(self, imu):
        self.context.imu = imu

    def update_nav_status(self, status):
        self.context.nav_status = str(status)

    def set_emergency_stop(self, enabled):
        self.context.emergency_stop = bool(enabled)
        if self.context.emergency_stop:
            self.stop_all()

    def set_chassis_velocity(self, vx, vy=0.0, wz=0.0):
        return self.body.chassis.set_velocity(vx, vy, wz, self.owner)

    def set_chassis_twist(self, twist):
        return self.body.chassis.set_twist(twist, self.owner)

    def pid_to_relative_pose(self, pose_error, now=None):
        cmd = self.chassis_pid.compute_cmd(pose_error, now)
        self.context.relative_pose_error = _relative_error_list(pose_error)
        self.context.last_pid_cmd = [
            float(cmd.linear.x),
            float(cmd.linear.y),
            float(cmd.angular.z),
        ]
        self.body.chassis.set_twist(cmd, self.owner)
        return cmd

    def reset_chassis_pid(self):
        self.chassis_pid.reset()

    def set_suspension_heights(self, heights):
        return self.body.suspension.set_wheel_heights(heights, self.owner)

    def set_all_suspension_height(self, height):
        return self.body.suspension.set_all_height(height, self.owner)

    def run_suspension_math_once(self):
        result = self.suspension_math.tick(self.context)
        self.context.suspension_state = result['phase'].value
        self.body.suspension.set_wheel_heights(result['wheel_targets'], self.owner)
        return result

    def reset_suspension_math(self):
        self.suspension_math.reset()

    def stop_all(self):
        self.body.chassis.acquire(self.owner)
        self.body.suspension.acquire(self.owner)
        self.body.stop_all(self.owner)

    def release_controllers(self):
        self.body.chassis.release(self.owner)
        self.body.suspension.release(self.owner)


def _debounce_pe(current_values, last_states, counters):
    filtered = list(last_states)
    for index, current_pe in enumerate(current_values[:4]):
        if current_pe != last_states[index]:
            counters[index] += 1
            if counters[index] >= 2:
                filtered[index] = current_pe
                last_states[index] = current_pe
                counters[index] = 0
        else:
            counters[index] = 0
    return filtered


def _relative_error_list(pose_error):
    if hasattr(pose_error, 'linear') and hasattr(pose_error, 'angular'):
        return [
            float(pose_error.linear.x),
            float(pose_error.linear.y),
            float(pose_error.angular.z),
        ]
    values = list(pose_error)
    return [float(values[0]), float(values[1]), float(values[2])]
