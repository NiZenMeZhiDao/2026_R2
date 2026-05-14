import collections
import math
import time as time_module

from robot_runtime.libraries.suspension_math import SuspensionMath, SuspensionPhase
from robot_runtime.pid_config import chassis_deadzone
from robot_runtime.robot_body import RobotBody
from robot_runtime.robot_context import RobotContext


class RuntimeCore:
    """Robot skill API used by the middle layer."""

    def __init__(self, node):
        self._node = node
        self.context = RobotContext()
        self.body = RobotBody(node, self.context)
        self.owner = 'runtime_core'
        self.suspension_math = SuspensionMath()
        self._chassis_deadzone = chassis_deadzone()
        self._distance_buffers = [collections.deque(maxlen=5) for _ in range(8)]
        self._pe_debounce_counters = [0] * 4
        self._pe_last_states = [0] * 4
        self._skill_period = 0.005
        self._motion_skill = None
        self._suspension_skill = None
        self._last_motion_publish_time = None
        self._last_suspension_result = None
        self._skill_timer = None
        if hasattr(node, 'create_timer'):
            self._skill_timer = node.create_timer(self._skill_period, self.tick_skills)

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
                self._set_internal_direction(1)
            elif data[11] < 0:
                self._set_internal_direction(-1)
            else:
                self._set_internal_direction(0)

    def set_direction_control_by_lower_machine(self, enabled):
        self.context.control_direction_by_lower_machine = bool(enabled)

    def update_suspension_state(self, value):
        self.context.suspension_state = int(value)

    def update_suspension_status(self, value):
        self.context.suspension_status = str(value)

    def update_robot_pose(self, pose):
        self.context.robot_pose = pose
        self.context.robot_pose_map = pose
        x, y, theta = _pose_to_xytheta(pose)
        self.context.robot_pose_map_xytheta = [x, y, theta]
        self.context.robot_x = x
        self.context.robot_y = y
        self.context.robot_theta = theta
        self.context.robot_pose_frame = str(pose.header.frame_id)
        self.context.localization_ready = True

    def update_robot_pose_odom(self, pose):
        self.context.robot_pose_odom = pose
        x, y, theta = _pose_to_xytheta(pose)
        self.context.robot_pose_odom_xytheta = [x, y, theta]
        self.context.odom_x = x
        self.context.odom_y = y
        self.context.odom_theta = theta
        self.context.odom_pose_frame = str(pose.header.frame_id)
        self.context.odom_ready = True

    def update_imu(self, imu):
        self.context.imu = imu

    def update_nav_status(self, status):
        self.context.nav_status = str(status)

    def update_localization_status(self, status):
        self.context.localization_status = str(status)
        self.context.localization_ready = self.context.localization_status in (
            'localized',
            'localized_unaligned',
        )

    def set_emergency_stop(self, enabled):
        self.context.emergency_stop = bool(enabled)
        if self.context.emergency_stop:
            self._motion_skill = None
            self._suspension_skill = None
            self._stop_all()

    # ===== Student-facing skills =====
    def move(self, vx, vy=0.0, wz=0.0, time=0.0, duration=None, time_sec=None):
        """Move asynchronously; duration 0 means hold until a new motion command."""
        if duration is None:
            duration = time
        if time_sec is not None:
            duration = time_sec
        now = time_module.monotonic()
        duration = max(0.0, float(duration))
        self._motion_skill = {
            'kind': 'move',
            'vx': float(vx),
            'vy': float(vy),
            'wz': float(wz),
            'deadline': None if duration == 0.0 else now + duration,
        }
        self.context.active_motion_skill = 'move'
        self.context.move_to_target = []
        self.context.move_to_error = []
        self.context.move_to_body_error = []
        self._last_motion_publish_time = None
        return self._motion_skill.copy()

    def move_to(
        self,
        x,
        y,
        theta,
        vx=0.3,
        vy=0.2,
        wz=0.5,
        timeout=10.0,
        wait=True,
    ):
        """Move toward a map pose; by default wait until arrival or timeout."""
        self._motion_skill = {
            'kind': 'move_to',
            'x': float(x),
            'y': float(y),
            'theta': float(theta),
            'vx_limit': abs(float(vx)),
            'vy_limit': abs(float(vy)),
            'wz_limit': abs(float(wz)),
            'xy_tolerance': 0.05,
            'theta_tolerance': 0.05,
        }
        self.context.active_motion_skill = 'move_to'
        self.context.move_to_target = [float(x), float(y), float(theta)]
        self.context.move_to_error = []
        self.context.move_to_body_error = []
        self._last_motion_publish_time = None
        if not wait:
            return self._motion_skill.copy()
        return self._wait_motion_skill(timeout)

    def stop(self):
        """Stop all active skills and publish a stop command."""
        self._motion_skill = None
        self._suspension_skill = None
        self.context.active_motion_skill = 'idle'
        self.context.active_suspension_skill = 'idle'
        self.context.stepmode_enabled = False
        self._stop_all()

    def set_height(self, height):
        """Hold all four suspension wheels at the requested height asynchronously."""
        self._suspension_skill = {
            'kind': 'height',
            'height': float(height),
        }
        self.context.active_suspension_skill = 'height'
        self.context.stepmode_enabled = False
        self.context.stepmode_direction = 0
        self.context.stepmode_direction_name = _stepmode_direction_name(0)
        return self._suspension_skill.copy()

    def set_stepmode(self, enabled, direction=0):
        """Enable or disable the asynchronous stair suspension state machine."""
        if not enabled:
            self._suspension_skill = None
            self._reset_suspension_math()
            self.context.active_suspension_skill = 'idle'
            self.context.stepmode_enabled = False
            return {'kind': 'stepmode', 'enabled': False}

        internal_direction = _stepmode_direction_to_internal(direction)
        self._set_internal_direction(internal_direction)
        self._reset_suspension_math()
        self._suspension_skill = {
            'kind': 'stepmode',
            'enabled': True,
            'direction': int(direction),
            'internal_direction': internal_direction,
        }
        self.context.active_suspension_skill = 'stepmode'
        self.context.stepmode_enabled = True
        self.context.stepmode_direction = int(direction)
        self.context.stepmode_direction_name = _stepmode_direction_name(direction)
        return self._suspension_skill.copy()

    def tick_skills(self):
        """Run one cycle of the active asynchronous skills."""
        if self.context.emergency_stop:
            self._motion_skill = None
            self._suspension_skill = None
            self.context.active_motion_skill = 'idle'
            self.context.active_suspension_skill = 'idle'
            self.context.stepmode_enabled = False
            self._stop_all()
            return

        try:
            self._tick_motion_skill()
            self._tick_suspension_skill()
        except Exception as exc:
            self.context.last_error = str(exc)

    @property
    def active_motion_skill(self):
        return None if self._motion_skill is None else self._motion_skill.copy()

    @property
    def active_suspension_skill(self):
        return None if self._suspension_skill is None else self._suspension_skill.copy()

    @property
    def last_suspension_result(self):
        return self._last_suspension_result

    def _publish_chassis_velocity(self, vx, vy=0.0, wz=0.0):
        compensated = _apply_chassis_deadzone(
            vx,
            vy,
            wz,
            xy_deadzone=self._chassis_deadzone['xy'],
            wz_deadzone=self._chassis_deadzone['wz'],
        )
        self.context.chassis_velocity_target = list(compensated)
        return self.body.chassis.set_velocity(*compensated, owner=self.owner)

    def _publish_suspension_heights(self, heights):
        self.context.suspension_target = [float(value) for value in heights[:4]]
        return self.body.suspension.set_wheel_heights(heights, self.owner)

    def _publish_all_suspension_height(self, height):
        self.context.suspension_target = [float(height)] * 4
        return self.body.suspension.set_all_height(height, self.owner)

    def _run_suspension_math_once(self):
        result = self.suspension_math.tick(self.context)
        self.context.suspension_state = result['phase'].value
        self._publish_suspension_heights(result['wheel_targets'])
        return result

    def _reset_suspension_math(self):
        self.suspension_math.reset()

    def _stop_all(self):
        self.body.chassis.acquire(self.owner)
        self.body.suspension.acquire(self.owner)
        self.context.chassis_velocity_target = [0.0, 0.0, 0.0]
        self.body.stop_all(self.owner)

    def _set_internal_direction(self, value):
        self.context.move_direction = int(value)
        self.suspension_math.set_direction(self.context.move_direction)

    def _tick_motion_skill(self):
        if self._motion_skill is None:
            return

        kind = self._motion_skill['kind']
        if kind == 'move':
            now = time_module.monotonic()
            deadline = self._motion_skill['deadline']
            if deadline is not None and now >= deadline:
                self._publish_chassis_velocity(0.0, 0.0, 0.0)
                self._motion_skill = None
                self.context.active_motion_skill = 'idle'
                return
            self._publish_chassis_velocity(
                self._motion_skill['vx'],
                self._motion_skill['vy'],
                self._motion_skill['wz'],
            )
            self._last_motion_publish_time = now
            return

        if kind == 'move_to':
            cmd = self._compute_move_to_velocity(self._motion_skill)
            if cmd is None:
                return
            vx, vy, wz, done = cmd
            self._publish_chassis_velocity(vx, vy, wz)
            if done:
                self._motion_skill = None
                self.context.active_motion_skill = 'idle'

    def _tick_suspension_skill(self):
        if self._suspension_skill is None:
            return

        kind = self._suspension_skill['kind']
        if kind == 'height':
            self._publish_all_suspension_height(self._suspension_skill['height'])
            return

        if kind == 'stepmode':
            self._set_internal_direction(self._suspension_skill['internal_direction'])
            self._last_suspension_result = self._run_suspension_math_once()

    def _wait_motion_skill(self, timeout):
        deadline = time_module.monotonic() + float(timeout)
        while self._motion_skill is not None:
            _spin_node_once(self._node)
            self.tick_skills()
            if self._motion_skill is None:
                break
            if time_module.monotonic() >= deadline:
                target = list(self.context.move_to_target)
                error = list(self.context.move_to_error)
                self._motion_skill = None
                self.context.active_motion_skill = 'idle'
                self._publish_chassis_velocity(0.0, 0.0, 0.0)
                raise TimeoutError(
                    'move_to timed out after %.1fs; target=%s error=%s last_error=%s' % (
                        float(timeout),
                        target,
                        error,
                        self.context.last_error or 'none',
                    )
                )
            time_module.sleep(self._skill_period)
        return {
            'kind': 'move_to',
            'target': list(self.context.move_to_target),
            'error': list(self.context.move_to_error),
            'reached': True,
        }

    def _compute_move_to_velocity(self, skill):
        pose = self.context.robot_pose_map or self.context.robot_pose
        ready = self.context.localization_ready or self.context.odom_ready
        if not ready:
            self._publish_chassis_velocity(0.0, 0.0, 0.0)
            self.context.last_error = 'move_to requires localization or odometry'
            return None
        if pose is None:
            pose = self.context.robot_pose_odom
        if pose is None:
            self._publish_chassis_velocity(0.0, 0.0, 0.0)
            self.context.last_error = 'move_to requires robot_pose_map, robot_pose, or robot_pose_odom'
            return None

        current_x = float(pose.pose.position.x)
        current_y = float(pose.pose.position.y)
        current_yaw = _yaw_from_orientation(pose.pose.orientation)
        dx = skill['x'] - current_x
        dy = skill['y'] - current_y
        yaw_error = _wrap_angle(skill['theta'] - current_yaw)
        self.context.move_to_error = [float(dx), float(dy), float(yaw_error)]
        self.context.relative_pose_error = list(self.context.move_to_error)

        cos_yaw = math.cos(current_yaw)
        sin_yaw = math.sin(current_yaw)
        forward_error = cos_yaw * dx + sin_yaw * dy
        left_error = -sin_yaw * dx + cos_yaw * dy
        self.context.move_to_body_error = [
            float(forward_error),
            float(left_error),
            float(yaw_error),
        ]

        done_xy = math.hypot(dx, dy) <= skill['xy_tolerance']
        done_yaw = abs(yaw_error) <= skill['theta_tolerance']
        if done_xy and done_yaw:
            return 0.0, 0.0, 0.0, True

        vx = _clamp(0.8 * forward_error, -skill['vx_limit'], skill['vx_limit'])
        vy = _clamp(0.8 * left_error, -skill['vy_limit'], skill['vy_limit'])
        wz = _clamp(1.2 * yaw_error, -skill['wz_limit'], skill['wz_limit'])
        return vx, vy, wz, False


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


def _yaw_from_orientation(orientation):
    x = float(orientation.x)
    y = float(orientation.y)
    z = float(orientation.z)
    w = float(orientation.w)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _wrap_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def _clamp(value, lower, upper):
    if upper <= 0.0:
        return 0.0
    return max(lower, min(upper, value))


def _apply_chassis_deadzone(vx, vy, wz, xy_deadzone=0.0, wz_deadzone=0.0):
    return (
        _add_signed_deadzone(vx, xy_deadzone),
        _add_signed_deadzone(vy, xy_deadzone),
        _add_signed_deadzone(wz, wz_deadzone),
    )


def _add_signed_deadzone(value, deadzone):
    value = float(value)
    deadzone = abs(float(deadzone))
    if abs(value) <= 1e-9:
        return 0.0
    if value > 0.0:
        return value + deadzone
    if value < 0.0:
        return value - deadzone
    return value


def _stepmode_direction_to_internal(direction):
    direction = int(direction)
    if direction > 0:
        return -1
    if direction < 0:
        return 1
    return 0


def _stepmode_direction_name(direction):
    direction = int(direction)
    if direction > 0:
        return '左'
    if direction < 0:
        return '右'
    return '前进'


def _pose_to_xytheta(pose):
    return (
        float(pose.pose.position.x),
        float(pose.pose.position.y),
        _yaw_from_orientation(pose.pose.orientation),
    )


def _spin_node_once(node):
    try:
        import rclpy
    except ImportError:
        return
    if hasattr(rclpy, 'ok') and not rclpy.ok():
        return
    try:
        for _ in range(6):
            rclpy.spin_once(node, timeout_sec=0.0)
    except Exception:
        return
