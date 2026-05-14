import collections
from dataclasses import dataclass, field
from enum import Enum
import math


class SuspensionPhase(Enum):
    IDLE = 0
    UP_1_PREPARE = 10
    UP_2_LIFT = 11
    UP_3_FRONT_DOCK = 12
    UP_4_RETRACT_FRONT = 13
    UP_5_FRONT_LAND = 14
    UP_6_SIDE_DOCK_RETRACT_REAR = 15
    UP_7_REAR_LAND = 16
    UP_8_RECOVER = 17
    DOWN_1_PREPARE = 20
    DOWN_2_FRONT_HOVER_LAND = 21
    DOWN_3_REAR_HOVER_LAND = 22
    DOWN_4_RECOVERY = 23


class MoveDirection(Enum):
    FORWARD = 0
    LEFT = 1
    RIGHT = 2


@dataclass
class SuspensionConfig:
    lift_low: float = 205.0
    lift_high: float = 400.0
    init_height: float = 30.0
    height_tolerance: float = 20.0
    step_detect_threshold: float = 200.0
    idle_trigger_stable_ticks: int = 2


@dataclass
class SuspensionState:
    phase: SuspensionPhase = SuspensionPhase.IDLE
    direction: MoveDirection = MoveDirection.FORWARD
    direction_latched: bool = False
    target_height: float = 0.0
    height_latched: bool = False
    stable_counters: dict = field(
        default_factory=lambda: collections.defaultdict(int)
    )


class SuspensionMath:
    """Active suspension state machine that reads detailed RobotContext data."""

    def __init__(self, config=None):
        self.config = config or SuspensionConfig()
        self.state = SuspensionState()

    def set_direction(self, direction_value):
        if self.state.direction_latched:
            return
        mapping = {
            0: MoveDirection.FORWARD,
            1: MoveDirection.LEFT,
            -1: MoveDirection.RIGHT,
        }
        self.state.direction = mapping.get(direction_value, MoveDirection.FORWARD)

    def reset(self):
        self.state = SuspensionState()

    def detect_step(self, context):
        distances = _distances(context)
        valid = [value for value in distances if _is_valid_number(value)]
        if not valid:
            return False, 0.0
        detected = any(value < self.config.step_detect_threshold for value in valid)
        return detected, max(valid)

    def tick(self, context):
        self.set_direction(context.move_direction)
        wheel_targets = _current_targets(context, self.config.init_height)
        phase = self.state.phase
        previous_phase = phase

        v_wheels_idx, v_pe_idx, v_distances_idx = _virtual_mapping(self.state.direction)
        v0, v1, v2, v3 = 0, 1, 2, 3

        if phase == SuspensionPhase.IDLE:
            up_distance = _get_v_distance(context, v_distances_idx, v1)
            down_distance = _get_v_distance(context, v_distances_idx, v0)
            cond_up = _is_valid_number(up_distance) and up_distance < 200.0
            cond_down = _is_valid_number(down_distance) and down_distance > 200.0
            if _is_stable(
                self.state,
                cond_up,
                'idle_to_up',
                threshold=self.config.idle_trigger_stable_ticks,
            ):
                self.state.phase = SuspensionPhase.UP_1_PREPARE
            elif _is_stable(
                self.state,
                cond_down,
                'idle_to_down',
                threshold=self.config.idle_trigger_stable_ticks,
            ):
                self.state.phase = SuspensionPhase.DOWN_1_PREPARE

        elif phase == SuspensionPhase.UP_1_PREPARE:
            self.state.target_height = self.config.lift_low
            wheel_targets = [self.state.target_height] * 4
            if _is_stable(
                self.state,
                _check_height_reached(
                    context,
                    v_wheels_idx,
                    [v0, v1, v2, v3],
                    self.state.target_height,
                    self.config.height_tolerance,
                ),
                'up1_height',
                threshold=2,
            ):
                self.state.phase = SuspensionPhase.UP_2_LIFT

        elif phase == SuspensionPhase.UP_2_LIFT:
            cond_high_dist = _get_v_distance(context, v_distances_idx, v1) < 200.0
            if _is_stable(self.state, cond_high_dist, 'up2_high_dist', threshold=18):
                self.state.target_height = self.config.lift_high
                wheel_targets = [self.state.target_height] * 4
                if _is_stable(
                    self.state,
                    _check_height_reached(
                        context,
                        v_wheels_idx,
                        [v0, v1, v2, v3],
                        self.state.target_height,
                        self.config.height_tolerance,
                    ),
                    'up2_height',
                    threshold=2,
                ):
                    self.state.phase = SuspensionPhase.UP_3_FRONT_DOCK
            elif _is_stable(self.state, not cond_high_dist, 'up2_low_dist'):
                self.state.phase = SuspensionPhase.UP_3_FRONT_DOCK

        elif phase == SuspensionPhase.UP_3_FRONT_DOCK:
            if _is_stable(
                self.state,
                _get_v_distance(context, v_distances_idx, v0) < 80.0,
                'up3_dist',
            ):
                self.state.phase = SuspensionPhase.UP_4_RETRACT_FRONT

        elif phase == SuspensionPhase.UP_4_RETRACT_FRONT:
            wheel_targets = _set_v_wheel_height(wheel_targets, v_wheels_idx, [v0, v1], 5.0)
            if _is_stable(
                self.state,
                _check_height_reached(
                    context,
                    v_wheels_idx,
                    [v0, v1],
                    5.0,
                    self.config.height_tolerance,
                ),
                'up4_height',
                threshold=2,
            ):
                self.state.phase = SuspensionPhase.UP_5_FRONT_LAND

        elif phase == SuspensionPhase.UP_5_FRONT_LAND:
            cond_pe = _get_v_pe(context, v_pe_idx, v0) == 1
            if _is_stable(self.state, cond_pe, 'up5_pe'):
                wheel_targets = _set_v_wheel_height(wheel_targets, v_wheels_idx, [v0, v1], 7.0)
                wheel_targets = _set_v_wheel_height(
                    wheel_targets,
                    v_wheels_idx,
                    [v2, v3],
                    self.state.target_height + 3.0,
                )
                if _is_stable(
                    self.state,
                    _check_height_reached(
                        context,
                        v_wheels_idx,
                        [v2, v3],
                        self.state.target_height + 3.0,
                        self.config.height_tolerance,
                    ),
                    'up5_height',
                    threshold=2,
                ):
                    self.state.phase = SuspensionPhase.UP_6_SIDE_DOCK_RETRACT_REAR

        elif phase == SuspensionPhase.UP_6_SIDE_DOCK_RETRACT_REAR:
            cond_pe = _get_v_pe(context, v_pe_idx, v2) == 1
            if _is_stable(self.state, cond_pe, 'up6_pe', threshold=20):
                wheel_targets = _set_v_wheel_height(wheel_targets, v_wheels_idx, [v2, v3], 0.0)
                if _is_stable(
                    self.state,
                    _check_height_reached(
                        context,
                        v_wheels_idx,
                        [v2, v3],
                        0.0,
                        self.config.height_tolerance,
                    ),
                    'up6_height',
                    threshold=2,
                ):
                    self.state.phase = SuspensionPhase.UP_7_REAR_LAND

        elif phase == SuspensionPhase.UP_7_REAR_LAND:
            cond_pe = (
                _get_v_pe(context, v_pe_idx, v2) == 1 and
                _get_v_pe(context, v_pe_idx, v3) == 1
            )
            if _is_stable(self.state, cond_pe, 'up7_pe'):
                wheel_targets = _set_v_wheel_height(wheel_targets, v_wheels_idx, [v2, v3], 3.0)
                if _is_stable(
                    self.state,
                    _check_height_reached(
                        context,
                        v_wheels_idx,
                        [v2, v3],
                        3.0,
                        self.config.height_tolerance,
                    ),
                    'up7_height',
                    threshold=2,
                ):
                    self.state.phase = SuspensionPhase.UP_8_RECOVER

        elif phase == SuspensionPhase.UP_8_RECOVER:
            wheel_targets = [self.config.init_height] * 4
            if _is_stable(
                self.state,
                _check_height_reached(
                    context,
                    v_wheels_idx,
                    [v0, v1, v2, v3],
                    self.config.init_height,
                    self.config.height_tolerance,
                ),
                'up8_height',
                threshold=2,
            ):
                self.state.phase = SuspensionPhase.IDLE

        elif phase == SuspensionPhase.DOWN_1_PREPARE:
            cond_pe = _get_v_pe(context, v_pe_idx, v0) == 0
            if _is_stable(self.state, cond_pe, 'down1_pe'):
                if not self.state.height_latched:
                    dist = _get_v_distance(context, v_distances_idx, v0)
                    if dist > 380.0:
                        self.state.target_height = self.config.lift_high
                    elif dist > 180.0:
                        self.state.target_height = self.config.lift_low
                    else:
                        self.state.target_height = self.config.lift_low
                    self.state.height_latched = True

                wheel_targets = _set_v_wheel_height(
                    wheel_targets,
                    v_wheels_idx,
                    [v0, v1],
                    self.state.target_height + 30.0,
                )
                if _is_stable(
                    self.state,
                    _check_height_reached(
                        context,
                        v_wheels_idx,
                        [v0, v1],
                        self.state.target_height + 10.0,
                        self.config.height_tolerance,
                    ),
                    'down1_height',
                    threshold=2,
                ):
                    self.state.height_latched = False
                    self.state.phase = SuspensionPhase.DOWN_2_FRONT_HOVER_LAND

        elif phase == SuspensionPhase.DOWN_2_FRONT_HOVER_LAND:
            cond_pe = _get_v_pe(context, v_pe_idx, v3) == 0
            if _is_stable(self.state, cond_pe, 'down2_pe'):
                wheel_targets = _set_v_wheel_height(
                    wheel_targets,
                    v_wheels_idx,
                    [v2, v3],
                    self.state.target_height + 10.0,
                )
                if _is_stable(
                    self.state,
                    _check_height_reached(
                        context,
                        v_wheels_idx,
                        [v2, v3],
                        self.state.target_height,
                        self.config.height_tolerance,
                    ),
                    'down2_height',
                    threshold=2,
                ):
                    self.state.phase = SuspensionPhase.DOWN_3_REAR_HOVER_LAND

        elif phase == SuspensionPhase.DOWN_3_REAR_HOVER_LAND:
            cond_dist = _get_v_distance(context, v_distances_idx, v3) > 200.0
            if _is_stable(self.state, cond_dist, 'down3_dist'):
                wheel_targets = [self.config.init_height] * 4
                self.state.phase = SuspensionPhase.DOWN_4_RECOVERY

        elif phase == SuspensionPhase.DOWN_4_RECOVERY:
            if _is_stable(
                self.state,
                _check_height_reached(
                    context,
                    v_wheels_idx,
                    [v0, v1, v2, v3],
                    self.config.init_height,
                    self.config.height_tolerance,
                ),
                'down4_height',
                threshold=2,
            ):
                self.state.phase = SuspensionPhase.IDLE

        if self.state.phase != previous_phase:
            if previous_phase == SuspensionPhase.IDLE:
                self.state.direction_latched = True
            elif self.state.phase == SuspensionPhase.IDLE:
                self.state.direction_latched = False
            self.state.stable_counters.clear()

        return {
            'phase': self.state.phase,
            'wheel_targets': [float(value) for value in wheel_targets],
            'target_height': float(self.state.target_height),
        }


def _distances(context):
    values = context.filtered_distances or context.distances
    return [float(value) for value in values]


def _virtual_mapping(direction):
    if direction == MoveDirection.FORWARD:
        return [2, 1, 0, 3], [0, 1, 3, 2], [0, 1, 5, 4]
    if direction == MoveDirection.LEFT:
        return [0, 2, 3, 1], [1, 2, 0, 3], [2, 3, 7, 6]
    if direction == MoveDirection.RIGHT:
        return [1, 3, 2, 0], [3, 0, 2, 1], [6, 7, 3, 2]
    return [2, 1, 0, 3], [0, 1, 3, 2], [0, 1, 5, 4]


def _is_stable(state, condition, key, threshold=5):
    if condition:
        if state.stable_counters[key] < threshold:
            state.stable_counters[key] += 1
        if state.stable_counters[key] >= threshold:
            return True
    else:
        state.stable_counters[key] = 0
    return False


def _current_targets(context, default_height):
    values = [float(value) for value in context.suspension_target[:4]]
    while len(values) < 4:
        values.append(float(default_height))
    return values[:4]


def _get_v_pe(context, v_pe_idx, v_idx):
    pe_values = list(context.filtered_pe_switches or context.pe_switches)
    while len(pe_values) < 4:
        pe_values.append(0)
    return int(pe_values[v_pe_idx[v_idx]])


def _get_v_distance(context, v_distances_idx, v_idx):
    distances = _distances(context)
    while len(distances) <= v_distances_idx[v_idx]:
        distances.append(float('nan'))
    return float(distances[v_distances_idx[v_idx]])


def _distance_ready(context, v_distances_idx):
    distances = _distances(context)
    if len(distances) <= max(v_distances_idx):
        return False
    return all(_is_valid_number(distances[index]) for index in v_distances_idx)


def _check_height_reached(context, v_wheels_idx, virtual_indices, target_h, tolerance):
    heights = list(context.wheel_heights)
    while len(heights) < 4:
        heights.append(0.0)
    for v_idx in virtual_indices:
        phys_idx = v_wheels_idx[v_idx]
        if abs(float(heights[phys_idx]) - float(target_h)) > float(tolerance):
            return False
    return True


def _set_v_wheel_height(targets, v_wheels_idx, v_indices, height):
    outputs = list(targets[:4])
    while len(outputs) < 4:
        outputs.append(0.0)
    for v_idx in v_indices:
        outputs[v_wheels_idx[v_idx]] = float(height)
    return outputs


def _is_valid_number(value):
    return math.isfinite(float(value))
