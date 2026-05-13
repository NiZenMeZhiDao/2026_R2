from types import SimpleNamespace

from robot_runtime.libraries.suspension_math import (
    MoveDirection,
    SuspensionMath,
    SuspensionPhase,
)


def _context(**overrides):
    values = {
        'move_direction': 0,
        'filtered_distances': [300.0] * 8,
        'distances': [],
        'filtered_pe_switches': [0] * 4,
        'pe_switches': [0] * 4,
        'wheel_heights': [30.0] * 4,
        'suspension_target': [30.0] * 4,
    }
    values.update(overrides)
    return SimpleNamespace(**values)


def test_idle_ignores_incomplete_distance_frame():
    math_lib = SuspensionMath()
    context = _context(filtered_distances=[float('inf')] * 8)

    for _ in range(6):
        result = math_lib.tick(context)

    assert result['phase'] == SuspensionPhase.IDLE


def test_idle_can_start_up_with_only_forward_trigger_distance_valid():
    math_lib = SuspensionMath()
    context = _context(
        filtered_distances=[
            float('nan'),
            100.0,
            float('nan'),
            float('nan'),
            float('nan'),
            float('nan'),
            float('nan'),
            float('nan'),
        ],
    )

    for _ in range(5):
        result = math_lib.tick(context)

    assert result['phase'] == SuspensionPhase.UP_1_PREPARE


def test_direction_is_latched_while_sequence_runs():
    math_lib = SuspensionMath()
    math_lib.state.phase = SuspensionPhase.UP_4_RETRACT_FRONT
    math_lib.state.direction = MoveDirection.RIGHT
    math_lib.state.direction_latched = True

    result = math_lib.tick(_context(move_direction=-1))

    assert math_lib.state.direction == MoveDirection.RIGHT
    assert result['wheel_targets'] == [30.0, 5.0, 30.0, 5.0]


def test_unknown_direction_falls_back_to_forward():
    math_lib = SuspensionMath()

    math_lib.set_direction(3)

    assert math_lib.state.direction == MoveDirection.FORWARD


def test_direction_unlatches_after_recovery_returns_to_idle():
    math_lib = SuspensionMath()
    math_lib.state.phase = SuspensionPhase.UP_8_RECOVER
    math_lib.state.direction = MoveDirection.LEFT
    math_lib.state.direction_latched = True
    context = _context(move_direction=-1, wheel_heights=[30.0] * 4)

    math_lib.tick(context)
    result = math_lib.tick(context)

    assert result['phase'] == SuspensionPhase.IDLE
    assert math_lib.state.direction_latched is False


def test_detect_step_ignores_non_finite_distances():
    math_lib = SuspensionMath()
    context = _context(filtered_distances=[float('inf'), float('nan'), 250.0])

    detected, height = math_lib.detect_step(context)

    assert detected is False
    assert height == 250.0
