from robot_runtime.robot_context import RobotContext
from robot_runtime.debug_context import format_debug_context


def test_debug_context_includes_control_and_sensor_details():
    context = RobotContext()
    context.chassis_velocity_target = [0.1, -0.2, 0.3]
    context.suspension_target = [10.0, 20.0, 30.0, 40.0]
    context.wheel_heights = [11.0, 21.0, 31.0, 41.0]
    context.lower_machine_raw = [1, 0, 1, 0, 11.0, 21.0, 31.0, 41.0, 9.0]
    context.filtered_pe_switches = [1, 0, 1, 0]
    context.distances = [100.0, 101.0, 102.0]
    context.filtered_distances = [99.0, 100.0, 101.0]
    context.stepmode_enabled = True
    context.stepmode_direction = 1
    context.stepmode_direction_name = '左'
    context.move_to_target = [1.0, 2.0, 0.5]
    context.move_to_frame = 'odom'
    context.move_to_error = [0.1, -0.2, 0.05]
    context.move_to_body_error = [0.2, -0.1, 0.05]

    text = format_debug_context(context)

    assert '底盘速度指令' in text
    assert '四个轮子高度指令' in text
    assert '上楼梯方向=左(1)' in text
    assert '目标frame=odom' in text
    assert '位置误差dx=0.100' in text
    assert '车体系move_to误差：forward=0.200, left=-0.100, yaw=0.050' in text
    assert '光电遮挡情况r0x0201前四个数据' in text
    assert 'r0x0201第5-8个数据' in text
    assert 'Monitor汇总' in text
    assert '/t0x0102_action=[10.000, 20.000, 30.000, 40.000]' in text
    assert '/r0x0201=[1.000, 0.000, 1.000, 0.000, 11.000, 21.000, 31.000, 41.000, 9.000]' in text
    assert '/sensor_distances=[100.000, 101.000, 102.000]' in text
