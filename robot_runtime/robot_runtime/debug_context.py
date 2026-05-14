def format_debug_context(context):
    lower_machine_raw = list(context.lower_machine_raw)
    pe_raw = _padded(context.lower_machine_raw[:4], 4, 0.0)
    pe_filtered = _padded(context.filtered_pe_switches, 4, 0)
    wheel_heights = _padded(context.wheel_heights, 4, float('nan'))
    suspension_target = _padded(context.suspension_target, 4, float('nan'))
    chassis_target = _padded(context.chassis_velocity_target, 3, 0.0)
    distances = _padded(context.distances, 8, float('nan'))
    filtered_distances = _padded(context.filtered_distances, 8, float('nan'))
    pose = _padded(context.robot_pose_map_xytheta, 3, float('nan'))
    odom = _padded(context.robot_pose_odom_xytheta, 3, float('nan'))
    move_target = _padded(context.move_to_target, 3, float('nan'))
    move_error = _padded(context.move_to_error, 3, float('nan'))
    move_body_error = _padded(context.move_to_body_error, 3, float('nan'))

    return '\n'.join([
        '【机器人运行时调试信息】',
        '状态总览：定位=%s(%s)，导航=%s，急停=%s，最近错误=%s' % (
            context.localization_status,
            '可用' if context.localization_ready else '不可用',
            context.nav_status,
            '触发' if context.emergency_stop else '正常',
            context.last_error or '无',
        ),
        '底盘速度指令：vx=%s, vy=%s, wz=%s' % (
            _fmt(chassis_target[0]),
            _fmt(chassis_target[1]),
            _fmt(chassis_target[2]),
        ),
        '当前运动skill：%s；目标位置[x,y,theta]=[%s, %s, %s]；位置误差dx=%s, dy=%s, dtheta=%s' % (
            context.active_motion_skill,
            _fmt(move_target[0]),
            _fmt(move_target[1]),
            _fmt(move_target[2]),
            _fmt(move_error[0]),
            _fmt(move_error[1]),
            _fmt(move_error[2]),
        ),
        '车体系move_to误差：forward=%s, left=%s, yaw=%s' % (
            _fmt(move_body_error[0]),
            _fmt(move_body_error[1]),
            _fmt(move_body_error[2]),
        ),
        'map定位：frame=%s, x=%s, y=%s, theta=%s' % (
            context.robot_pose_frame or '未知',
            _fmt(pose[0]),
            _fmt(pose[1]),
            _fmt(pose[2]),
        ),
        'odom里程计：frame=%s, x=%s, y=%s, theta=%s' % (
            context.odom_pose_frame or '未知',
            _fmt(odom[0]),
            _fmt(odom[1]),
            _fmt(odom[2]),
        ),
        '悬挂skill：%s；台阶模式=%s；上楼梯方向=%s(%d)' % (
            context.active_suspension_skill,
            '开启' if context.stepmode_enabled else '关闭',
            context.stepmode_direction_name,
            context.stepmode_direction,
        ),
        '四个轮子高度指令：FL=%s, FR=%s, RL=%s, RR=%s' % (
            _fmt(suspension_target[0]),
            _fmt(suspension_target[1]),
            _fmt(suspension_target[2]),
            _fmt(suspension_target[3]),
        ),
        '四轮实时高度(r0x0201第5-8个数据)：FL=%s, FR=%s, RL=%s, RR=%s' % (
            _fmt(wheel_heights[0]),
            _fmt(wheel_heights[1]),
            _fmt(wheel_heights[2]),
            _fmt(wheel_heights[3]),
        ),
        '光电遮挡情况r0x0201前四个数据：raw=[%d, %d, %d, %d]，debounce=[%d, %d, %d, %d]' % (
            int(pe_raw[0]),
            int(pe_raw[1]),
            int(pe_raw[2]),
            int(pe_raw[3]),
            int(pe_filtered[0]),
            int(pe_filtered[1]),
            int(pe_filtered[2]),
            int(pe_filtered[3]),
        ),
        '测距传感器原始值：%s' % _fmt_list(distances),
        '测距传感器滤波值：%s' % _fmt_list(filtered_distances),
        '台阶检测：%s；估计高度=%s；悬挂状态=%s；状态文本=%s' % (
            '检测到' if context.step_detected else '未检测到',
            _fmt(context.step_height),
            context.suspension_state,
            context.suspension_status,
        ),
        'Monitor汇总：/runtime/debug=本消息；/t0x0102_action=%s；/r0x0201=%s；/sensor_distances=%s' % (
            _fmt_list(context.suspension_target),
            _fmt_list(lower_machine_raw),
            _fmt_list(context.distances),
        ),
    ])


def _padded(values, size, fill):
    result = list(values)
    while len(result) < size:
        result.append(fill)
    return result[:size]


def _fmt(value):
    try:
        value = float(value)
    except (TypeError, ValueError):
        return str(value)
    if value != value:
        return '无数据'
    if value == float('inf') or value == -float('inf'):
        return '无穷'
    return '%.3f' % value


def _fmt_list(values):
    if not values:
        return '[]'
    return '[' + ', '.join(_fmt(value) for value in values) + ']'
