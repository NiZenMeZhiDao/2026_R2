from dataclasses import dataclass, field
from typing import Any, List, Optional

try:
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import Imu
except ImportError:
    PoseStamped = Any
    Imu = Any


@dataclass
class RobotContext:
    """Shared bottom-layer state cache updated by runtime callbacks."""

    robot_pose: Optional[PoseStamped] = None
    robot_pose_map: Optional[PoseStamped] = None
    robot_pose_odom: Optional[PoseStamped] = None
    robot_pose_map_xytheta: List[float] = field(default_factory=list)
    robot_pose_odom_xytheta: List[float] = field(default_factory=list)
    robot_x: float = 0.0
    robot_y: float = 0.0
    robot_theta: float = 0.0
    odom_x: float = 0.0
    odom_y: float = 0.0
    odom_theta: float = 0.0
    robot_pose_frame: str = ''
    odom_pose_frame: str = ''
    map_ready: bool = False
    localization_ready: bool = False
    odom_ready: bool = False
    robot_pose_receive_time: float = 0.0
    odom_pose_receive_time: float = 0.0
    nav_status: str = 'idle'
    localization_status: str = 'idle'
    imu: Optional[Imu] = None

    distances: List[float] = field(default_factory=list)
    filtered_distances: List[float] = field(default_factory=list)
    pe_switches: List[int] = field(default_factory=list)
    filtered_pe_switches: List[int] = field(default_factory=list)
    wheel_heights: List[float] = field(default_factory=list)
    lower_machine_raw: List[float] = field(default_factory=list)
    move_direction: int = 0
    control_direction_by_lower_machine: bool = False

    step_detected: bool = False
    step_height: float = 0.0
    suspension_state: int = 0
    suspension_status: str = 'idle'
    suspension_target: List[float] = field(default_factory=list)
    chassis_velocity_target: List[float] = field(default_factory=list)
    active_motion_skill: str = 'idle'
    active_suspension_skill: str = 'idle'
    stepmode_enabled: bool = False
    stepmode_direction: int = 0
    stepmode_direction_name: str = '前进'
    move_to_target: List[float] = field(default_factory=list)
    move_to_frame: str = 'map'
    move_to_error: List[float] = field(default_factory=list)
    move_to_body_error: List[float] = field(default_factory=list)

    relative_pose_error: List[float] = field(default_factory=list)
    last_pid_cmd: List[float] = field(default_factory=list)

    emergency_stop: bool = False
    last_error: str = ''
