from dataclasses import dataclass, field
from typing import List, Optional

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu


@dataclass
class RobotContext:
    """Shared bottom-layer state cache updated by runtime callbacks."""

    robot_pose: Optional[PoseStamped] = None
    nav_status: str = 'idle'
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

    relative_pose_error: List[float] = field(default_factory=list)
    last_pid_cmd: List[float] = field(default_factory=list)

    emergency_stop: bool = False
    last_error: str = ''
