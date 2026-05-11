from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple


SHAPE_SPHERE = 0
SHAPE_BOX = 1
SHAPE_CAPSULE = 2

MODE_STATIC = 0
MODE_DYNAMIC = 1
MODE_DYNAMIC_BONE = 2


@dataclass
class RigidBodyData:
    index: int
    name: str
    obj: Any
    bone_name: str
    shape: int
    mode: int
    size: Tuple[float, float, float]
    mass: float
    friction: float
    restitution: float
    linear_damping: float
    angular_damping: float
    collision_group_number: int
    no_collision_mask: int
    local_matrix: Any
    bone_offset_matrix: Optional[Any] = None


@dataclass
class JointData:
    index: int
    name: str
    obj: Any
    rigid_a_index: int
    rigid_b_index: int
    local_matrix: Any
    linear_lower: Tuple[float, float, float]
    linear_upper: Tuple[float, float, float]
    angular_lower: Tuple[float, float, float]
    angular_upper: Tuple[float, float, float]
    spring_linear: Tuple[float, float, float]
    spring_linear_damping: Tuple[float, float, float]
    spring_angular: Tuple[float, float, float]
    spring_angular_damping: Tuple[float, float, float]
    disable_collisions: bool
    joint_stop_erp: float = -1.0
    joint_stop_cfm: float = -1.0
    locked_joint_stop_erp: float = -1.0
    locked_joint_stop_cfm: float = -1.0
    zone_rule: str = ""


@dataclass
class PmxModelData:
    root: Any
    armature: Optional[Any]
    rigid_bodies: List[RigidBodyData]
    joints: List[JointData]
    object_to_rigid_index: Dict[Any, int]
    non_collision_pairs: List[Tuple[int, int]]
