from typing import List

MOVE_GROUP_ARM: str = "left_arm"
MOVE_GROUP_GRIPPER: str = "left_hand"

OPEN_GRIPPER_JOINT_POSITIONS: List[float] = [0.04, 0.04]
CLOSED_GRIPPER_JOINT_POSITIONS: List[float] = [0.0, 0.0]

prefix: str = "L_panda_"

def joint_names(prefix: str = prefix) -> List[str]:
    return [
        prefix + "joint1",
        prefix + "joint2",
        prefix + "joint3",
        prefix + "joint4",
        prefix + "joint5",
        prefix + "joint6",
        prefix + "joint7",
    ]


def base_link_name(prefix: str = prefix) -> str:
    return prefix + "link0"


def end_effector_name(prefix: str = prefix) -> str:
    return prefix + "hand"


def gripper_joint_names(prefix: str = prefix) -> List[str]:
    return [
        prefix + "finger_joint1",
        prefix + "finger_joint2",
    ]
