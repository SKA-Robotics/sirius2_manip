import roboticstoolbox as rtb
from pathlib import Path
from typing import Optional
import numpy as np
import math

def load_urdf(urdf_path: str, gripper: Optional[str] = None) -> rtb.Robot:
    """
    Loads a URDF file and returns a roboticstoolbox robot object.
    """
    xacro_file_path = Path(urdf_path).resolve()
    project_root = xacro_file_path.parent
    xacro_file_path = str(xacro_file_path)
    tld_path = str(project_root)
    links, name, urdf_string, urdf_filepath_original = rtb.Robot.URDF_read(xacro_file_path, tld=tld_path)
    if gripper is None:
        robot = rtb.Robot(links, name=name, urdf_string=urdf_string, urdf_filepath=urdf_filepath_original)
    else:
        gripper_idx = -1
        for i, link in enumerate(links):
            if link.name == gripper:
                gripper_idx = i
                break
        if gripper_idx == -1:
            raise ValueError(f"Gripper link '{gripper}' not found in URDF file {xacro_file_path}")
        robot = rtb.Robot(links, name=name, urdf_string=urdf_string, urdf_filepath=urdf_filepath_original, gripper_links = links[gripper_idx])
    return robot


def angle_axis_error(T: np.ndarray, Td: np.ndarray) -> np.ndarray:
    """
    Returns the error vector between T and Td in angle-axis form.

    :param T: The current pose
    :param Tep: The desired pose

    :returns e: the error vector between T and Td
    """

    e = np.empty(6)

    # The position error
    e[:3] = Td[:3, -1] - T[:3, -1]

    R = Td[:3, :3] @ T[:3, :3].T

    li = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])

    if np.linalg.norm(li) < 1e-6:
        # If li is a zero vector (or very close to it)

        # diagonal matrix case
        if np.trace(R) > 0:
            # (1,1,1) case
            a = np.zeros((3,))
        else:
            a = np.pi / 2 * (np.diag(R) + 1)
    else:
        # non-diagonal matrix case
        ln = np.linalg.norm(li)
        a = math.atan2(ln, np.trace(R) - 1) * li / ln

    e[3:] = a

    return e