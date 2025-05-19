import roboticstoolbox as rtb
from pathlib import Path
from typing import Optional

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