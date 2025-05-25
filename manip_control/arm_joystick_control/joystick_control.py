from enum import Enum
from typing import List, Dict
from ros_middleware.base import RosMiddleware
from arm_joystick_control.ros_joy_receiver import RosJoyReceiver
from arm_joystick_control.ros_command_sender import RosCommandSender
from arm_joystick_control.utils import max_abs, trig_to_axis
import time
import yaml

class JoystickControlConfig:
    max_linear_velocity: float = 0.2
    max_angular_velocity: float = 0.4
    max_joint_velocities: List[float] = [0.5, 0.4, 0.4, 0.9, 0.8, 1.0]
    node_name: str = "arm_joystick_control"
    base_frame_id: str = "base_link"
    ee_frame_id: str = "tool"
    twist_topic: str = "/twist_cmd"
    joint_topic: str = "/joint_cmd"
    gripper_topic: str = "/gripper_cmd"
    preset_request_topic: str = "/preset_request"
    joy_topic: str = "/joy"
    ros_version: str = "ros1"

DEFAULT_CONFIG = JoystickControlConfig()

MANIP_PRESET_DATABASE = {
    "ik_ready": [0.0, -0.5, 1.85, -1.53, -0.96, -0.06],
}

class JoystickControl():

    class Axis(Enum):
        LINEAR_X = 0
        LINEAR_Y = 1
        LINEAR_Z = 2
        ANGULAR_X = 3
        ANGULAR_Y = 4
        ANGULAR_Z = 5
        GRIPPER = 6
        JOINT_1 = 7
        JOINT_2 = 8
        JOINT_3 = 9
        JOINT_4 = 10
        JOINT_5 = 11
        JOINT_6 = 12
        
    class Button(Enum):
        SET_FRAME_TOOL = 1
        SET_FRAME_BASE = 2
        SET_JOINT_MODE = 3
        CHANGE_MOVEMENT_MODE = 4
        GOTO_IK_READY = 5

    class SpaceMode(Enum):
        CARTESIAN = 0
        JOINT = 1

    class MovementMode(Enum):
        LINEAR = 0
        ANGULAR = 1

    def __init__(self, config: JoystickControlConfig = DEFAULT_CONFIG):
        self.config = config
        self.movement_mode = self.MovementMode.LINEAR
        self.space_mode = self.SpaceMode.JOINT
        self.frame = config.base_frame_id
        self.gui = None

        self.ros = self._start_ros(config)
        self.ros.start()
        self.joy_receiver = RosJoyReceiver(self.ros, config.joy_topic)
        self.command_sender = RosCommandSender(self.ros, config.twist_topic, config.joint_topic, config.gripper_topic, config.preset_request_topic)
    
    def run(self):
        self.joy_receiver.register_callback(self.receive_command)
        try:
            while True:
                time.sleep(0.02)
        except KeyboardInterrupt:
            pass
        finally:
            self.ros.stop()

    def receive_command(self, raw_axes: List[float], raw_buttons: List[int]):
        buttons = self._process_buttons(raw_buttons)
        self._handle_buttons(buttons)
        if buttons[self.Button.GOTO_IK_READY]:
            self._send_preset_request("ik_ready")
        else:
            self._update_gui(raw_axes, raw_buttons)
            axes  = self._process_axes(raw_axes)
            self._publish_command(axes)
            self._control_gripper(axes[self.Axis.GRIPPER])
    
    def _process_axes(self, axes: List[float]) -> Dict[Axis, float]:
        return  {
            self.Axis.LINEAR_X: axes[1],
            self.Axis.LINEAR_Y: max_abs(axes[0], axes[3]),
            self.Axis.LINEAR_Z: axes[4],
            self.Axis.ANGULAR_X: -axes[0],
            self.Axis.ANGULAR_Y: max_abs(axes[1], axes[4]),  
            self.Axis.ANGULAR_Z: axes[3],
            self.Axis.GRIPPER: max_abs(trig_to_axis(axes[2]), trig_to_axis(axes[5])),
            self.Axis.JOINT_1: axes[0],
            self.Axis.JOINT_2: axes[1],
            self.Axis.JOINT_3: -axes[4],
            self.Axis.JOINT_4: -axes[3],
            self.Axis.JOINT_5: -axes[1],
            self.Axis.JOINT_6: -axes[0],
        }
    
    def _process_buttons(self, buttons: List[int]) -> Dict[Button, bool]:
        return {
            self.Button.SET_FRAME_BASE: buttons[0],
            self.Button.SET_FRAME_TOOL: buttons[1],
            self.Button.SET_JOINT_MODE: buttons[2],
            self.Button.CHANGE_MOVEMENT_MODE: buttons[4] or buttons[5],
            self.Button.GOTO_IK_READY: buttons[3],
        }

    def _handle_buttons(self, buttons: Dict[Button, bool]):
        if buttons[self.Button.SET_FRAME_TOOL]:
            self.frame = self.config.ee_frame_id
            self.space_mode = self.SpaceMode.CARTESIAN
        elif buttons[self.Button.SET_FRAME_BASE]:
            self.frame = self.config.base_frame_id
            self.space_mode = self.SpaceMode.CARTESIAN
        elif buttons[self.Button.SET_JOINT_MODE]:
            self.space_mode = self.SpaceMode.JOINT

        if buttons[self.Button.CHANGE_MOVEMENT_MODE]:
            self.movement_mode = self.MovementMode.ANGULAR
        else:
            self.movement_mode = self.MovementMode.LINEAR
        
    def _publish_command(self, axes: Dict[Axis, float]):
        if self.space_mode == self.SpaceMode.CARTESIAN:
            if self.movement_mode == self.MovementMode.LINEAR:
                command = [
                    axes[self.Axis.LINEAR_X] * self.config.max_linear_velocity,
                    axes[self.Axis.LINEAR_Y] * self.config.max_linear_velocity,
                    axes[self.Axis.LINEAR_Z] * self.config.max_linear_velocity,
                    0.0,
                    0.0,
                    0.0,
                ]
            elif self.movement_mode == self.MovementMode.ANGULAR:
                command = [
                    0.0,
                    0.0,
                    0.0,
                    axes[self.Axis.ANGULAR_X] * self.config.max_angular_velocity,
                    axes[self.Axis.ANGULAR_Y] * self.config.max_angular_velocity,
                    axes[self.Axis.ANGULAR_Z] * self.config.max_angular_velocity,
                ]
            self.command_sender.send_twist_command(command, self.frame)

        elif self.space_mode == self.SpaceMode.JOINT:
            if self.movement_mode == self.MovementMode.LINEAR:
                command = [
                    axes[self.Axis.JOINT_1] * self.config.max_joint_velocities[0],
                    axes[self.Axis.JOINT_2] * self.config.max_joint_velocities[1],
                    axes[self.Axis.JOINT_3] * self.config.max_joint_velocities[2],
                    axes[self.Axis.JOINT_4] * self.config.max_joint_velocities[3],
                    0.0,
                    0.0,
                ]
            elif self.movement_mode == self.MovementMode.ANGULAR:
                command = [
                    0.0,
                    0.0,
                    axes[self.Axis.JOINT_3] * self.config.max_joint_velocities[2],
                    axes[self.Axis.JOINT_4] * self.config.max_joint_velocities[3],
                    axes[self.Axis.JOINT_5] * self.config.max_joint_velocities[4],
                    axes[self.Axis.JOINT_6] * self.config.max_joint_velocities[5],
                ]
            self.command_sender.send_joint_command(command)
    

    def _control_gripper(self, gripper_axis: float):
        gripper_position = (1 - gripper_axis)
        self.command_sender.send_gripper_command(gripper_position)
    
    def add_gui(self, gui):
        self.gui = gui
    
    def _update_gui(self, axes, buttons):
        if self.gui is not None:
            self.gui.update(axes, buttons, self.space_mode, self.movement_mode, self.frame)
    
    def _start_ros(self, config: JoystickControlConfig) -> RosMiddleware:
        if config.ros_version == "ros2":
            from ros_middleware.ros2 import Ros2Middleware
            ros = Ros2Middleware(config.node_name)
        elif config.ros_version == "ros1":
            from ros_middleware.ros1 import Ros1Middleware
            ros = Ros1Middleware(config.node_name)
        else:
            raise ValueError(f"Invalid ROS version: {config.ros_version}. Must be either 'ros1' or 'ros2'.")
        return ros
    
    def _send_preset_request(self, preset_name: str):
        if preset_name not in MANIP_PRESET_DATABASE:
            print(f"Manip preset {preset_name} is not defined")
            return
        target_q = MANIP_PRESET_DATABASE[preset_name]
        self.command_sender.send_preset_command(target_q)


if __name__ == "__main__":
    print("Starting joystick control")
    joystick_control = JoystickControl()
    joystick_control.run()
    print("Joystick control finished")