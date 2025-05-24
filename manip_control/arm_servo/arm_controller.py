import time
import math
import numpy as np
import spatialmath as sm
import spatialgeometry as sg
import roboticstoolbox as rtb
from typing import List
from arm_servo.command import Command, CommandType
from arm_servo.ros_robot_interface import RosRobotInterface
from arm_servo.ros_command_receiver import RosCommandReceiver
from arm_servo.motion_control import TwistController, PoseServo
from arm_servo.ros_visualizer import RosVisualizer
from arm_servo.utils import load_urdf


class ArmControllerConfig:
    # Control params
    control_frequency: float = 30
    command_timeout: float = 0.3
    servo_gain: np.ndarray = np.array([20, 20, 20, 10, 10, 10])
    singularity_avoidance_A: float = 1.6    # Values based on paper
    singularity_avoidance_B: float = 1.491  # Values based on paper
    max_setpoint_distance: float = 0.05

    # Robot params
    robot_urdf_path: str = "robot_urdf/sirius2_manip.urdf.xacro"

    # ROS params
    ros_version: str = "ros1"
    node_name: str = "manip_control"
    twist_topic: str = "/twist_cmd"
    pose_topic: str = "/pose_cmd"
    joint_topic: str = "/joint_cmd"
    robot_state_topic: str = "/manip/joint_states"
    robot_command_topic: str = "/manip/set_joint_states"
    robot_joint_names: List[str] = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

    # Visualization params
    use_sim: bool = False
    sim_browser: str = "firefox"


DEFAULT_CONFIG = ArmControllerConfig()


class ArmController:
    def __init__(self, config: ArmControllerConfig, robot_interface: RosRobotInterface, command_interface: RosCommandReceiver, env, viz):
        self.config = config
        self.command = None
        self.robot_interface = robot_interface
        self.command_interface = command_interface
        self.robot = load_urdf(config.robot_urdf_path)
        self.pose_servo = PoseServo(self.robot, gain=config.servo_gain)
        self.twist_controller = TwistController(self.robot, config.singularity_avoidance_A, config.singularity_avoidance_B)
        self.goal = sg.Axes(0.1)
        self.ee_axes = sg.Axes(0.1)
        self.command_timeout = config.command_timeout
        self.env = env
        self.viz = viz
        self.compute_duration = 0
        self.loop_duration = 100
        reach = self.robot.reach
        print(reach)
        print(type(reach))

    def loop(self):
        self.robot.q, _ = self.robot_interface.get_state()
        self.goal.T = self.robot.fkine(self.robot.q, tool=self.robot.tool)

        dt = 1 / self.config.control_frequency
        start_time = time.monotonic()
        while True:
            loop_start_time = time.monotonic()
            self.command = self.command_interface.pop_command()
            qd = self.compute_control(dt)
            self.robot.q = self.robot.q + qd * dt

            # Clamp joint values to the joint limits
            for i in range(self.robot.n):
                self.robot.q[i] = max(min(self.robot.q[i], self.robot.qlim[1][i]), self.robot.qlim[0][i])

            self.robot_interface.set_joint_state(q=self.robot.q)
            
            self.ee_axes.T = self.robot.fkine(self.robot.q, tool=self.robot.tool)
            compute_end_time = time.monotonic()

            if self.config.use_sim:
                self.env.step(dt)
            if self.viz:
                self.viz.visualize_goal_pose(self.goal.T, "base_link")
            
            # Try to make the loop run at the correct frequency
            sleep_time = dt - ((time.monotonic() - start_time) % dt)
            if sleep_time > 0:
                time.sleep(sleep_time)

            loop_end_time = time.monotonic()
            self.compute_duration = compute_end_time - loop_start_time
            self.loop_duration = loop_end_time - loop_start_time

    
    def compute_control(self, dt: float) -> np.ndarray:
        """
        Calculates the joint velocities required to execute the command.

        Returns:
            np.ndarray: The joint velocities required to execute the command.
        """
        if self.command is not None and self.command.timestamp < time.time() - self.command_timeout:
            print(f"Command timed out: {self.command.timestamp} < {time.time() - self.command_timeout}")
            self.command = None

        if self.command is None:
            return np.zeros(self.robot.n)

        if self.command.type == CommandType.END_EFFECTOR_POSE_CMD:
            self.goal.T = self.command.data
            ev = self.pose_servo.compute_pose_control(self.goal.T)
            qd = self.twist_controller.compute_twist_control(ev)
        elif self.command.type == CommandType.END_EFFECTOR_TWIST_CMD:
            new_goal_T = self.goal.T
            new_goal_T = new_goal_T @ sm.SE3.Trans(self.command.data[0] * dt, self.command.data[1] * dt, self.command.data[2] * dt).A
            new_goal_T = new_goal_T @ sm.SE3.RPY(self.command.data[3] * dt, self.command.data[4] * dt, self.command.data[5] * dt).A
            new_goal_T_position_error = np.linalg.norm(self.ee_axes.T[0:3, 3] - new_goal_T[0:3, 3])
            if new_goal_T_position_error <= self.config.max_setpoint_distance:
                self.goal.T = new_goal_T # The maximum distance from ee to goal is limited to avoid going far out of range of the arm
            ev = self.pose_servo.compute_pose_control(self.goal.T)
            qd = self.twist_controller.compute_twist_control(ev)
        elif self.command.type == CommandType.JOINT_VELOCITY_CMD:
            if len(self.command.data) != self.robot.n:
                print(f"Command received for {len(self.command.data)} joints, but robot has {self.robot.n} joints")
                self.command = Command(CommandType.JOINT_VELOCITY_CMD, np.zeros(self.robot.n), time.time())
            self.goal.T = self.robot.fkine(self.robot.q, tool=self.robot.tool).A # reset the end effector pose goal
            qd = self.command.data
        # elif self.command.type == CommandType.TRAJECTORY_CMD:
        #     qd = self.command.data[self.trajectory_idx]
        #     self.trajectory_idx += 1
        #     if self.trajectory_idx >= len(self.command.data):
        #         self.trajectory_idx = 0
        #         self.running = False
        else:
            print(f"Unknown command type: {self.command.type}")
            return np.zeros(self.robot.n)
        return qd


    

def main():
    config = DEFAULT_CONFIG

    # Start ros2 integration
    if config.ros_version == "ros2":
        from ros_middleware.ros2 import Ros2Middleware
        ros = Ros2Middleware(config.node_name)
    elif config.ros_version == "ros1":
        from ros_middleware.ros1 import Ros1Middleware
        ros = Ros1Middleware(config.node_name)
    else:
        raise ValueError(f"Invalid ROS version: {config.ros_version}. Must be either 'ros1' or 'ros2'.")

    ros.start()

    robot_interface = RosRobotInterface(ros, config.robot_joint_names, config.robot_state_topic, config.robot_command_topic)
    command_interface = RosCommandReceiver(ros, config.twist_topic, config.pose_topic, config.joint_topic)

    if config.use_sim:
        from swift import Swift
    env = Swift() if config.use_sim else None

    # viz = None
    viz = RosVisualizer(ros, "/manip_goal")

    arm_controller = ArmController(config, robot_interface, command_interface, env, viz)

    if config.use_sim:
        env.launch(realtime=False, browser=config.sim_browser)
        env.add(arm_controller.robot)
        env.add(arm_controller.ee_axes)
        env.add(arm_controller.goal)

    try:
        arm_controller.loop()

    except KeyboardInterrupt:
        pass
    finally:
        if config.use_sim:
            env.close()

        ros.stop()


if __name__ == "__main__":
    main()