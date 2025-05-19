from typing import List
from geometry_msgs.msg import TwistStamped
from control_msgs.msg import JointJog
from std_msgs.msg import Float64
from ros_middleware.base import RosMiddleware

class RosCommandSender:
    def __init__(self, ros: RosMiddleware, twist_topic: str, joint_topic: str, gripper_topic: str):
        self.ros = ros
        self.twist_publisher = self.ros.create_publisher(twist_topic, TwistStamped)
        self.joint_publisher = self.ros.create_publisher(joint_topic, JointJog)
        self.gripper_publisher = self.ros.create_publisher(gripper_topic, Float64)

    def send_twist_command(self, twist_data: List[float], frame_id: str):
        command = TwistStamped()
        command.header.stamp = self.ros.get_timestamp()
        command.header.frame_id = frame_id
        command.twist.linear.x = twist_data[0]
        command.twist.linear.y = twist_data[1]
        command.twist.linear.z = twist_data[2]
        command.twist.angular.x = twist_data[3]
        command.twist.angular.y = twist_data[4]
        command.twist.angular.z = twist_data[5]
        self.twist_publisher.publish(command)

    def send_joint_command(self, joint_data: List[float]):
        command = JointJog()
        command.header.stamp = self.ros.get_timestamp()
        command.velocities = joint_data
        self.joint_publisher.publish(command)

    def send_gripper_command(self, gripper_position: float):
        command = Float64()
        command.data = gripper_position
        self.gripper_publisher.publish(command)
