from typing import Tuple, Optional, List
import numpy as np
import roboticstoolbox as rtb
from sensor_msgs.msg import JointState
import threading
from ros_middleware.base import RosMiddleware

class RosRobotInterface:
    def __init__(self, ros: RosMiddleware, joint_names: List[str], state_topic: str, command_topic: str):
        self.ros = ros
        self.joint_state_lock = threading.Lock()
        self.state_received = False
        self.joint_names = joint_names
        self.state_topic = state_topic
        self.joint_state_sub = self.ros.create_subscriber(state_topic, JointState, self.joint_state_callback)
        self.joint_state_pub = self.ros.create_publisher(command_topic, JointState)

    def joint_state_callback(self, msg: JointState):
        if len(msg.velocity) != len(msg.position):
            msg.velocity = [0.0] * len(msg.position)
        
        for name in self.joint_names:
            if name not in msg.name:
                print(f"Received invalid joint state message. Missing joint name: {name}")
                return
        
        indices = {name: i for i, name in enumerate(msg.name)}
        position = np.array([msg.position[indices[name]] for name in self.joint_names])
        velocity = np.array([msg.velocity[indices[name]] for name in self.joint_names])
        
        with self.joint_state_lock:
            self.q = position
            self.qd = velocity
            self.state_received = True


    def get_state(self) -> Tuple[np.ndarray, np.ndarray]:
        if not self.state_received:
            print(f"Waiting for robot joint state on topic {self.state_topic}")
            while not self.state_received:
                pass
            print("Robot joint state received")
        with self.joint_state_lock:
            return self.q, self.qd

    def set_joint_state(self, q: Optional[np.ndarray] = None, qd: Optional[np.ndarray] = None) -> None:
        if qd is None and q is None:
            return

        if qd is None:
            msg = JointState()
            msg.header.stamp = self.ros.get_timestamp()
            msg.name = self.joint_names
            msg.position = q.tolist()
            self.joint_state_pub.publish(msg)

        if q is None:
            msg = JointState()
            msg.header.stamp = self.ros.get_timestamp()
            msg.name = self.joint_names
            msg.velocity = qd.tolist()
            self.joint_state_pub.publish(msg)
