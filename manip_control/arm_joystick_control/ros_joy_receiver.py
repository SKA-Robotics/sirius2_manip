from sensor_msgs.msg import Joy
from typing import Callable, List
from ros_middleware.base import RosMiddleware

class RosJoyReceiver:
    def __init__(self, ros: RosMiddleware, topic: str):
        self.ros = ros
        self.joy_subscriber = self.ros.create_subscriber(topic, Joy, self.joy_callback)
        self.callback = None

    def joy_callback(self, msg: Joy):
        if self.callback is not None:
            self.callback(msg.axes, msg.buttons)

    def register_callback(self, callback: Callable[[List[float], List[int]], None]):
        self.callback = callback