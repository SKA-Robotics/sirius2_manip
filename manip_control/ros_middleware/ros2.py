import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
import threading


class Ros2Middleware:
    def __init__(self, node_name: str):
        self.node_name = node_name

    def start(self):
        rclpy.init()
        self.node = Node(self.node_name)
        self.executor = SingleThreadedExecutor()
        self.ros2_thread = threading.Thread(target=self._spin)
        self.ros2_thread.daemon = True
        self.ros2_thread.start()
    
    def _spin(self):
        self.executor.add_node(self.node)
        try:
            self.executor.spin()
        except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
            pass
        finally:
            self.executor.remove_node(self.node)

    def create_publisher(self, topic_name: str, message_type):
        return self.node.create_publisher(message_type, topic_name, qos_profile=10)
    
    def create_subscriber(self, topic_name: str, message_type, callback):
        return self.node.create_subscription(message_type, topic_name, callback, qos_profile=10)

    def stop(self):
        self.executor.shutdown()
        self.ros2_thread.join()
        self.node.destroy_node()
        rclpy.try_shutdown()
    
    def get_timestamp(self):
        return self.node.get_clock().now().to_msg()