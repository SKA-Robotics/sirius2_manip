import threading
import rospy


class Ros1Middleware:
    def __init__(self, node_name: str):
        self.node_name = node_name

    def start(self):
        rospy.init_node(self.node_name)
        self.ros1_thread = threading.Thread(target=self._spin)
        self.ros1_thread.daemon = True
        self.ros1_thread.start()
    
    def _spin(self):
        try:
            rospy.spin()
        except KeyboardInterrupt:
            pass

    def create_publisher(self, topic_name: str, message_type):
        return rospy.Publisher(topic_name, message_type, queue_size=10)
    
    def create_subscriber(self, topic_name: str, message_type, callback):
        return rospy.Subscriber(topic_name, message_type, queue_size=10)

    def stop(self):
        rospy.signal_shutdown("Stopped by the user")
    
    def get_timestamp(self):
        return rospy.Time.now()