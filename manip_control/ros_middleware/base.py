
class RosMiddleware:
    def __init__(self, node_name: str):
        self.node_name = node_name

    def start(self):
        ...

    def stop(self):
        ...
    
    def create_publisher(self, topic_name: str, message_type):
        ...

    def create_subscriber(self, topic_name: str, message_type, callback):
        ...
    
    def get_timestamp(self):
        ...