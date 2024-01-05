import rclpy
from std_msgs.msg import String

class MyPublisher:
    def __init__(self):
        # Initialize the ROS 2 node
        rclpy.init()
        self.node = rclpy.create_node('my_publisher')

        # Create a publisher that publishes messages of type String to the 'my_topic' topic
        self.publisher = self.node.create_publisher(String, 'my_topic', 10)

        # Create a timer to periodically publish a message
        self.timer = self.node.create_timer(1.0, self.publish_message)

    def publish_message(self):
        # Create a message of type String and publish it
        msg = String()
        msg.data = 'Hello, ROS 2!'
        self.publisher.publish(msg)
        self.node.get_logger().info('Message published: {}'.format(msg.data))

    def run(self):
        # Spin the ROS 2 node to process callbacks
        rclpy.spin(self.node)

        # Cleanup
        self.node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    my_publisher = MyPublisher()
    my_publisher.run()
