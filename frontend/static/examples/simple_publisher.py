import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')
        self.publisher = self.create_publisher(String, 'robot_status', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.publish_status)
        self.status_counter = 0

    def publish_status(self):
        msg = String()
        msg.data = f'Robot status: Operational - {self.status_counter}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')
        self.status_counter += 1

def main(args=None):
    rclpy.init(args=args)
    publisher = SimplePublisher()

    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        pass
    finally:
        publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()