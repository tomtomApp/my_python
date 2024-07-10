import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class Listener(Node):
    def __init__(self):
        super().__init__('listener')
        self.create_subscription(Twist, 'cmd_vel', self.chatter_callback, 10)

    def chatter_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.linear.x)

def main(args=None):
    rclpy.init(args=args)
    listener = Listener()
    rclpy.spin(listener)
    rclpy.shutdown()
    '''
    try:
        listener = Listener()
        rclpy.spin(listener)
    finally:
        listener.destroy_node()
        rclpy.shutdown()
    '''

if __name__ == '__main__':
    main()
