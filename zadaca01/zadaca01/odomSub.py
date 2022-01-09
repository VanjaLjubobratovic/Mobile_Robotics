import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class OdometrySub(Node):
    def __init__(self):
        super().__init__('odometry_sub')
        self.subscriber = self.create_subscription(
            Odometry,
            'odom',
            self.listener_callback,
            10
        )
    
    def listener_callback(self, msg):
        self.get_logger().info(f'Position(X, Y): {round(msg.pose.pose.position.x, 5), round(msg.pose.pose.position.y, 5)}')

def main(args = None):
    rclpy.init(args = args)
    odometry_sub = OdometrySub()
    rclpy.spin(odometry_sub)
    rclpy.shutdown

if __name__ == "__main__":
    main()