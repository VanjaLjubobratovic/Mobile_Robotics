import rclpy
import math
from rclpy.node import Node
from rclpy.utilities import get_available_rmw_implementations
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import Twist, Point, PoseStamped
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import SpawnModel, GetWorldProperties
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from std_srvs.srv import Empty
import sys

class OdometryStuff(Node):
    angle = -1
    min_distance = -1
    cur_x = 0.0
    cur_y = 0.0
    orient = 0.0
    fl = False

    def __init__(self):
        super().__init__('betterRobotMover')
        global fl
        fl = False

        self.publisher = self.create_publisher(Twist, 'cmd_vel', 1)
        self.marker_publisher = self.create_publisher(Marker, 'visualization_marker', 1)

        self.srv = self.create_service(Empty, 'gotoClosest', self.read_request)

        self.subscribePillar = self.create_subscription(
            LaserScan,
            'scan',
            self.read_scan,
            10
        )

        self.subscriberGoal = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

    def read_request(self, request, response):
        global fl
        fl = True
        return response
    
    def read_scan(self, msg):
        global min_distance
        global angle
        global fl
        direction = 1
        distances = msg.ranges
        min_distance = min(distances)
        angle = distances.index(min_distance) * msg.angle_increment

        if fl == False:
            return

        if 0 <= angle <= 3.14:
            direction = 1
        else:
            direction = -1

        moveMsg = Twist()
        if min_distance > 100 or angle <= 0:
            return
        
        if min_distance <= 0.32:
            moveMsg.linear.x = 0.0
            moveMsg.angular.z = 0.0
            fl = False
            #self.get_logger().info(f'Angle is {angle} and distance is {min_distance}') 
        elif angle < 0.3:
            if min_distance <= 0.4:
                moveMsg.linear.x = 0.1
                moveMsg.angular.z = 0.0
            else:
                moveMsg.linear.x = 0.25
                moveMsg.angular.z = 0.0
        else:
            moveMsg.linear.x = 0.0
            moveMsg.angular.z = 0.4 * direction
        self.publisher.publish(moveMsg)
        #self.get_logger().info(f'Angle is {angle} and distance is {min_distance}') 
    
    def odom_callback(self, msg):
        global cur_x
        global cur_y
        global orient

        cur_x = msg.pose.pose.position.x
        cur_y = msg.pose.pose.position.y
        orient = 2 * math.acos(msg.pose.pose.orientation.w)

        goal = Point()
        goal.x = cur_x + min_distance * math.cos(angle + orient)
        goal.y = cur_y + min_distance * math.sin(angle + orient)

        marker = Marker()
        marker.header.frame_id = 'odom'
        marker.type = 2
        marker.id = 0
        marker.action = Marker.ADD

        marker.pose.position.x = goal.x
        marker.pose.position.y = goal.y
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2

        marker.color.a = 0.5
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        self.marker_publisher.publish(marker)


def main(args = None):
        global fl
        rclpy.init(args = args)
        better_robot_mover = OdometryStuff()
        rclpy.spin(better_robot_mover)
        rclpy.shutdown()

if __name__ == "__main__":
    main()