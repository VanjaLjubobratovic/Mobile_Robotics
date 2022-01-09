import rclpy
import math
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import Twist, Point, PoseStamped
from nav_msgs.msg import Odometry
import time

goal_x = 0.0
goal_y = 0.0

class OdometryStuff(Node):
    def __init__(self):
        super().__init__('robot_mover')

        self.publisher = self.create_publisher(Twist, 'cmd_vel', 1)

        self.subscriber = self.create_subscription(
            PoseStamped,
            'goal_pose',
            self.change_goal,
            10
        )

        self.subscriberGoal = self.create_subscription(
            Odometry,
            'odom',
            self.timer_callback,
            10
        )

    def change_goal(self, msg):
        global goal_x
        global goal_y
        goal_x = msg.pose.position.x
        goal_y = msg.pose.position.y
        #self.get_logger().info(f'GOAL SET: {goal_x, goal_y}') 

    def timer_callback(self, msg):
        moveMsg = Twist()


        current = Point()
        current.x = msg.pose.pose.position.x
        current.y = msg.pose.pose.position.y

        inc_x = goal_x - current.x
        inc_y = goal_y - current.y
        angle_goal = math.atan2(inc_y, inc_x)

        t3 = +2.0 * (msg.pose.pose.orientation.w * msg.pose.pose.orientation.z + msg.pose.pose.orientation.y * msg.pose.pose.orientation.x)
        t4 = +1.0 - 2.0 * (msg.pose.pose.orientation.y **2 + msg.pose.pose.orientation.z ** 2)
        theta = math.atan2(t3, t4)

        if abs(angle_goal - theta) > 0.5:
            if abs(inc_x) <= 0.04 and abs(inc_y) <= 0.04:
                moveMsg.linear.x = 0.0
                moveMsg.angular.z = 0.0
            else:
                moveMsg.linear.x = 0.0
                moveMsg.angular.z = 0.4 * (angle_goal - theta)
        else:    
            moveMsg.linear.x = 0.3
            moveMsg.angular.z = 0.0
            #self.get_logger().info(f'\nMOVE\n')

        self.publisher.publish(moveMsg)
        #self.get_logger().info(f'GOAL: {goal_x, goal_y}')
        #self.get_logger().info(f'CURRENT: {round(current.x, 4), round(current.y, 4)}, \n ANGLE GOAL: {angle_goal} \n ANGLE DIFF: {abs(angle_goal - theta)},  \n THETA: {theta}, \n INC: {round(inc_x, 3), round(inc_y,3)}')

def main(args = None):
        rclpy.init(args = args)
        robot_mover = OdometryStuff()
        rclpy.spin(robot_mover)
        rclpy.shutdown()

if __name__ == "__main__":
    main()