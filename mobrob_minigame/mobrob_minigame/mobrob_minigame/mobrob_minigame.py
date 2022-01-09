import rclpy
from rclpy.node import Node

from std_srvs.srv import Empty
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import numpy as np


PACKAGE_NAME = 'mobrob_minigame'


class MobRobMinigame(Node):
    def __init__(self):
        super().__init__(PACKAGE_NAME)

        self.world_size = 7
        self.origin_distance = self.world_size / 2.0 - 0.2

        self.robot_position = (0, 0)
        self.ball_position = (5, 0)
        self.ball_remaining = 5
        self.game_started = False

        # Creating publisher for /gazebo/set_model_state
        self.model_state_publisher = self.create_publisher(ModelState, '/gazebo/set_model_state', 10)

        self.create_subscription(Odometry, '/robot/odom', self.odometry_callback, 1)
        self.create_subscription(Twist, '/robot/cmd_vel', self.velocity_callback, 1)
        self.twist_publisher = self.create_publisher(Twist, '/mobrob/cmd_vel', 1)

        self.create_service(Empty, 'start_game', self.start_game_callback)

    def start_game_callback(self, request, response):
        self.get_logger().info(f'Service /start_game called')
        self.ball_remaining = 5
        self.game_started = True
        self.move_ball_random()
        return response

    def distance(self, a, b):
        return np.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

    def odometry_callback(self, msg):
        self.robot_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)

        self.check_distance_ball()

    def check_distance_ball(self):
        if self.distance(self.robot_position, self.ball_position) < 0.25:
            self.ball_remaining -= 1
            self.get_logger().info(f'Ball picked, remaining: {self.ball_remaining}')

            if self.ball_remaining <= 0:
                self.move_ball(5.0, 0.0)
                self.game_started = False
                self.get_logger().warning(f'Game finished')
                return

            self.move_ball_random()

    def get_random(self):
        return -(self.world_size / 2.0 - 1) + np.random.random() * (self.world_size - 2)

    def move_ball_random(self):
        while True:
            bx = self.get_random()
            by = self.get_random()

            if self.distance(self.robot_position, (bx, by)) >= 1:
                break

        self.move_ball(bx, by)

    def move_ball(self, bx, by):
        self.ball_position = (bx, by)

        # Moving ball to new position
        new_ball_position = ModelState()
        new_ball_position.model_name = 'simple_ball'
        new_ball_position.pose.position.x = bx
        new_ball_position.pose.position.y = by
        new_ball_position.pose.position.z = 0.15

        self.model_state_publisher.publish(new_ball_position)

        # Outputting the new pillar position
        self.get_logger().info(f'Ball position: x: {bx}, y: {by}')

    def velocity_callback(self, msg):
        max_lin_vel = 0.25
        max_ang_vel = 0.5

        if self.game_started:
            if msg.linear.x > max_lin_vel:
                msg.linear.x = max_lin_vel
            elif msg.linear.x < -max_lin_vel:
                msg.linear.x = -max_lin_vel

            if msg.angular.z > max_ang_vel:
                msg.angular.z = max_ang_vel
            elif msg.angular.z < -max_ang_vel:
                msg.angular.z = -max_ang_vel
        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
        msg.linear.y = 0.0

        self.twist_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    mobrob_minigame = MobRobMinigame()
    rclpy.spin(mobrob_minigame)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
