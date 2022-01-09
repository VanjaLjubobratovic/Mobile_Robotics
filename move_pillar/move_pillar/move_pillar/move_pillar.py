import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SpawnModel, GetWorldProperties

from ament_index_python.packages import get_package_share_directory


PACKAGE_NAME = 'move_pillar'


class MovePillar(Node):
    def __init__(self):
        super().__init__(PACKAGE_NAME)

        self.spawn_model('unit_cylinder', 'pillar.sdf')

        # Creating subscriber for 2D goal pose
        self.create_subscription(PoseStamped, '/goal_pose', self.move_pillar_callback, 10)

        # Creating publisher for /gazebo/set_model_state
        self.model_state_publisher = self.create_publisher(ModelState, '/gazebo/set_model_state', 10)

    def spawn_model(self, model_name, model_filename):
        self.get_logger().warning(f'Trying to spawn model: {model_name}')
        # Check if model already exists
        if not self.check_model_exists(model_name):
            # Loading pillar model
            model = SpawnModel.Request()
            model.model_name = model_name
            model.model_xml = open(f'{get_package_share_directory(PACKAGE_NAME)}/models/{model_filename}', 'r').read()

            # Creating a service client
            spawn_model_client = self.create_client(SpawnModel, '/gazebo/spawn_sdf_model')

            while not spawn_model_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().warning('Service /gazebo/spawn_sdf_model not available, waiting again...')

            future = spawn_model_client.call_async(model)
            while not future.done():
                rclpy.spin_once(self)

            if future.result().success:
                self.get_logger().info('Model spawned')
            else:
                self.get_logger().warning(f'{future.result().status_message}')
        else:
            self.get_logger().warning('Model already existing')

    def check_model_exists(self, model_name):
        properties = GetWorldProperties.Request()

        world_properties = self.create_client(GetWorldProperties, '/gazebo/get_world_properties')
        while not world_properties.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning('Service /gazebo/get_world_properties not available, waiting again...')

        future = world_properties.call_async(properties)
        while not future.done():
            rclpy.spin_once(self)

        return model_name in future.result().model_names

    def move_pillar_callback(self, msg):
        if msg.pose.position is None:
            return

        # Moving pillar to new position
        new_pillar_position = ModelState()
        new_pillar_position.model_name = 'unit_cylinder'
        new_pillar_position.pose.position.x = msg.pose.position.x
        new_pillar_position.pose.position.y = msg.pose.position.y
        new_pillar_position.pose.position.z = 1.0

        self.model_state_publisher.publish(new_pillar_position)

        # Outputting the new pillar position
        self.get_logger().info(f'Pillar position: x: {msg.pose.position.x}, y: {msg.pose.position.y}')


def main(args=None):
    rclpy.init(args=args)

    move_pillar = MovePillar()
    rclpy.spin(move_pillar)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
