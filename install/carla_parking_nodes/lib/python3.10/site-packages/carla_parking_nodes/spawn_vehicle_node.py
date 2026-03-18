import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import carla


class SpawnVehicleNode(Node):

    def __init__(self):
        super().__init__('spawn_vehicle_node')

        self.publisher = self.create_publisher(
            String,
            '/carla_parking/spawn_status',
            10
        )

        self.spawn_vehicle()

    def spawn_vehicle(self):

        msg = String()

        try:
            client = carla.Client('localhost', 2000)
            client.set_timeout(10.0)

            world = client.get_world()

            bp_lib = world.get_blueprint_library()
            bp = bp_lib.find('vehicle.modular.mod01')

            spawn_transform = carla.Transform(
                carla.Location(x=285.0, y=-235.0, z=1.0),
                carla.Rotation(yaw=0.0)
            )

            vehicle = world.try_spawn_actor(bp, spawn_transform)

            if vehicle is None:
                msg.data = "Spawn failed"
            else:
                msg.data = f"Vehicle spawned with id {vehicle.id}"

        except Exception as e:
            msg.data = f"Spawn error: {str(e)}"

        self.publisher.publish(msg)
        self.get_logger().info(msg.data)


def main(args=None):

    rclpy.init(args=args)

    node = SpawnVehicleNode()

    rclpy.spin_once(node, timeout_sec=1.0)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()