import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import carla
import random


class SpawnVehicleNode(Node):

    def __init__(self):
        super().__init__('spawn_vehicle_node')

        # Publisher
        self.publisher = self.create_publisher(
            String,
            '/carla_parking/spawn_status',
            10
        )

        # Declare parameters
        self.declare_parameter('vehicle_blueprint', 'vehicle.*')
        self.declare_parameter('num_vehicles', 1)
        self.declare_parameter('random_spawn', True)

        self.declare_parameter('spawn_x', 285.0)
        self.declare_parameter('spawn_y', -235.0)
        self.declare_parameter('spawn_yaw', 0.0)

        self.spawn_vehicle()

    def spawn_vehicle(self):

        msg = String()

        try:

            client = carla.Client('localhost', 2000)
            client.set_timeout(10.0)

            world = client.get_world()
            bp_lib = world.get_blueprint_library()

            blueprint_name = self.get_parameter('vehicle_blueprint').value
            num_vehicles = self.get_parameter('num_vehicles').value
            random_spawn = self.get_parameter('random_spawn').value

            spawn_x = self.get_parameter('spawn_x').value
            spawn_y = self.get_parameter('spawn_y').value
            spawn_yaw = self.get_parameter('spawn_yaw').value

            # Blueprint selection
            if blueprint_name == "vehicle.*":
                vehicle_bp = random.choice(bp_lib.filter("vehicle.*"))

            else:
                vehicle_bp = bp_lib.find(blueprint_name)

            spawn_points = world.get_map().get_spawn_points()

            vehicles_spawned = []

            for i in range(num_vehicles):

                if random_spawn:
                    transform = random.choice(spawn_points)
                else:
                    transform = carla.Transform(
                        carla.Location(x=spawn_x, y=spawn_y, z=1.0),
                        carla.Rotation(yaw=spawn_yaw)
                    )

                vehicle = world.try_spawn_actor(vehicle_bp, transform)
                if random_spawn:
                    vehicle.set_autopilot(True)

                if vehicle is not None:
                    vehicles_spawned.append(vehicle.id)

            if len(vehicles_spawned) == 0:
                msg.data = "Spawn failed"
            else:
                msg.data = f"Spawned vehicles: {vehicles_spawned}"

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