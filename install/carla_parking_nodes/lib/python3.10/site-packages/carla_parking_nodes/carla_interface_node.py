#!/usr/bin/env python3

import math
import queue
from typing import Dict, Optional

import numpy as np
import carla

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg import String, Float32, Header
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, PointCloud2, PointField
from cv_bridge import CvBridge


from carla_parking_msgs.msg import VehicleControl


IM_WIDTH = 640
IM_HEIGHT = 480


SENSOR_LAYOUT = {
    "front_left_rgb": {
        "type": "sensor.camera.rgb",
        "xyz": (3.198, -0.822, 0.442),
        "rpy": (0.0, 0.0, 0.0),
        "topic": "/sensors/front_left_rgb/image_raw",
        "frame_id": "front_left_rgb_frame",
    },
    "front_right_rgb": {
        "type": "sensor.camera.rgb",
        "xyz": (3.198, 0.822, 0.442),
        "rpy": (0.0, 0.0, 0.0),
        "topic": "/sensors/front_right_rgb/image_raw",
        "frame_id": "front_right_rgb_frame",
    },
    "front_lidar": {
        "type": "sensor.lidar.ray_cast",
        "xyz": (0.486, 0.0, 0.51),          # temporary ibeo6-based placeholder
        "rpy": (0.007, -0.05, 0.00735),
        "topic": "/sensors/front_lidar/points",
        "frame_id": "front_lidar_frame",
    },
    "rear_left_lidar": {
        "type": "sensor.lidar.ray_cast",
        "xyz": (-0.671, 0.839, 0.402),      # temporary ibeo2-based placeholder
        "rpy": (0.021, 0.019, 2.125),
        "topic": "/sensors/rear_left_lidar/points",
        "frame_id": "rear_left_lidar_frame",
    },
    "rear_right_lidar": {
        "type": "sensor.lidar.ray_cast",
        "xyz": (-0.671, -0.839, 0.402),     # temporary ibeo10-based placeholder
        "rpy": (0.0, -0.007, -2.101),
        "topic": "/sensors/rear_right_lidar/points",
        "frame_id": "rear_right_lidar_frame",
    },
}


class CarlaInterfaceNode(Node):
    def __init__(self):
        super().__init__('carla_interface_node')

        self.declare_parameter('draw_sensor_debug', True)


        self.bridge = CvBridge()

        self.client: Optional[carla.Client] = None
        self.world: Optional[carla.World] = None
        self.bp_lib: Optional[carla.BlueprintLibrary] = None
        self.vehicle: Optional[carla.Actor] = None

        self.actors = []
        self.sensor_actors: Dict[str, carla.Actor] = {}

        self.sensor_publishers = {}
        self.sensor_queues = {}
        self.sensor_types = {}
        self.sensor_frame_ids = {}

        self.status_pub = self.create_publisher(String, '/carla_interface/status', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/ego/pose', 10)
        self.speed_pub = self.create_publisher(Float32, '/ego/speed', 10)
        self.yaw_pub = self.create_publisher(Float32, '/ego/yaw', 10)

        
        self.control_applied_pub = self.create_publisher(
            VehicleControl, '/vehicle/control_applied', 10
        )
        self.control_sub = self.create_subscription(
            VehicleControl,
            '/vehicle/control_cmd',
            self.control_cmd_callback,
            10
        )

        self.declare_parameter('host', 'localhost')
        self.declare_parameter('port', 2000)
        self.declare_parameter('timeout', 10.0)
        self.declare_parameter('vehicle_blueprint', 'vehicle.modular.mod01')

        self.declare_parameter('spawn_x', 285.45)
        self.declare_parameter('spawn_y', -235.73)
        self.declare_parameter('spawn_z', 1.0)
        self.declare_parameter('spawn_yaw', 0.0)

        self.declare_parameter(
            'chosen_sensors',
            [
                'front_left_rgb',
                'front_right_rgb',
                'front_lidar',
                'rear_left_lidar',
                'rear_right_lidar',
            ]
        )

        self.declare_parameter('ego_state_publish_rate', 20.0)
        self.declare_parameter('sensor_publish_rate', 50.0)

        self.connect_to_carla()
        self.spawn_ego_vehicle()
        self.setup_dynamic_sensor_interfaces()
        self.spawn_sensors()


        if bool(self.get_parameter('draw_sensor_debug').value):
            self.debug_timer = self.create_timer(0.1, self.debug_draw_loop)
        else:
            self.debug_timer = None

        ego_rate = float(self.get_parameter('ego_state_publish_rate').value)
        sensor_rate = float(self.get_parameter('sensor_publish_rate').value)

        self.ego_timer = self.create_timer(1.0 / ego_rate, self.publish_ego_state)
        self.sensor_timer = self.create_timer(1.0 / sensor_rate, self.publish_sensor_data)

        self.publish_status('carla_interface_node fully initialized')

    def publish_status(self, text: str):
        self.get_logger().info(text)

        try:
            if rclpy.ok() and self.status_pub is not None:
                msg = String()
                msg.data = text
                self.status_pub.publish(msg)
        except Exception:
            pass

    def connect_to_carla(self):
        host = self.get_parameter('host').value
        port = int(self.get_parameter('port').value)
        timeout = float(self.get_parameter('timeout').value)

        self.client = carla.Client(host, port)
        self.client.set_timeout(timeout)
        self.world = self.client.get_world()
        self.bp_lib = self.world.get_blueprint_library()

        self.publish_status(f'Connected to CARLA at {host}:{port}')

    def spawn_ego_vehicle(self):
        blueprint_id = self.get_parameter('vehicle_blueprint').value
        bp = self.bp_lib.find(blueprint_id)

        spawn_tf = carla.Transform(
            carla.Location(
                x=float(self.get_parameter('spawn_x').value),
                y=float(self.get_parameter('spawn_y').value),
                z=float(self.get_parameter('spawn_z').value),
            ),
            carla.Rotation(
                yaw=float(self.get_parameter('spawn_yaw').value)
            )
        )

        self.vehicle = self.world.try_spawn_actor(bp, spawn_tf)

        if self.vehicle is None:
            raise RuntimeError('Failed to spawn ego vehicle')

        self.actors.append(self.vehicle)
        self.publish_status(f'Spawned ego vehicle id={self.vehicle.id} type={blueprint_id}')

    def setup_dynamic_sensor_interfaces(self):
        chosen_sensors = list(self.get_parameter('chosen_sensors').value)

        for sensor_name in chosen_sensors:
            if sensor_name not in SENSOR_LAYOUT:
                self.get_logger().warn(f'Sensor {sensor_name} missing from SENSOR_LAYOUT')
                continue

            spec = SENSOR_LAYOUT[sensor_name]
            sensor_type = spec['type']
            topic = spec['topic']
            frame_id = spec['frame_id']

            self.sensor_types[sensor_name] = sensor_type
            self.sensor_frame_ids[sensor_name] = frame_id
            self.sensor_queues[sensor_name] = queue.Queue(maxsize=5)

            if sensor_type == 'sensor.camera.rgb':
                pub = self.create_publisher(Image, topic, qos_profile_sensor_data)
                self.sensor_publishers[sensor_name] = pub
                self.publish_status(f'Created camera publisher for {sensor_name} -> {topic}')

            elif sensor_type == 'sensor.lidar.ray_cast':
                pub = self.create_publisher(PointCloud2, topic, qos_profile_sensor_data)
                self.sensor_publishers[sensor_name] = pub
                self.publish_status(f'Created lidar publisher for {sensor_name} -> {topic}')

            else:
                self.get_logger().warn(
                    f'Unsupported sensor type {sensor_type} for sensor {sensor_name}'
                )

    def urdf_to_carla_transform(self, xyz, rpy):
        x_u, y_u, z_u = xyz
        roll_u, pitch_u, yaw_u = rpy

        x_c = x_u
        y_c = -y_u
        z_c = z_u

        roll_c = math.degrees(roll_u)
        pitch_c = math.degrees(pitch_u)
        yaw_c = -math.degrees(yaw_u)

        return carla.Transform(
            carla.Location(x=x_c, y=y_c, z=z_c),
            carla.Rotation(roll=roll_c, pitch=pitch_c, yaw=yaw_c),
        )

    def configure_sensor_blueprint(self, bp):
        if bp.id == 'sensor.camera.rgb':
            bp.set_attribute('image_size_x', str(IM_WIDTH))
            bp.set_attribute('image_size_y', str(IM_HEIGHT))
            bp.set_attribute('fov', '90')
            bp.set_attribute('sensor_tick', '0.05')

        elif bp.id == 'sensor.lidar.hss_lidar':
            bp.set_attribute('channels', '128')
            bp.set_attribute('range', '40')
            bp.set_attribute('rotation_frequency', '10')
            bp.set_attribute('horizontal_fov', '120')
            bp.set_attribute('horizontal_resolution', '0.1')
            bp.set_attribute('upper_fov', '12.9')
            bp.set_attribute('lower_fov', '-12.5')
            bp.set_attribute('sensor_tick', '0.1')
            bp.set_attribute('dropoff_general_rate', '0.0')
            bp.set_attribute('dropoff_intensity_limit', '1.0')
            bp.set_attribute('dropoff_zero_intensity', '0.0')
            bp.set_attribute('noise_stddev', '0.0')

        elif bp.id == 'sensor.lidar.ray_cast':
            bp.set_attribute('channels', '128')
            bp.set_attribute('range', '15')
            bp.set_attribute('points_per_second', '600000')
            bp.set_attribute('rotation_frequency', '20')
            bp.set_attribute('horizontal_fov', '130')
            bp.set_attribute('upper_fov', '12.5')
            bp.set_attribute('lower_fov', '-20.5')
            bp.set_attribute('sensor_tick', '0.05')
            bp.set_attribute('dropoff_general_rate', '0.0')
            bp.set_attribute('dropoff_intensity_limit', '1.0')
            bp.set_attribute('dropoff_zero_intensity', '0.0')
            bp.set_attribute('noise_stddev', '0.0')

    def spawn_sensors(self):
        chosen_sensors = list(self.get_parameter('chosen_sensors').value)

        for sensor_name in chosen_sensors:
            if sensor_name not in SENSOR_LAYOUT:
                continue

            spec = SENSOR_LAYOUT[sensor_name]
            bp = self.bp_lib.find(spec['type'])
            self.configure_sensor_blueprint(bp)

            tf = self.urdf_to_carla_transform(spec['xyz'], spec['rpy'])
            actor = self.world.try_spawn_actor(bp, tf, attach_to=self.vehicle)

            if actor is None:
                self.get_logger().warn(f'Failed to spawn sensor {sensor_name}')
                continue

            self.sensor_actors[sensor_name] = actor
            self.actors.append(actor)

            sensor_type = spec['type']
            if sensor_type == 'sensor.camera.rgb':
                actor.listen(
                    lambda image, name=sensor_name: self.sensor_callback(image, name)
                )
            elif sensor_type == 'sensor.lidar.ray_cast':
                actor.listen(
                    lambda lidar, name=sensor_name: self.sensor_callback(lidar, name)
                )

            self.publish_status(
                f'Spawned sensor {sensor_name} type={sensor_type} '
                f'xyz={spec["xyz"]} rpy={spec["rpy"]}'
            )


    def draw_pose_arrow(self, tf, color, label="", life_time=0.2, length=1.5):
        yaw = math.radians(tf.rotation.yaw)

        start = tf.location + carla.Location(z=0.3)

        end = carla.Location(
            x=start.x + length * math.cos(yaw),
            y=start.y + length * math.sin(yaw),
            z=start.z
        )

        self.world.debug.draw_arrow(
            start, end,
            thickness=0.08,
            arrow_size=0.2,
            color=color,
            life_time=life_time
        )

        if label:
            self.world.debug.draw_string(
                start + carla.Location(z=0.4),
                label,
                draw_shadow=False,
                color=color,
                life_time=life_time
            )


    def debug_draw_loop(self):
        if self.world is None:
            return

        if self.vehicle is not None:
            try:
                tf = self.vehicle.get_transform()
                self.draw_pose_arrow(
                    tf,
                    color=carla.Color(0, 255, 0),
                    label="EGO",
                    life_time=0.2,
                    length=2.0
                )
            except Exception:
                pass

        colors = {
            "front_left_rgb": carla.Color(0, 255, 0),
            "front_right_rgb": carla.Color(0, 200, 0),
            "front_lidar": carla.Color(255, 255, 255),
            "rear_left_lidar": carla.Color(0, 255, 255),
            "rear_right_lidar": carla.Color(255, 0, 255),
        }

        for name, actor in self.sensor_actors.items():
            try:
                tf = actor.get_transform()
                color = colors.get(name, carla.Color(255, 255, 0))

                self.draw_pose_arrow(
                    tf,
                    color=color,
                    label=name,
                    life_time=0.2,
                    length=1.5
                )
            except Exception:
                pass





    def sensor_callback(self, data, sensor_name: str):
        if sensor_name not in self.sensor_queues:
            return

        q = self.sensor_queues[sensor_name]
        try:
            if q.full():
                q.get_nowait()
            q.put_nowait(data)
        except Exception:
            pass

    def publish_sensor_data(self):
        for sensor_name, publisher in self.sensor_publishers.items():
            sensor_type = self.sensor_types[sensor_name]
            frame_id = self.sensor_frame_ids[sensor_name]
            q = self.sensor_queues[sensor_name]

            latest = None
            while not q.empty():
                try:
                    latest = q.get_nowait()
                except queue.Empty:
                    break

            if latest is None:
                continue

            if sensor_type == 'sensor.camera.rgb':
                msg = self.carla_image_to_ros_image(latest, frame_id)
                publisher.publish(msg)

            elif sensor_type == 'sensor.lidar.ray_cast':
                msg = self.carla_lidar_to_pointcloud2(latest, frame_id)
                publisher.publish(msg)

    def carla_image_to_ros_image(self, image: carla.Image, frame_id: str) -> Image:
        arr = np.frombuffer(image.raw_data, dtype=np.uint8)
        arr = arr.reshape((image.height, image.width, 4))
        img_bgr = arr[:, :, :3]

        msg = self.bridge.cv2_to_imgmsg(img_bgr, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        return msg

    def carla_lidar_to_pointcloud2(self, lidar_data: carla.LidarMeasurement, frame_id: str) -> PointCloud2:
        points = np.frombuffer(lidar_data.raw_data, dtype=np.float32).reshape(-1, 4)

        msg = PointCloud2()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id

        msg.height = 1
        msg.width = points.shape[0]
        msg.is_bigendian = False
        msg.is_dense = True

        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        msg.point_step = 16
        msg.row_step = msg.point_step * points.shape[0]
        msg.data = points.astype(np.float32).tobytes()
        return msg

    def publish_ego_state(self):
        if self.vehicle is None:
            return

        tf = self.vehicle.get_transform()
        vel = self.vehicle.get_velocity()
        speed = math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)

        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.position.x = float(tf.location.x)
        pose_msg.pose.position.y = float(tf.location.y)
        pose_msg.pose.position.z = float(tf.location.z)

        qx, qy, qz, qw = self.yaw_to_quaternion(tf.rotation.yaw)
        pose_msg.pose.orientation.x = qx
        pose_msg.pose.orientation.y = qy
        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw
        self.pose_pub.publish(pose_msg)

        speed_msg = Float32()
        speed_msg.data = float(speed)
        self.speed_pub.publish(speed_msg)

        yaw_msg = Float32()
        yaw_msg.data = float(tf.rotation.yaw)
        self.yaw_pub.publish(yaw_msg)

        if self.control_applied_pub is not None:
            ctrl = self.vehicle.get_control()
            ctrl_msg = VehicleControl()
            ctrl_msg.throttle = float(ctrl.throttle)
            ctrl_msg.steer = float(ctrl.steer)
            ctrl_msg.brake = float(ctrl.brake)
            ctrl_msg.hand_brake = bool(ctrl.hand_brake)
            ctrl_msg.reverse = bool(ctrl.reverse)
            ctrl_msg.manual_gear_shift = bool(ctrl.manual_gear_shift)
            ctrl_msg.gear = int(ctrl.gear)
            self.control_applied_pub.publish(ctrl_msg)

    def yaw_to_quaternion(self, yaw_deg: float):
        yaw = math.radians(yaw_deg)
        qx = 0.0
        qy = 0.0
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        return qx, qy, qz, qw

    def control_cmd_callback(self, msg : VehicleControl):
        if self.vehicle is None:
            return

        ctrl = carla.VehicleControl()
        ctrl.throttle = float(msg.throttle)
        ctrl.steer = float(msg.steer)
        ctrl.brake = float(msg.brake)
        ctrl.hand_brake = bool(msg.hand_brake)
        ctrl.reverse = bool(msg.reverse)
        ctrl.manual_gear_shift = bool(msg.manual_gear_shift)
        ctrl.gear = int(msg.gear)

        self.vehicle.apply_control(ctrl)

    def destroy_all_actors(self):
        try:
            self.get_logger().info('Destroying actors...')
        except Exception:
            pass

        try:
            if hasattr(self, 'debug_timer') and self.debug_timer is not None:
                self.debug_timer.cancel()
        except Exception:
            pass

        # Stop sensors first
        for actor in reversed(self.actors):
            try:
                actor.stop()
            except Exception:
                pass

        # Then destroy everything
        for actor in reversed(self.actors):
            try:
                actor.destroy()
            except Exception as e:
                try:
                    self.get_logger().warn(f'Failed to destroy actor: {e}')
                except Exception:
                    pass

        self.actors.clear()
        self.sensor_actors.clear()
        self.sensor_publishers.clear()
        self.sensor_queues.clear()
        self.sensor_types.clear()
        self.sensor_frame_ids.clear()
        self.vehicle = None

        try:
            self.get_logger().info('All actors destroyed')
        except Exception:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = CarlaInterfaceNode()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    except Exception as e:
        print(f'[carla_interface_node] Fatal error: {e}')

    finally:
        if node is not None:
            try:
                node.destroy_all_actors()
            except Exception as e:
                print(f'[carla_interface_node] Cleanup error: {e}')

            try:
                node.destroy_node()
            except Exception as e:
                print(f'[carla_interface_node] Node destroy error: {e}')

        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass



if __name__ == '__main__':
    main()