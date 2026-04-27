#!/usr/bin/env python3

import math
import queue
from typing import Dict, Optional
import random
import numpy as np
import carla
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg import String, Float32, Header, UInt32, Empty
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, PointCloud2, PointField, Imu
from cv_bridge import CvBridge


from carla_parking_msgs.msg import VehicleControl

SENSOR_DEBUG_FLAG = False

#VEHICLE CONSTANTS
MAX_THROTTLE = 0.45

# CAMERA PARAMS
IM_WIDTH = 1936
IM_HEIGHT = 1216
DALSA_LENS_FOV = 31 # FOVh​=2⋅arctan(sensor width/2⋅focal length​) 12mm is ≈31.2° // 16mm is ≈23.6°

#CARLA LOCATION TO GEOMETRICAL MIDDLE
VEH_WIDTH = 4 * 0.623
VEH_LENGTH = 6.85 * 0.623
COM_OFFSET = -0.4

# URDF TO CARLA OFFSET
URDF_OFFSET_X = 0.33
URDF_OFFSET_Z = 0.4

# ARCH SPAWN PARAMETERS
ARCH_SPAWN_FLAG = False
CARGOBOX_COORDINATES = (290.9, -201.03)
MIN_RAD= 8
MAX_RAD= 15
MIN_ANG= -15
MAX_ANG= 15
MAX_YAW = 10

SENSOR_LAYOUT = {

    "leopard1": {
        "type": "sensor.camera.rgb",
        "xyz": (-0.655, -0.177, 0.350),
        "rpy": (0.0, 0.0, 3.142),
        "topic": "/sensors/leopard1_rgb/image_raw",
        "frame_id": "leopard1_rgb_frame",
    },

    "leopard2": {
        "type": "sensor.camera.rgb",
        "xyz": (0.597, -1.152, 0.380),
        "rpy": (0.0, 0.0, 0.0),
        "topic": "/sensors/leopard2_rgb/image_raw",
        "frame_id": "leopard2_rgb_frame",
    },

    "leopard3": {
        "type": "sensor.camera.rgb",
        "xyz": (0.597, 1.152, 0.380),
        "rpy": (0.0, 0.0, 0.0),
        "topic": "/sensors/leopard3_rgb/image_raw",
        "frame_id": "leopard3_rgb_frame",
    },

    "leopard4": {
        "type": "sensor.camera.rgb",
        "xyz": (3.198, -0.822, 0.442),
        "rpy": (0.0, 0.0, 0.0),
        "topic": "/sensors/leopard4_rgb/image_raw",
        "frame_id": "leopard4_rgb_frame",
    },

    "leopard5": {
        "type": "sensor.camera.rgb",
        "xyz": (3.198, 0.822, 0.442),
        "rpy": (0.0, 0.0, 0.0),
        "topic": "/sensors/leopard5_rgb/image_raw",
        "frame_id": "leopard5_rgb_frame",
    },

    "leopard6": {
        "type": "sensor.camera.rgb",
        "xyz": (0.545, 0.668, 0.466),
        "rpy": (0.0, 0.0, 0.0),
        "topic": "/sensors/leopard6_rgb/image_raw",
        "frame_id": "leopard6_rgb_frame",
    },

    "leopard7": {
        "type": "sensor.camera.rgb",
        "xyz": (0.545, -0.668, 0.466),
        "rpy": (0.0, 0.0, 0.0),
        "topic": "/sensors/leopard7_rgb/image_raw",
        "frame_id": "leopard7_rgb_frame",
    },

    "leopard8": {
        "type": "sensor.camera.rgb",
        "xyz": (-0.590, -1.078, 0.458),
        "rpy": (0.0, 0.0, -2.133),
        "topic": "/sensors/leopard8_rgb/image_raw",
        "frame_id": "leopard8_rgb_frame",
    },

    "leopard9": {
        "type": "sensor.camera.rgb",
        "xyz": (-0.590, 1.078, 0.458),
        "rpy": (0.0, 0.0, 2.133),
        "topic": "/sensors/leopard9_rgb/image_raw",
        "frame_id": "leopard9_rgb_frame",
    },

    "dalsa1": {
        "type": "sensor.camera.rgb",
        "xyz": (-0.444, -1.216, 0.532),
        "rpy": (0.0, 0.0, -1.571),
        "topic": "/sensors/dalsa1_rgb/image_raw",
        "frame_id": "dalsa1_rgb_frame",
    },

    "dalsa3": {
        "type": "sensor.camera.rgb",
        "xyz": (-0.444, 1.216, 0.532),
        "rpy": (0.0, 0.0, 1.571),
        "topic": "/sensors/dalsa3_rgb/image_raw",
        "frame_id": "dalsa3_rgb_frame",
    },

    "dalsa4": {
        "type": "sensor.camera.rgb",
        "xyz": (-0.677, 0.122, 0.311),
        "rpy": (0.0, 0.0, 3.142),
        "topic": "/sensors/dalsa4_rgb/image_raw",
        "frame_id": "dalsa4_rgb_frame",
    },

    # LIDAR
    "ibeo1": {
        "type": "sensor.lidar.ray_cast",
        "xyz": (-0.611, 0.0, 0.344),
        "rpy": (3.1416, 0.0, 3.1416),
        "topic": "/sensors/ibeo1/points",
        "frame_id": "ibeo1_frame",
    },

    "ibeo2": {
        "type": "sensor.lidar.ray_cast",
        "xyz": (-0.671, 0.839, 0.402),
        "rpy": (0.021, 0.019, -2.125),
        "topic": "/sensors/ibeo2/points",
        "frame_id": "ibeo2_frame",
    },

    "ibeo3": {
        "type": "sensor.lidar.ray_cast",
        "xyz": (-0.472, 1.14, 0.484),
        "rpy": (0.007, 0.079, 1.411),
        "topic": "/sensors/ibeo3/points",
        "frame_id": "ibeo3_frame",
    },

    "ibeo4": {
        "type": "sensor.lidar.ray_cast",
        "xyz": (-0.328, 1.132, 0.59),
        "rpy": (0.0, 0.096, 0.5218),
        "topic": "/sensors/ibeo4/points",
        "frame_id": "ibeo4_frame",
    },

    "ibeo5": {
        "type": "sensor.lidar.ray_cast",
        "xyz": (5.0, 0.0, 2.59),
        "rpy": (3.133, 0.665, 0.005),
        "topic": "/sensors/ibeo5/points",
        "frame_id": "ibeo5_frame",
    },

    "ibeo6": {
        "type": "sensor.lidar.ray_cast",
        "xyz": (0.486, 0.0, 0.51),
        "rpy": (0.007, -0.05, 0.00735),
        "topic": "/sensors/ibeo6/points",
        "frame_id": "ibeo6_frame",
    },

    "ibeo7": {
        "type": "sensor.lidar.ray_cast",
        "xyz": (0.0, 0.0, 0.0),
        "rpy": (0.0, 0.0, 0.0),
        "topic": "/sensors/ibeo7/points",
        "frame_id": "ibeo7_frame",
        # weird
    },

    "ibeo8": {
        "type": "sensor.lidar.ray_cast",
        "xyz": (-0.328, -1.132, 0.59),
        "rpy": (0.035, 0.102, -0.529),
        "topic": "/sensors/ibeo8/points",
        "frame_id": "ibeo8_frame",
    },

    "ibeo9": {
        "type": "sensor.lidar.ray_cast",
        "xyz": (-0.472, -1.14, 0.484),
        "rpy": (0.026, 0.1025, -1.445),
        "topic": "/sensors/ibeo9/points",
        "frame_id": "ibeo9_frame",
    },

    "ibeo10": {
        "type": "sensor.lidar.ray_cast",
        "xyz": (-0.671, -0.839, 0.402),
        "rpy": (0.0, -0.007, 2.101),
        "topic": "/sensors/ibeo10/points",
        "frame_id": "ibeo10_frame",
    },


    # IDEAL SENSORS
    "seyond6": {
        "type": "sensor.lidar.ray_cast",
        "xyz": (0.6, -0.20, 0.55),
        "rpy": (0.0, 0.0, 0.0),
        "topic": "/sensors/seyond6/points",
        "frame_id": "seyond6_frame",
    },


    "dalsa2": { 
        "type": "sensor.camera.rgb",
        "xyz": (0.05, 0.02, 0.78),
        "rpy": (0.0, 0.0, 0.0),
        "topic": "/sensors/dalsa2_rgb/image_raw",
        "frame_id": "dalsa2_rgb_frame",
    },



    # LASERS
    "laser1": {
        "type": "sensor.lidar.ray_cast",
        "xyz": (0.517, 0.369, 0.382),
        "rpy": (1.571, 0.0, 0.0),
        "topic": "/sensors/laser1/points",
        "frame_id": "laser1_frame",
    },
    "laser2": {
        "type": "sensor.lidar.ray_cast",
        "xyz": (3.079, 0.731, 0.234),
        "rpy": (-1.571, 0.0, -1.571),
        "topic": "/sensors/laser2/points",
        "frame_id": "laser2_frame",
    },
    "laser3": {
        "type": "sensor.lidar.ray_cast",
        "xyz": (3.079, -0.731, 0.234),
        "rpy": (1.571, 0.0, 1.571),
        "topic": "/sensors/laser3/points",
        "frame_id": "laser3_frame",
    },
    "laser4": {
        "type": "sensor.lidar.ray_cast",
        "xyz": (0.517, -0.369, 0.382),
        "rpy": (-1.571, 0.0, 0.0),
        "topic": "/sensors/laser4/points",
        "frame_id": "laser4_frame",
    },
    "imu": {
        "type": "sensor.other.imu",
        "xyz": (0.41, 0.0, 0.667),
        "rpy": (0.0, 0.0, 3.1416),
        "topic": "/sensors/imu/data",
        "frame_id": "imu_frame",
    },


}

class CarlaInterfaceNode(Node):
    def __init__(self):
        super().__init__('carla_interface_node')

        self.declare_parameter('draw_sensor_debug', SENSOR_DEBUG_FLAG)
        self.declare_parameter('arch_spawn_flag', ARCH_SPAWN_FLAG)

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
        self.laser_distance_publishers = {}

        self.status_pub = self.create_publisher(String, '/carla_interface/status', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/ego/pose', 10)
        self.speed_pub = self.create_publisher(Float32, '/ego/speed', 10)
        self.yaw_pub = self.create_publisher(Float32, '/ego/yaw', 10)
        self.episode_reset_pub = self.create_publisher(UInt32, '/episode/reset', 10)
        
        self.episode_id = 0

        
        self.control_applied_pub = self.create_publisher(
            VehicleControl, '/vehicle/control_applied', 10
        )
        self.control_sub = self.create_subscription(
            VehicleControl,
            '/vehicle/control_cmd',
            self.control_cmd_callback,
            10
        )
        self.reset_request_sub = self.create_subscription(
            Empty,
            '/episode/reset_request',
            self.reset_request_callback,
            10
        )

        self.declare_parameter('host', 'localhost')
        self.declare_parameter('port', 2000)
        self.declare_parameter('timeout', 10.0)
        self.declare_parameter('vehicle_blueprint', 'vehicle.fk.ushift')

        self.declare_parameter('spawn_x', 280.45)
        self.declare_parameter('spawn_y', -201.2)
        # self.declare_parameter('spawn_z', 0.8)
        self.declare_parameter('spawn_z', 0.3)
        self.declare_parameter('spawn_yaw', 180.0)

        self.declare_parameter(
            'chosen_sensors',
            [
                'seyond6',
                'leopard4',
                'leopard5',
                'dalsa2',
                'laser1',
                'laser2',
                'laser3',
                'laser4',
                'imu',
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

        # ego_rate = float(self.get_parameter('ego_state_publish_rate').value)
        # sensor_rate = float(self.get_parameter('sensor_publish_rate').value)

        # self.ego_timer = self.create_timer(1.0 / ego_rate, self.publish_ego_state)
        # self.sensor_timer = self.create_timer(1.0 / sensor_rate, self.publish_sensor_data)
        self.sim_timer = self.create_timer(0.001, self.step_simulation) # Maybe change time
        self.goal_reach_timer = self.create_timer(3, self.check_goal_and_reset)

        self.publish_status('carla_interface_node  init')

        # self.get_logger().info(f'Chosen sensor {list(self.get_parameter("chosen_sensors").value)}')

        for sensor_name in self.sensor_publishers.keys():
            spec = SENSOR_LAYOUT[sensor_name]
            self.get_logger().info(
                f'{sensor_name}: type={spec["type"]}, topic={spec["topic"]}, frame={spec["frame_id"]}'
            )

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

        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        settings.substepping = True
        settings.max_substep_delta_time = 0.01
        settings.max_substeps = 10
        self.world.apply_settings(settings)

        self.publish_status(f'connected at {host}:{port}')

    def spawn_ego_vehicle(self):
        blueprint_id = self.get_parameter('vehicle_blueprint').value
        bp = self.bp_lib.find(blueprint_id)

        if bool(self.get_parameter('arch_spawn_flag').value):
            x,y, yaw = self.random_spawn_point(CARGOBOX_COORDINATES, MIN_RAD, MAX_RAD,MIN_ANG, MAX_ANG, MAX_YAW)
            spawn_tf = carla.Transform(
                carla.Location(
                    x=float(x),
                    y=float(y),
                    z=float(self.get_parameter('spawn_z').value),
                ),
                carla.Rotation(
                    yaw=float(yaw)
                )
            )
        else:
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
        time.sleep(2)

        if self.vehicle is None:
            raise RuntimeError('Failed to spawn ego vehicle')

        self.actors.append(self.vehicle)
        self.publish_status(f'Spawned ego vehicle id={self.vehicle.id} type={blueprint_id}')
        self.get_logger().info(
            f'vehicle transform: x={spawn_tf.location.x:.2f}, '
            f'y={spawn_tf.location.y:.2f}, z={spawn_tf.location.z:.2f}, '
            f'yaw={spawn_tf.rotation.yaw:.2f}'
        )

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
            self.sensor_queues[sensor_name] = queue.Queue(maxsize=50)

            if sensor_type == 'sensor.camera.rgb':
                pub = self.create_publisher(Image, topic, qos_profile_sensor_data)
                self.sensor_publishers[sensor_name] = pub
                self.publish_status(f'Created camera publisher for {sensor_name} -> {topic}')
            elif sensor_type == 'sensor.lidar.ray_cast':
                pub = self.create_publisher(PointCloud2, topic, qos_profile_sensor_data)
                self.sensor_publishers[sensor_name] = pub
                self.publish_status(f'Created lidar publisher for {sensor_name} -> {topic}')

                if 'laser' in sensor_name.lower():
                    dist_topic = f'/sensors/{sensor_name}/distance'
                    dist_pub = self.create_publisher(Float32, dist_topic, qos_profile_sensor_data)
                    self.laser_distance_publishers[sensor_name] = dist_pub
                    self.publish_status(f'Created laser distance publisher for {sensor_name} -> {dist_topic}')
          
            elif sensor_type == 'sensor.other.imu':
                pub = self.create_publisher(Imu, topic, qos_profile_sensor_data)
                self.sensor_publishers[sensor_name] = pub
                self.publish_status(f'Created IMU publisher for {sensor_name} -> {topic}')
            else:
                self.get_logger().warn(
                    f'Unsupported sensor type {sensor_type} for sensor {sensor_name}'
                )

    def urdf_to_carla_transform(self, xyz, rpy, body_x_offset=COM_OFFSET):
        x_u, y_u, z_u = xyz
        roll_u, pitch_u, yaw_u = rpy

        x_c = -(x_u + body_x_offset) + URDF_OFFSET_X
        y_c = y_u
        z_c = z_u + URDF_OFFSET_Z

        roll_c = math.degrees(roll_u)
        pitch_c = math.degrees(pitch_u)
        yaw_c = -math.degrees(yaw_u) + 180.0

        return carla.Transform(
            carla.Location(x=x_c, y=y_c, z=z_c),
            carla.Rotation(roll=roll_c, pitch=pitch_c, yaw=yaw_c),
        )
    
    def reset_request_callback(self, msg):
        self.get_logger().info('Received reset request')
        self.reset_episode()

    def step_simulation(self):
        if self.world is None or self.vehicle is None:
            return

        try:
            self.world.tick()
            self.publish_ego_state()
            self.publish_sensor_data()   # old version


            
        except Exception as e:
            self.get_logger().warn(f'step_simulation failed: {e}')

    def random_spawn_point(self, center, min_radius, max_radius,
                            start_angle=-15, end_angle= 15, max_yaw_deg = 15):
        cx, cy = center

        u = random.random()
        r = math.sqrt(u * (max_radius**2 - min_radius**2) + min_radius**2)

        theta = math.radians(random.uniform(start_angle, end_angle))+ math.pi

        x = cx + r * math.cos(theta)
        y = cy + r * math.sin(theta)

        if y < cy:
            yaw = random.uniform(0.0, max_yaw_deg)
        else:
            yaw = random.uniform(-max_yaw_deg, 0.0)
        yaw += 180.0
        yaw = (yaw + 180) % 360 - 180
        if bool(self.get_parameter('draw_sensor_debug').value):
            self.draw_spawn_box(center, min_radius, max_radius,
                        start_angle, end_angle)
        return x, y, yaw
    
    def draw_spawn_box(self, center, min_radius, max_radius,
                   start_angle=-15, end_angle=15,
                   z=0.5, life_time=50.0):

        cx, cy = center

        a0 = math.radians(start_angle) + math.pi
        a1 = math.radians(end_angle) + math.pi

        points = [
            (cx + min_radius * math.cos(a0), cy + min_radius * math.sin(a0)),
            (cx + min_radius * math.cos(a1), cy + min_radius * math.sin(a1)),
            (cx + max_radius * math.cos(a0), cy + max_radius * math.sin(a0)),
            (cx + max_radius * math.cos(a1), cy + max_radius * math.sin(a1)),
        ]

        for (x, y) in points:
            self.world.debug.draw_point(
                carla.Location(x=float(x), y=float(y), z=float(z)),
                size=0.12,
                color=carla.Color(255, 0, 255),
                life_time=life_time
            )

    def configure_sensor_blueprint(self, bp, sensor_name = None):
        if bp.id == 'sensor.camera.rgb':
            img_x = IM_WIDTH
            img_y = IM_HEIGHT
            fov = 90.0
            sensor_tick = 0.05
            if sensor_name is not None:
                name= sensor_name.lower()
                if 'leopard' in name:
                    img_x = 2880
                    img_y = 1860
                    fov = 120.0
                    sensor_tick = 0.05
                elif 'dalsa' in name:
                    img_x = 1936
                    img_y = 1216
                    fov = DALSA_LENS_FOV
                    sensor_tick = 0.05
                        
            bp.set_attribute('image_size_x', str(img_x))
            bp.set_attribute('image_size_y', str(img_y))
            bp.set_attribute('fov', str(fov))
            bp.set_attribute('sensor_tick', str(sensor_tick))
            
        elif bp.id == 'sensor.lidar.ray_cast':
            if sensor_name is not None:
                name= sensor_name.lower()
                if 'laser' in name: #0.2 and 23 0.5  // 1115
                    bp.set_attribute('channels', '1')
                    bp.set_attribute('points_per_second', '500')
                    bp.set_attribute('horizontal_fov', '1')
                    bp.set_attribute('upper_fov', '0')
                    bp.set_attribute('lower_fov', '0')
                    bp.set_attribute('range', '12')
                    bp.set_attribute('rotation_frequency', '10')
                    bp.set_attribute('sensor_tick', '0.1')
                    bp.set_attribute('dropoff_general_rate', '0.0')
                    bp.set_attribute('dropoff_intensity_limit', '1.0')
                    bp.set_attribute('dropoff_zero_intensity', '0.0')
                    bp.set_attribute('noise_stddev', '0.0')
                    if name in ['laser1', 'laser4']:
                        bp.set_attribute('sensor_tick', '0.2')
                    elif name in ['laser2', 'laser3']:
                        bp.set_attribute('sensor_tick', '0.5')
                    else:
                        bp.set_attribute('sensor_tick', '0.5')

                else:
            
                    bp.set_attribute('channels', '150')
                    bp.set_attribute('range', '250')              
                    bp.set_attribute('points_per_second', '1000000')
                    bp.set_attribute('rotation_frequency', '10')
                    bp.set_attribute('horizontal_fov', '120')
                    bp.set_attribute('upper_fov', '12.5')
                    bp.set_attribute('lower_fov', '-12.5')
                    bp.set_attribute('sensor_tick', '0.1')

                    bp.set_attribute('dropoff_general_rate', '0.0')
                    bp.set_attribute('dropoff_intensity_limit', '1.0')
                    bp.set_attribute('dropoff_zero_intensity', '0.0')

                    bp.set_attribute('noise_stddev', '0.01')
                    bp.set_attribute('atmosphere_attenuation_rate', '0.002')


        elif bp.id == 'sensor.other.imu':
            bp.set_attribute('sensor_tick', '0.01') 
            bp.set_attribute('noise_accel_stddev_x', '0.0055')
            bp.set_attribute('noise_accel_stddev_y', '0.0055')
            bp.set_attribute('noise_accel_stddev_z', '0.0055')

            bp.set_attribute('noise_gyro_bias_x', '0.001745')
            bp.set_attribute('noise_gyro_bias_y', '0.001745')
            bp.set_attribute('noise_gyro_bias_z', '0.001745')

            bp.set_attribute('noise_gyro_stddev_x', '0.00016')
            bp.set_attribute('noise_gyro_stddev_y', '0.00016')
            bp.set_attribute('noise_gyro_stddev_z', '0.00016')

            bp.set_attribute('noise_seed', '1')

    def spawn_sensors(self):
        chosen_sensors = list(self.get_parameter('chosen_sensors').value)

        for sensor_name in chosen_sensors:
            if sensor_name not in SENSOR_LAYOUT:
                continue

            spec = SENSOR_LAYOUT[sensor_name]
            bp = self.bp_lib.find(spec['type'])
            self.configure_sensor_blueprint(bp, sensor_name)

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
            elif sensor_type == 'sensor.other.imu':
                actor.listen(lambda imu, name=sensor_name: self.sensor_callback(imu, name)
                )

            self.publish_status(
                f'Spawned sensor {sensor_name} type={sensor_type} '
                f'xyz={spec["xyz"]} rpy={spec["rpy"]}'
            )

    def carla_imu_to_ros_imu(self, imu_data, frame_id: str) -> Imu:
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id

        msg.linear_acceleration.x = float(imu_data.accelerometer.x)
        msg.linear_acceleration.y = float(imu_data.accelerometer.y)
        msg.linear_acceleration.z = float(imu_data.accelerometer.z)

        msg.angular_velocity.x = float(imu_data.gyroscope.x)
        msg.angular_velocity.y = float(imu_data.gyroscope.y)
        msg.angular_velocity.z = float(imu_data.gyroscope.z)

        tf = self.vehicle.get_transform()

        roll = math.radians(tf.rotation.roll)
        pitch = math.radians(tf.rotation.pitch)
        yaw = math.radians(tf.rotation.yaw)

        qx, qy, qz, qw = self.euler_to_quaternion(roll, pitch, yaw)

        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw

        msg.orientation_covariance[0] = 1e-3
        msg.orientation_covariance[4] = 1e-3
        msg.orientation_covariance[8] = 1e-3

        msg.angular_velocity_covariance[0] = 1e-6
        msg.angular_velocity_covariance[4] = 1e-6
        msg.angular_velocity_covariance[8] = 1e-6

        msg.linear_acceleration_covariance[0] = 1e-4
        msg.linear_acceleration_covariance[4] = 1e-4
        msg.linear_acceleration_covariance[8] = 1e-4

        return msg
    
    def euler_to_quaternion(self, roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy

        return qx, qy, qz, qw

    def draw_pose_arrow(self, tf, color, label="", life_time=0.4, length=0.5):
        yaw = math.radians(tf.rotation.yaw)

        start = tf.location

        end = carla.Location(
            x=start.x + length * math.cos(yaw),
            y=start.y + length * math.sin(yaw),
            z=start.z
        )

        self.world.debug.draw_arrow(
            start, end,
            thickness=0.04,
            arrow_size=0.05,
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

    def get_ego_center(self, offset=COM_OFFSET):
        if self.vehicle is None:
            return None

        try:
            tf = self.vehicle.get_transform()
            loc = tf.location
            yaw = math.radians(tf.rotation.yaw)

            # Fwd vector
            fx = math.cos(yaw)
            fy = math.sin(yaw)

            cx = loc.x + offset * fx
            cy = loc.y + offset * fy
            cz = loc.z

            return carla.Location(x=float(cx), y=float(cy), z=float(cz))

        except Exception as e:
            self.get_logger().warn(f"get_ego_center failed: {e}")
            return None

    def get_rear_position(self):
        return self.get_ego_center(offset=-VEH_LENGTH / 2)
    def get_front_position(self):
        return self.get_ego_center(offset=+VEH_LENGTH / 2)
        
    def draw_boundary_box(self, tf, life_time=0.1, length=VEH_LENGTH, width=VEH_WIDTH, offset=COM_OFFSET):
        loc = tf.location
        yaw = np.radians(tf.rotation.yaw)

        # Forward and right vectors
        forward = np.array([np.cos(yaw), np.sin(yaw)])
        right = np.array([-np.sin(yaw), np.cos(yaw)])

 
        center = np.array([
            loc.x + offset * forward[0],
            loc.y + offset * forward[1]
        ])

        half_l = length / 2.0
        half_w = width / 2.0

        corners = [
            center + forward * half_l - right * half_w,  # front-left
            center + forward * half_l + right * half_w,  # front-right
            center - forward * half_l + right * half_w,  # rear-right
            center - forward * half_l - right * half_w,  # rear-left
        ]

        corners_carla = [
            carla.Location(x=float(p[0]), y=float(p[1]), z=loc.z + 0.2)
            for p in corners
        ]

        # Draw lines
        for i in range(4):
            self.world.debug.draw_line(
                corners_carla[i],
                corners_carla[(i + 1) % 4],
                thickness=0.12,
                color=carla.Color(0, 255, 0),
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
                    color=carla.Color(255, 255, 255),
                    label="EGO",
                    life_time=0.5,
                    length=1.0
                )
                self.draw_boundary_box(tf, life_time=0.5)
            except Exception:
                pass


        for name, actor in self.sensor_actors.items():
            try:
                tf = actor.get_transform()
                sensor_type = self.sensor_types.get(name, "")

                if sensor_type == "sensor.camera.rgb":
                    color = carla.Color(0, 255, 0)      # green for RGB cameras
                elif "lidar" in sensor_type:
                    color = carla.Color(255, 0, 0)      # red for lidars
                else:
                    color = carla.Color(255, 255, 0)    # yellow fallback

                self.draw_pose_arrow(
                    tf,
                    color=color,
                    label=name,
                    life_time=0.5,
                    length=0.5
                )
            except Exception:
                pass

    def sensor_callback(self, data, sensor_name: str):
        if sensor_name not in self.sensor_queues:
            return

        q = self.sensor_queues[sensor_name]

        try:
            q.put_nowait(data)
        except queue.Full:
            try:
                q.get_nowait()
                q.put_nowait(data)
            except Exception:
                pass

    def publish_sensor_data(self): # not used temp
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

                if sensor_name in self.laser_distance_publishers:
                    dist = self.lidar_to_distance(latest)
                    dist_msg = Float32()
                    dist_msg.data = float(dist)
                    self.laser_distance_publishers[sensor_name].publish(dist_msg)

            elif sensor_type == 'sensor.other.imu':
                msg = self.carla_imu_to_ros_imu(latest, frame_id)
                publisher.publish(msg)

    def publish_sensor_data_for_frame(self, frame: int):
        for sensor_name, publisher in self.sensor_publishers.items():
            sensor_type = self.sensor_types[sensor_name]
            frame_id = self.sensor_frame_ids[sensor_name]

            data = self.get_sensor_data_for_frame(sensor_name, frame, timeout=1.0)
            if data is None:
                self.get_logger().warn(f'No data for {sensor_name} at frame {frame}')
                continue

            if sensor_type == 'sensor.camera.rgb':
                msg = self.carla_image_to_ros_image(data, frame_id)
                publisher.publish(msg)

            elif sensor_type == 'sensor.lidar.ray_cast':
                msg = self.carla_lidar_to_pointcloud2(data, frame_id)
                publisher.publish(msg)

                if sensor_name in self.laser_distance_publishers:
                    dist = self.lidar_to_distance(data)
                    dist_msg = Float32()
                    dist_msg.data = float(dist)
                    self.laser_distance_publishers[sensor_name].publish(dist_msg)

            elif sensor_type == 'sensor.other.imu':
                msg = self.carla_imu_to_ros_imu(data, frame_id)
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
    
    def lidar_to_distance(self, lidar_data):
        points = np.frombuffer(lidar_data.raw_data, dtype=np.float32).reshape(-1, 4)

        if len(points) == 0:
            return 12.0  # range max for the lasers

        distances = np.linalg.norm(points[:, :3], axis=1)
        return float(np.min(distances))

    def publish_ego_state(self):
        if self.vehicle is None:
            return

        tf = self.vehicle.get_transform()
        vel = self.vehicle.get_velocity()
        yaw_rad = math.radians(tf.rotation.yaw)

        forward_x = math.cos(yaw_rad)
        forward_y = math.sin(yaw_rad)

        speed = vel.x * forward_x + vel.y * forward_y

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

    def get_sensor_data_for_frame(self, sensor_name: str, target_frame: int, timeout=1.0):
        q = self.sensor_queues[sensor_name]
        deadline = time.time() + timeout

        while time.time() < deadline:
            remaining = max(0.0, deadline - time.time())
            try:
                data = q.get(timeout=remaining)
            except queue.Empty:
                return None

            if data.frame == target_frame:
                return data

            # discard older frames
            if data.frame < target_frame:
                continue

            # if somehow we got a future frame, return it or warn
            if data.frame > target_frame:
                self.get_logger().warn(
                    f'{sensor_name}: got future frame {data.frame}, expected {target_frame}'
                )
                return data

        return None
    
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
        ctrl.throttle = min(float(msg.throttle), MAX_THROTTLE)
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

        for actor in reversed(self.actors):
            try:
                actor.stop()
            except Exception:
                pass

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
        self.laser_distance_publishers.clear()
        self.vehicle = None

        try:
            self.get_logger().info('All actors destroyed')
        except Exception:
            pass

    def check_goal_and_reset(self):
        if self.vehicle is None:
            return

        tf = self.vehicle.get_transform()


        goal_x = CARGOBOX_COORDINATES[0]
        goal_y = CARGOBOX_COORDINATES[1]
        goal_yaw = 180.0

        pos_tol = 1.5
        yaw_tol = 5.0
        speed_tol = 0.3

        vel = self.vehicle.get_velocity()
        speed = math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)

        dx = tf.location.x - goal_x
        dy = tf.location.y - goal_y
        pos_err = math.sqrt(dx * dx + dy * dy)

        yaw_err = abs((tf.rotation.yaw - goal_yaw + 180.0) % 360.0 - 180.0)

        reached = (
            pos_err < pos_tol and
            yaw_err < yaw_tol and
            speed < speed_tol
        )

        if reached and not self.goal_reached:
            self.goal_reached = True
            self.reset_episode()

        elif not reached:
            self.goal_reached = False
            
    def reset_episode(self):
        if self.vehicle is None:
            return

        try:
            self.vehicle.apply_control(carla.VehicleControl(
                throttle=0.0,
                steer=0.0,
                brake=1.0,
                hand_brake=True
            ))

            time.sleep(1.5)

            self.vehicle.set_simulate_physics(False)

            x, y, yaw = self.random_spawn_point(
                CARGOBOX_COORDINATES,
                MIN_RAD,
                MAX_RAD,
                MIN_ANG,
                MAX_ANG,
                MAX_YAW
            )

            new_tf = carla.Transform(
                carla.Location(
                    x=float(x),
                    y=float(y),
                    z=float(self.get_parameter('spawn_z').value),
                ),
                carla.Rotation(yaw=float(yaw))
            )

            self.vehicle.set_transform(new_tf)

            self.vehicle.set_target_velocity(carla.Vector3D(0.0, 0.0, 0.0))
            self.vehicle.set_target_angular_velocity(carla.Vector3D(0.0, 0.0, 0.0))

            time.sleep(1.5)
            self.vehicle.set_simulate_physics(True)

            self.vehicle.apply_control(carla.VehicleControl(
                throttle=0.0,
                steer=0.0,
                brake=0.0,
                hand_brake=False
            ))

            self.episode_id += 1

            msg = UInt32()
            msg.data = self.episode_id
            self.episode_reset_pub.publish(msg)

            self.publish_status(
                f'Episode reset -> episode_id={self.episode_id}, '
                f'x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}'
            )

        except Exception as e:
            self.get_logger().error(f'reset_episode failed: {e}')

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