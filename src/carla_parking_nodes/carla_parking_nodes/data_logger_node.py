#!/usr/bin/env python3

import csv
import json
from pathlib import Path
from datetime import datetime

import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32, UInt32
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, PointCloud2, Imu
from cv_bridge import CvBridge
from sensor_msgs_py import point_cloud2
from rclpy.qos import qos_profile_sensor_data

from carla_parking_msgs.msg import VehicleControl

DATA_PATH = '/data/carla_datasets'
IMG_RESIZE = (224, 224)

# Goal pose in world frame: (x, y, yaw_degrees)
CARGOBOX_COORDINATES = (290.9, -201.03, 180.0)


class DataLoggerNode(Node):
    def __init__(self):
        super().__init__('data_logger_node')

        self.declare_parameter('dataset_root', DATA_PATH)
        self.declare_parameter('run_name', '')
        self.declare_parameter('image_format', 'png')
        self.declare_parameter('log_rate_hz', 10.0)

        dataset_root = Path(self.get_parameter('dataset_root').value)
        run_name = self.get_parameter('run_name').value
        self.image_format = self.get_parameter('image_format').value
        self.log_rate_hz = float(self.get_parameter('log_rate_hz').value)

        if run_name == '':
            timestamp = datetime.now().strftime('%m_%d_%H_%M')
            run_name = f'dataset_run_{timestamp}'

        self.run_dir = dataset_root / run_name
        self.metadata_dir = self.run_dir / 'metadata'
        self.records_dir = self.run_dir / 'records'
        self.sensors_dir = self.run_dir / 'sensors'

        self.dalsa2_dir = self.sensors_dir / 'dalsa2_rgb'
        self.leopard4_dir = self.sensors_dir / 'leopard4_rgb'
        self.leopard5_dir = self.sensors_dir / 'leopard5_rgb'
        self.seyond6_dir = self.sensors_dir / 'seyond6_lidar'

        self._make_dirs()

        self.bridge = CvBridge()
        self.sample_id = 0
        self.episode_id = 0

        # Camera / lidar
        self.latest_dalsa2 = None
        self.latest_leopard4 = None
        self.latest_leopard5 = None
        self.latest_seyond6_xyz = None

        # Lasers
        self.latest_laser1 = None
        self.latest_laser2 = None
        self.latest_laser3 = None
        self.latest_laser4 = None

        # IMU
        self.latest_imu = None

        # Control cmd
        self.latest_cmd_throttle = None
        self.latest_cmd_steer = None
        self.latest_cmd_brake = None
        self.latest_cmd_reverse = None
        self.latest_cmd_gear = None

        # Control applied
        self.latest_applied_throttle = None
        self.latest_applied_steer = None
        self.latest_applied_brake = None
        self.latest_applied_reverse = None
        self.latest_applied_gear = None

        # Ego state
        self.latest_speed = None
        self.latest_yaw = None  

        self.latest_pose_x = None
        self.latest_pose_y = None
        self.latest_pose_z = None
        self.latest_ori_x = None
        self.latest_ori_y = None
        self.latest_ori_z = None
        self.latest_ori_w = None

        # Goal-relative errors
        self.yaw_error = None
        self.position_error_x = None
        self.position_error_y = None
        self.goal_distance = None

        self.csv_path = self.records_dir / 'samples.csv'
        self._init_csv()
        self._write_metadata()

        self.create_subscription(Image, '/sensors/dalsa2_rgb/image_raw', self.dalsa2_cb, qos_profile_sensor_data)
        self.create_subscription(Image, '/sensors/leopard4_rgb/image_raw', self.leopard4_cb, qos_profile_sensor_data)
        self.create_subscription(Image, '/sensors/leopard5_rgb/image_raw', self.leopard5_cb, qos_profile_sensor_data)

        self.create_subscription(PointCloud2, '/sensors/seyond6/points', self.seyond6_cb, qos_profile_sensor_data)

        self.create_subscription(Float32, '/sensors/laser1/distance', self.laser1_cb, qos_profile_sensor_data)
        self.create_subscription(Float32, '/sensors/laser2/distance', self.laser2_cb, qos_profile_sensor_data)
        self.create_subscription(Float32, '/sensors/laser3/distance', self.laser3_cb, qos_profile_sensor_data)
        self.create_subscription(Float32, '/sensors/laser4/distance', self.laser4_cb, qos_profile_sensor_data)

        self.create_subscription(Imu, '/sensors/imu/data', self.imu_cb, qos_profile_sensor_data)

        self.create_subscription(VehicleControl, '/vehicle/control_cmd', self.control_cmd_cb, 10)
        self.create_subscription(VehicleControl, '/vehicle/control_applied', self.control_applied_cb, 10)

        self.create_subscription(Float32, '/ego/speed', self.speed_cb, 10)
        self.create_subscription(Float32, '/ego/yaw', self.yaw_cb, 10)
        self.create_subscription(PoseStamped, '/ego/pose', self.pose_cb, 10)
        self.create_subscription(UInt32, '/episode/reset', self.episode_reset_cb, 10)

        period = 1.0 / self.log_rate_hz
        self.timer = self.create_timer(period, self.log_sample)

        self.get_logger().info(f'Data logger started. Writing to: {self.run_dir}')

    # =========================================================
    # Setup
    # =========================================================
    def _make_dirs(self):
        self.metadata_dir.mkdir(parents=True, exist_ok=True)
        self.records_dir.mkdir(parents=True, exist_ok=True)
        self.dalsa2_dir.mkdir(parents=True, exist_ok=True)
        self.leopard4_dir.mkdir(parents=True, exist_ok=True)
        self.leopard5_dir.mkdir(parents=True, exist_ok=True)
        self.seyond6_dir.mkdir(parents=True, exist_ok=True)

    def _init_csv(self):
        header = [
            'sample_id',
            'episode_id',
            'timestamp_ros_sec',
            'timestamp_ros_nanosec',

            'cmd_throttle',
            'cmd_steer',
            'cmd_brake',
            'cmd_reverse',
            'cmd_gear',

            'applied_throttle',
            'applied_steer',
            'applied_brake',
            'applied_reverse',
            'applied_gear',

            'speed',
            'yaw',

            'pose_x',
            'pose_y',
            'pose_z',
            'ori_x',
            'ori_y',
            'ori_z',
            'ori_w',

            'goal_x',
            'goal_y',
            'goal_yaw',
            'goal_rel_x',
            'goal_rel_y',
            'goal_distance',
            'goal_yaw_error',

            'laser1_distance',
            'laser2_distance',
            'laser3_distance',
            'laser4_distance',

            'imu_ang_vel_x',
            'imu_ang_vel_y',
            'imu_ang_vel_z',
            'imu_lin_acc_x',
            'imu_lin_acc_y',
            'imu_lin_acc_z',
            'imu_ori_x',
            'imu_ori_y',
            'imu_ori_z',
            'imu_ori_w',

            'dalsa2_rgb_path',
            'leopard4_rgb_path',
            'leopard5_rgb_path',
            'seyond6_lidar_path',
        ]

        with open(self.csv_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(header)

    def _write_metadata(self):
        run_info = {
            'node_name': 'data_logger_node',
            'created_at': datetime.now().isoformat(),
            'image_format': self.image_format,
            'log_rate_hz': self.log_rate_hz,
            'ego_yaw_unit': 'degrees',
            'ego_yaw_range': '[-180, 180]',
            'goal_pose_world': {
                'x': CARGOBOX_COORDINATES[0],
                'y': CARGOBOX_COORDINATES[1],
                'yaw_degrees': CARGOBOX_COORDINATES[2],
            }
        }

        schema = {
            'control_cmd': [
                'cmd_throttle', 'cmd_steer', 'cmd_brake', 'cmd_reverse', 'cmd_gear'
            ],
            'control_applied': [
                'applied_throttle', 'applied_steer', 'applied_brake', 'applied_reverse', 'applied_gear'
            ],
            'vehicle_state': [
                'speed', 'yaw',
                'pose_x', 'pose_y', 'pose_z',
                'ori_x', 'ori_y', 'ori_z', 'ori_w'
            ],
            'goal_relative': [
                'goal_x', 'goal_y', 'goal_yaw',
                'goal_rel_x', 'goal_rel_y',
                'goal_distance', 'goal_yaw_error'
            ],
            'lasers': [
                'laser1_distance', 'laser2_distance', 'laser3_distance', 'laser4_distance'
            ],
            'imu': [
                'imu_ang_vel_x', 'imu_ang_vel_y', 'imu_ang_vel_z',
                'imu_lin_acc_x', 'imu_lin_acc_y', 'imu_lin_acc_z',
                'imu_ori_x', 'imu_ori_y', 'imu_ori_z', 'imu_ori_w'
            ],
            'sensor_files': [
                'dalsa2_rgb_path', 'leopard4_rgb_path', 'leopard5_rgb_path', 'seyond6_lidar_path'
            ]
        }

        with open(self.metadata_dir / 'run_info.json', 'w') as f:
            json.dump(run_info, f, indent=2)

        with open(self.metadata_dir / 'schema.json', 'w') as f:
            json.dump(schema, f, indent=2)

    # =========================================================
    # Utility
    # =========================================================
    def wrap_to_pi(self, angle_rad: float) -> float:
        return (angle_rad + np.pi) % (2.0 * np.pi) - np.pi

    def current_yaw_rad(self):
        if self.latest_yaw is None:
            return None
        return np.deg2rad(self.latest_yaw)

    def goal_yaw_rad(self):
        return np.deg2rad(CARGOBOX_COORDINATES[2])

    def compute_goal_relative(self):
        if self.latest_pose_x is None or self.latest_pose_y is None or self.latest_yaw is None:
            return None, None, None, None

        goal_x, goal_y, _ = CARGOBOX_COORDINATES

        dx_world = goal_x - self.latest_pose_x
        dy_world = goal_y - self.latest_pose_y

        yaw = self.current_yaw_rad()
        if yaw is None:
            return None, None, None, None

        # Transform goal from world frame into ego frame
        goal_rel_x = np.cos(yaw) * dx_world + np.sin(yaw) * dy_world
        goal_rel_y = -np.sin(yaw) * dx_world + np.cos(yaw) * dy_world
        goal_distance = float(np.hypot(goal_rel_x, goal_rel_y))

        goal_yaw = self.goal_yaw_rad()
        goal_yaw_error = float(self.wrap_to_pi(goal_yaw - yaw))

        return float(goal_rel_x), float(goal_rel_y), goal_distance, goal_yaw_error

    # =========================================================
    # Callbacks
    # =========================================================
    def dalsa2_cb(self, msg: Image):
        try:
            self.latest_dalsa2 = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'dalsa2_cb failed: {e}')

    def leopard4_cb(self, msg: Image):
        try:
            self.latest_leopard4 = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'leopard4_cb failed: {e}')

    def leopard5_cb(self, msg: Image):
        try:
            self.latest_leopard5 = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'leopard5_cb failed: {e}')

    def seyond6_cb(self, msg: PointCloud2):
        try:
            pts = []
            for p in point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
                pts.append([p[0], p[1], p[2]])

            if len(pts) == 0:
                self.latest_seyond6_xyz = np.zeros((0, 3), dtype=np.float32)
            else:
                self.latest_seyond6_xyz = np.asarray(pts, dtype=np.float32)

        except Exception as e:
            self.get_logger().warn(f'seyond6_cb failed: {e}')

    def laser1_cb(self, msg: Float32):
        self.latest_laser1 = float(msg.data)

    def laser2_cb(self, msg: Float32):
        self.latest_laser2 = float(msg.data)

    def laser3_cb(self, msg: Float32):
        self.latest_laser3 = float(msg.data)

    def laser4_cb(self, msg: Float32):
        self.latest_laser4 = float(msg.data)

    def imu_cb(self, msg: Imu):
        self.latest_imu = msg

    def control_cmd_cb(self, msg: VehicleControl):
        self.latest_cmd_throttle = float(msg.throttle)
        self.latest_cmd_steer = float(msg.steer)
        self.latest_cmd_brake = float(msg.brake)
        self.latest_cmd_reverse = int(msg.reverse)
        self.latest_cmd_gear = int(msg.gear)

    def control_applied_cb(self, msg: VehicleControl):
        self.latest_applied_throttle = float(msg.throttle)
        self.latest_applied_steer = float(msg.steer)
        self.latest_applied_brake = float(msg.brake)
        self.latest_applied_reverse = int(msg.reverse)
        self.latest_applied_gear = int(msg.gear)

    def speed_cb(self, msg: Float32):
        self.latest_speed = float(msg.data)

    def yaw_cb(self, msg: Float32):
        self.latest_yaw = float(msg.data)

    def pose_cb(self, msg: PoseStamped):
        self.latest_pose_x = float(msg.pose.position.x)
        self.latest_pose_y = float(msg.pose.position.y)
        self.latest_pose_z = float(msg.pose.position.z)

        self.latest_ori_x = float(msg.pose.orientation.x)
        self.latest_ori_y = float(msg.pose.orientation.y)
        self.latest_ori_z = float(msg.pose.orientation.z)
        self.latest_ori_w = float(msg.pose.orientation.w)

    def episode_reset_cb(self, msg: UInt32):
        self.episode_id = int(msg.data)
        self.sample_id = 0
        self.get_logger().info(f'Received new episode_id={self.episode_id}')

    # =========================================================
    # Saving helpers
    # =========================================================
    def make_sensor_stem(self, sample_id: int) -> str:
        return f'ep{self.episode_id:04d}_{sample_id:06d}'

    def save_image(self, img, directory: Path, sample_id: int):
        img_resized = cv2.resize(img, IMG_RESIZE, interpolation=cv2.INTER_AREA)
        stem = self.make_sensor_stem(sample_id)
        rel_path = Path('sensors') / directory.name / f'{stem}.{self.image_format}'
        abs_path = self.run_dir / rel_path
        cv2.imwrite(str(abs_path), img_resized)
        return str(rel_path)

    def save_lidar(self, xyz: np.ndarray, directory: Path, sample_id: int):
        stem = self.make_sensor_stem(sample_id)
        rel_path = Path('sensors') / directory.name / f'{stem}.npy'
        abs_path = self.run_dir / rel_path
        np.save(abs_path, xyz)
        return str(rel_path)

    def append_csv_row(self, row):
        with open(self.csv_path, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(row)

    # =========================================================
    # Main logging
    # =========================================================
    def log_sample(self):
        if self.latest_dalsa2 is None:
            return
        if self.latest_seyond6_xyz is None:
            return
        if self.latest_applied_throttle is None and self.latest_cmd_throttle is None:
            return

        now = self.get_clock().now().to_msg()

        goal_rel_x, goal_rel_y, goal_distance, goal_yaw_error = self.compute_goal_relative()

        self.position_error_x = goal_rel_x
        self.position_error_y = goal_rel_y
        self.goal_distance = goal_distance
        self.yaw_error = goal_yaw_error

        dalsa2_path = self.save_image(self.latest_dalsa2, self.dalsa2_dir, self.sample_id)

        leopard4_path = ''
        if self.latest_leopard4 is not None:
            leopard4_path = self.save_image(self.latest_leopard4, self.leopard4_dir, self.sample_id)

        leopard5_path = ''
        if self.latest_leopard5 is not None:
            leopard5_path = self.save_image(self.latest_leopard5, self.leopard5_dir, self.sample_id)

        seyond6_path = self.save_lidar(self.latest_seyond6_xyz, self.seyond6_dir, self.sample_id)

        imu_ang_vel_x = 0.0
        imu_ang_vel_y = 0.0
        imu_ang_vel_z = 0.0
        imu_lin_acc_x = 0.0
        imu_lin_acc_y = 0.0
        imu_lin_acc_z = 0.0
        imu_ori_x = 0.0
        imu_ori_y = 0.0
        imu_ori_z = 0.0
        imu_ori_w = 1.0

        if self.latest_imu is not None:
            imu_ang_vel_x = float(self.latest_imu.angular_velocity.x)
            imu_ang_vel_y = float(self.latest_imu.angular_velocity.y)
            imu_ang_vel_z = float(self.latest_imu.angular_velocity.z)

            imu_lin_acc_x = float(self.latest_imu.linear_acceleration.x)
            imu_lin_acc_y = float(self.latest_imu.linear_acceleration.y)
            imu_lin_acc_z = float(self.latest_imu.linear_acceleration.z)

            imu_ori_x = float(self.latest_imu.orientation.x)
            imu_ori_y = float(self.latest_imu.orientation.y)
            imu_ori_z = float(self.latest_imu.orientation.z)
            imu_ori_w = float(self.latest_imu.orientation.w)

        row = [
            self.sample_id,
            self.episode_id,
            now.sec,
            now.nanosec,

            self.latest_cmd_throttle if self.latest_cmd_throttle is not None else 0.0,
            self.latest_cmd_steer if self.latest_cmd_steer is not None else 0.0,
            self.latest_cmd_brake if self.latest_cmd_brake is not None else 0.0,
            self.latest_cmd_reverse if self.latest_cmd_reverse is not None else 0,
            self.latest_cmd_gear if self.latest_cmd_gear is not None else 0,

            self.latest_applied_throttle if self.latest_applied_throttle is not None else 0.0,
            self.latest_applied_steer if self.latest_applied_steer is not None else 0.0,
            self.latest_applied_brake if self.latest_applied_brake is not None else 0.0,
            self.latest_applied_reverse if self.latest_applied_reverse is not None else 0,
            self.latest_applied_gear if self.latest_applied_gear is not None else 0,

            self.latest_speed if self.latest_speed is not None else 0.0,
            self.latest_yaw if self.latest_yaw is not None else 0.0,

            self.latest_pose_x if self.latest_pose_x is not None else 0.0,
            self.latest_pose_y if self.latest_pose_y is not None else 0.0,
            self.latest_pose_z if self.latest_pose_z is not None else 0.0,
            self.latest_ori_x if self.latest_ori_x is not None else 0.0,
            self.latest_ori_y if self.latest_ori_y is not None else 0.0,
            self.latest_ori_z if self.latest_ori_z is not None else 0.0,
            self.latest_ori_w if self.latest_ori_w is not None else 1.0,

            CARGOBOX_COORDINATES[0],
            CARGOBOX_COORDINATES[1],
            CARGOBOX_COORDINATES[2],
            goal_rel_x if goal_rel_x is not None else 0.0,
            goal_rel_y if goal_rel_y is not None else 0.0,
            goal_distance if goal_distance is not None else 0.0,
            goal_yaw_error if goal_yaw_error is not None else 0.0,

            self.latest_laser1 if self.latest_laser1 is not None else 0.0,
            self.latest_laser2 if self.latest_laser2 is not None else 0.0,
            self.latest_laser3 if self.latest_laser3 is not None else 0.0,
            self.latest_laser4 if self.latest_laser4 is not None else 0.0,

            imu_ang_vel_x,
            imu_ang_vel_y,
            imu_ang_vel_z,
            imu_lin_acc_x,
            imu_lin_acc_y,
            imu_lin_acc_z,
            imu_ori_x,
            imu_ori_y,
            imu_ori_z,
            imu_ori_w,

            dalsa2_path,
            leopard4_path,
            leopard5_path,
            seyond6_path,
        ]

        self.append_csv_row(row)
        self.sample_id += 1


def main(args=None):
    rclpy.init(args=args)
    node = DataLoggerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()