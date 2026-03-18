import csv
import json
import os
from pathlib import Path
from datetime import datetime

import numpy as np
import cv2

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32, String
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Image, PointCloud2
from ackermann_msgs.msg import AckermannDriveStamped

from cv_bridge import CvBridge
from sensor_msgs_py import point_cloud2


class DataLoggerNode(Node):
    def __init__(self):
        super().__init__('data_logger_node')

        # -----------------------------
        # Parameters
        # -----------------------------
        self.declare_parameter('dataset_root', str(Path.home() / 'carla_datasets'))
        self.declare_parameter('run_name', '')
        self.declare_parameter('image_format', 'png')
        self.declare_parameter('log_rate_hz', 10.0)

        dataset_root = Path(self.get_parameter('dataset_root').value)
        run_name = self.get_parameter('run_name').value
        self.image_format = self.get_parameter('image_format').value
        self.log_rate_hz = float(self.get_parameter('log_rate_hz').value)

        if run_name == '':
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            run_name = f'dataset_run_{timestamp}'

        self.run_dir = dataset_root / run_name
        self.metadata_dir = self.run_dir / 'metadata'
        self.records_dir = self.run_dir / 'records'
        self.sensors_dir = self.run_dir / 'sensors'

        self.front_left_dir = self.sensors_dir / 'front_left_rgb'
        self.front_right_dir = self.sensors_dir / 'front_right_rgb'
        self.front_lidar_dir = self.sensors_dir / 'front_lidar'

        self._make_dirs()

        # -----------------------------
        # Dataset bookkeeping
        # -----------------------------
        self.bridge = CvBridge()
        self.sample_id = 0
        self.episode_id = 0

        self.latest_front_left = None
        self.latest_front_right = None
        self.latest_lidar_xyz = None

        self.latest_steer = None
        self.latest_throttle = None
        self.latest_brake = None
        self.latest_reverse = None

        self.latest_speed = None
        self.latest_pose_x = None
        self.latest_pose_y = None
        self.latest_yaw = None

        self.latest_phase = 'unknown'
        self.latest_target_slot_id = 'none'

        self.csv_path = self.records_dir / 'samples.csv'
        self._init_csv()

        self._write_metadata()

        # -----------------------------
        # Subscribers
        # Replace these topics with your real ones
        # -----------------------------
        self.create_subscription(Image, '/sensors/front_left_rgb', self.front_left_cb, 10)
        self.create_subscription(Image, '/sensors/front_right_rgb', self.front_right_cb, 10)
        self.create_subscription(PointCloud2, '/sensors/front_lidar', self.front_lidar_cb, 10)

        # Example control topic (Ackermann)
        self.create_subscription(AckermannDriveStamped, '/vehicle/control_cmd', self.control_cb, 10)

        # Example speed topic
        self.create_subscription(Float32, '/ego/speed', self.speed_cb, 10)

        # Example high-level state topics
        self.create_subscription(String, '/parking/phase', self.phase_cb, 10)
        self.create_subscription(String, '/parking/target_slot_id', self.target_slot_cb, 10)

        # -----------------------------
        # Logging timer
        # -----------------------------
        period = 1.0 / self.log_rate_hz
        self.timer = self.create_timer(period, self.log_sample)

        self.get_logger().info(f'Data logger started. Writing to: {self.run_dir}')

    # =========================================================
    # Directory / metadata
    # =========================================================
    def _make_dirs(self):
        self.metadata_dir.mkdir(parents=True, exist_ok=True)
        self.records_dir.mkdir(parents=True, exist_ok=True)
        self.front_left_dir.mkdir(parents=True, exist_ok=True)
        self.front_right_dir.mkdir(parents=True, exist_ok=True)
        self.front_lidar_dir.mkdir(parents=True, exist_ok=True)

    def _init_csv(self):
        header = [
            'sample_id',
            'episode_id',
            'timestamp_ros_sec',
            'timestamp_ros_nanosec',

            'parking_phase',
            'target_slot_id',

            'steer',
            'throttle',
            'brake',
            'reverse',

            'speed',

            'pose_x',
            'pose_y',
            'yaw',

            'front_left_rgb_path',
            'front_right_rgb_path',
            'front_lidar_path',
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
        }

        schema = {
            'control': ['steer', 'throttle', 'brake', 'reverse'],
            'vehicle_state': ['speed', 'pose_x', 'pose_y', 'yaw'],
            'task': ['parking_phase', 'target_slot_id'],
            'sensors': ['front_left_rgb_path', 'front_right_rgb_path', 'front_lidar_path'],
        }

        with open(self.metadata_dir / 'run_info.json', 'w') as f:
            json.dump(run_info, f, indent=2)

        with open(self.metadata_dir / 'schema.json', 'w') as f:
            json.dump(schema, f, indent=2)

    # =========================================================
    # Callbacks
    # =========================================================
    def front_left_cb(self, msg: Image):
        try:
            self.latest_front_left = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'front_left_cb failed: {e}')

    def front_right_cb(self, msg: Image):
        try:
            self.latest_front_right = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'front_right_cb failed: {e}')

    def front_lidar_cb(self, msg: PointCloud2):
        try:
            points = []
            for p in point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
                points.append([p[0], p[1], p[2]])

            if len(points) == 0:
                self.latest_lidar_xyz = np.zeros((0, 3), dtype=np.float32)
            else:
                self.latest_lidar_xyz = np.asarray(points, dtype=np.float32)

        except Exception as e:
            self.get_logger().warn(f'front_lidar_cb failed: {e}')

    def control_cb(self, msg: AckermannDriveStamped):
        # Adapt this callback if you publish a different control message type
        try:
            self.latest_steer = float(msg.drive.steering_angle)
            self.latest_throttle = float(msg.drive.acceleration) if msg.drive.acceleration > 0.0 else 0.0
            self.latest_brake = abs(float(msg.drive.acceleration)) if msg.drive.acceleration < 0.0 else 0.0
            self.latest_reverse = 0
        except Exception as e:
            self.get_logger().warn(f'control_cb failed: {e}')

    def speed_cb(self, msg: Float32):
        self.latest_speed = float(msg.data)

    def phase_cb(self, msg: String):
        self.latest_phase = msg.data

    def target_slot_cb(self, msg: String):
        self.latest_target_slot_id = msg.data

    # =========================================================
    # Saving helpers
    # =========================================================
    def save_image(self, img, directory: Path, sample_id: int):
        rel_path = Path('sensors') / directory.name / f'{sample_id:06d}.{self.image_format}'
        abs_path = self.run_dir / rel_path
        cv2.imwrite(str(abs_path), img)
        return str(rel_path)

    def save_lidar(self, xyz: np.ndarray, directory: Path, sample_id: int):
        rel_path = Path('sensors') / directory.name / f'{sample_id:06d}.npy'
        abs_path = self.run_dir / rel_path
        np.save(abs_path, xyz)
        return str(rel_path)

    def append_csv_row(self, row):
        with open(self.csv_path, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(row)

    # =========================================================
    # Main logging loop
    # =========================================================
    def log_sample(self):
        # For now, require at least one control signal and one sensor
        if self.latest_front_left is None:
            return
        if self.latest_lidar_xyz is None:
            return
        if self.latest_steer is None:
            return

        now = self.get_clock().now().to_msg()

        # Save sensor files
        front_left_path = self.save_image(self.latest_front_left, self.front_left_dir, self.sample_id)

        if self.latest_front_right is not None:
            front_right_path = self.save_image(self.latest_front_right, self.front_right_dir, self.sample_id)
        else:
            front_right_path = ''

        front_lidar_path = self.save_lidar(self.latest_lidar_xyz, self.front_lidar_dir, self.sample_id)

        # Fill missing numeric fields conservatively
        steer = self.latest_steer if self.latest_steer is not None else 0.0
        throttle = self.latest_throttle if self.latest_throttle is not None else 0.0
        brake = self.latest_brake if self.latest_brake is not None else 0.0
        reverse = self.latest_reverse if self.latest_reverse is not None else 0

        speed = self.latest_speed if self.latest_speed is not None else 0.0
        pose_x = self.latest_pose_x if self.latest_pose_x is not None else 0.0
        pose_y = self.latest_pose_y if self.latest_pose_y is not None else 0.0
        yaw = self.latest_yaw if self.latest_yaw is not None else 0.0

        row = [
            self.sample_id,
            self.episode_id,
            now.sec,
            now.nanosec,

            self.latest_phase,
            self.latest_target_slot_id,

            steer,
            throttle,
            brake,
            reverse,

            speed,

            pose_x,
            pose_y,
            yaw,

            front_left_path,
            front_right_path,
            front_lidar_path,
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