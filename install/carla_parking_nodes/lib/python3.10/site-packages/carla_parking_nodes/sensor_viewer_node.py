#!/usr/bin/env python3

from collections import deque
from typing import Deque, Dict, List

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
from sensor_msgs_py import point_cloud2


LIDAR_IMG_SIZE = 700
LIDAR_RANGE_M = 30.0
LIDAR_HISTORY_LENGTH = 8

# Same extrinsics used in your CARLA interface node
# URDF convention: x forward, y left, z up
LIDAR_LAYOUT = {
    'front_lidar': {
        'xyz': (0.486, 0.0, 0.51),
        'rpy': (0.007, -0.05, 0.00735),
    },
    'rear_left_lidar': {
        'xyz': (-0.671, 0.839, 0.402),
        'rpy': (0.021, 0.019, 2.125),
    },
    'rear_right_lidar': {
        'xyz': (-0.671, -0.839, 0.402),
        'rpy': (0.0, -0.007, -2.101),
    },
}


class SensorViewerNode(Node):
    def __init__(self):
        super().__init__('sensor_viewer_node')

        self.bridge = CvBridge()

        self.latest_images: Dict[str, np.ndarray] = {}
        self.latest_lidar_bev: Dict[str, np.ndarray] = {}
        self.latest_lidar_points_vehicle: Dict[str, np.ndarray] = {}

        self.camera_topics = {
            'front_left_rgb': '/sensors/front_left_rgb/image_raw',
            'front_right_rgb': '/sensors/front_right_rgb/image_raw',
        }

        self.lidar_topics = {
            'front_lidar': '/sensors/front_lidar/points',
            'rear_left_lidar': '/sensors/rear_left_lidar/points',
            'rear_right_lidar': '/sensors/rear_right_lidar/points',
        }

        self.lidar_history_length = LIDAR_HISTORY_LENGTH
        self.lidar_history: Dict[str, Deque[np.ndarray]] = {
            name: deque(maxlen=self.lidar_history_length)
            for name in self.lidar_topics.keys()
        }

        self.camera_subs = []
        self.lidar_subs = []

        for name, topic in self.camera_topics.items():
            sub = self.create_subscription(
                Image,
                topic,
                lambda msg, sensor_name=name: self.camera_callback(msg, sensor_name),
                qos_profile_sensor_data
            )
            self.camera_subs.append(sub)
            self.get_logger().info(f'Subscribed to camera topic: {topic}')

        for name, topic in self.lidar_topics.items():
            sub = self.create_subscription(
                PointCloud2,
                topic,
                lambda msg, sensor_name=name: self.lidar_callback(msg, sensor_name),
                qos_profile_sensor_data
            )
            self.lidar_subs.append(sub)
            self.get_logger().info(f'Subscribed to lidar topic: {topic}')

        for name in self.camera_topics.keys():
            cv2.namedWindow(name, cv2.WINDOW_AUTOSIZE)

        for name in self.lidar_topics.keys():
            cv2.namedWindow(f'{name}_bev', cv2.WINDOW_AUTOSIZE)

        cv2.namedWindow('surround_lidar_bev', cv2.WINDOW_AUTOSIZE)

        self.display_timer = self.create_timer(0.03, self.display_loop)

        self.get_logger().info('sensor_viewer_node started')

    def camera_callback(self, msg: Image, sensor_name: str):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.latest_images[sensor_name] = img
        except Exception as e:
            self.get_logger().warn(f'Camera callback failed for {sensor_name}: {e}')

    def lidar_callback(self, msg: PointCloud2, sensor_name: str):
        try:
            raw_points = list(point_cloud2.read_points(
                msg,
                field_names=('x', 'y', 'z', 'intensity'),
                skip_nans=True
            ))

            if len(raw_points) == 0:
                return

            first = raw_points[0]

            if isinstance(first, (tuple, list)):
                pts_sensor = np.array(raw_points, dtype=np.float32)
            else:
                arr = np.array(raw_points)
                if arr.dtype.names is None:
                    pts_sensor = arr.astype(np.float32)
                else:
                    pts_sensor = np.stack([
                        arr['x'],
                        arr['y'],
                        arr['z'],
                        arr['intensity']
                    ], axis=1).astype(np.float32)

            pts_vehicle = self.transform_points_to_vehicle_frame(pts_sensor, sensor_name)

            self.latest_lidar_points_vehicle[sensor_name] = pts_vehicle
            self.lidar_history[sensor_name].append(pts_vehicle)

            self.latest_lidar_bev[sensor_name] = self.pointcloud_history_to_bev(
                list(self.lidar_history[sensor_name]),
                sensor_name
            )

        except Exception as e:
            self.get_logger().warn(f'Lidar callback failed for {sensor_name}: {e}')



    def urdf_pose_to_vehicle_frame(self, xyz, rpy):
        """
        Convert URDF sensor pose to vehicle frame convention:
        x forward, y right, z up
        """
        x_u, y_u, z_u = xyz
        roll_u, pitch_u, yaw_u = rpy

        x_v = x_u
        y_v = -y_u
        z_v = z_u

        roll_v = roll_u
        pitch_v = pitch_u
        yaw_v = -yaw_u

        return (x_v, y_v, z_v), (roll_v, pitch_v, yaw_v)


    def transform_points_to_vehicle_frame(self, pts: np.ndarray, sensor_name: str) -> np.ndarray:
        if sensor_name not in LIDAR_LAYOUT:
            return pts

        xyz = LIDAR_LAYOUT[sensor_name]['xyz']
        rpy = LIDAR_LAYOUT[sensor_name]['rpy']

        (tx, ty, tz), (_, _, yaw) = self.urdf_pose_to_vehicle_frame(xyz, rpy)

        forward = np.array([np.cos(yaw), np.sin(yaw)], dtype=np.float32)
        right = np.array([-np.sin(yaw), np.cos(yaw)], dtype=np.float32)

        xs = pts[:, 0]
        ys = pts[:, 1]
        zs = pts[:, 2]
        intensity = pts[:, 3]

        xy_vehicle = (
            np.outer(xs, forward) +
            np.outer(ys, right)
        )

        x_vehicle = xy_vehicle[:, 0] + tx
        y_vehicle = xy_vehicle[:, 1] + ty
        z_vehicle = zs + tz

        pts_vehicle = np.stack(
            [x_vehicle, y_vehicle, z_vehicle, intensity],
            axis=1
        ).astype(np.float32)

        return pts_vehicle

    def pointcloud_history_to_bev(self, pts_history: List[np.ndarray], sensor_name: str) -> np.ndarray:
        img_size = LIDAR_IMG_SIZE
        lidar_range = LIDAR_RANGE_M
        scale = img_size / (2.0 * lidar_range)

        bev = np.zeros((img_size, img_size, 3), dtype=np.uint8)

        n = len(pts_history)
        if n == 0:
            self.draw_ego_marker(bev)
            self.draw_parking_safety_boxes(bev)
            return bev

        for i, pts in enumerate(pts_history):
            if pts is None or pts.size == 0:
                continue

            x = pts[:, 0]
            y = pts[:, 1]

            mask = (
                (x >= -lidar_range) & (x <= lidar_range) &
                (y >= -lidar_range) & (y <= lidar_range)
            )

            x = x[mask]
            y = y[mask]

            px = (img_size / 2 + y * scale).astype(np.int32)
            py = (img_size / 2 - x * scale).astype(np.int32)

            valid = (
                (px >= 0) & (px < img_size) &
                (py >= 0) & (py < img_size)
            )

            px = px[valid]
            py = py[valid]

            brightness = int(60 + 195 * (i + 1) / max(n, 1))
            bev[py, px] = (brightness, brightness, brightness)

        self.draw_ego_marker(bev)
        self.draw_parking_safety_boxes(bev)

        return bev

    def fused_lidar_history_to_bev(self) -> np.ndarray:
        img_size = LIDAR_IMG_SIZE
        lidar_range = LIDAR_RANGE_M
        scale = img_size / (2.0 * lidar_range)

        bev = np.zeros((img_size, img_size, 3), dtype=np.uint8)

        colors = {
            'front_lidar': (255, 255, 255),
            'rear_left_lidar': (0, 255, 255),
            'rear_right_lidar': (255, 0, 255),
        }

        for sensor_name, history in self.lidar_history.items():
            color = colors.get(sensor_name, (200, 200, 200))
            history_list = list(history)
            n = len(history_list)

            for i, pts in enumerate(history_list):
                if pts is None or pts.size == 0:
                    continue

                x = pts[:, 0]
                y = pts[:, 1]

                mask = (
                    (x >= -lidar_range) & (x <= lidar_range) &
                    (y >= -lidar_range) & (y <= lidar_range)
                )

                x = x[mask]
                y = y[mask]

                px = (img_size / 2 + y * scale).astype(np.int32)
                py = (img_size / 2 - x * scale).astype(np.int32)

                valid = (
                    (px >= 0) & (px < img_size) &
                    (py >= 0) & (py < img_size)
                )

                px = px[valid]
                py = py[valid]

                fade = 0.35 + 0.65 * (i + 1) / max(n, 1)
                draw_color = (
                    int(color[0] * fade),
                    int(color[1] * fade),
                    int(color[2] * fade),
                )

                bev[py, px] = draw_color

        self.draw_ego_marker(bev)
        self.draw_parking_safety_boxes(bev)

        return bev

    def draw_ego_marker(self, img: np.ndarray):
        center = img.shape[0] // 2
        cv2.circle(img, (center, center), 6, (0, 0, 255), -1)
        cv2.arrowedLine(img, (center, center), (center, center - 70), (0, 255, 0), 2)

    def draw_box_on_bev(self, img, x_min, x_max, y_min, y_max, color=(0, 0, 255), thickness=2):
        img_size = img.shape[0]
        scale = img_size / (2.0 * LIDAR_RANGE_M)
        center = img_size // 2

        def to_px(x, y):
            px = int(center + y * scale)
            py = int(center - x * scale)
            return px, py

        p1 = to_px(x_min, y_min)
        p2 = to_px(x_min, y_max)
        p3 = to_px(x_max, y_max)
        p4 = to_px(x_max, y_min)

        cv2.line(img, p1, p2, color, thickness)
        cv2.line(img, p2, p3, color, thickness)
        cv2.line(img, p3, p4, color, thickness)
        cv2.line(img, p4, p1, color, thickness)

    def draw_parking_safety_boxes(self, img: np.ndarray):
        self.draw_box_on_bev(
            img,
            x_min=-8.0, x_max=-1.2,
            y_min=-1.8, y_max=1.8,
            color=(0, 255, 255),
            thickness=2
        )

        self.draw_box_on_bev(
            img,
            x_min=-5.5, x_max=-1.2,
            y_min=-1.4, y_max=1.4,
            color=(0, 0, 255),
            thickness=2
        )

        self.draw_box_on_bev(
            img,
            x_min=-2.3, x_max=2.3,
            y_min=-1.0, y_max=1.0,
            color=(0, 180, 0),
            thickness=1
        )

    def display_loop(self):
        for name, img in self.latest_images.items():
            cv2.imshow(name, img)

        for name, bev in self.latest_lidar_bev.items():
            cv2.imshow(f'{name}_bev', bev)

        fused = self.fused_lidar_history_to_bev()
        cv2.imshow('surround_lidar_bev', fused)

        key = cv2.waitKey(1)
        if key == 27:
            self.get_logger().info('ESC pressed, shutting down sensor_viewer_node')
            rclpy.shutdown()

    def destroy_node(self):
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = SensorViewerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()

        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()