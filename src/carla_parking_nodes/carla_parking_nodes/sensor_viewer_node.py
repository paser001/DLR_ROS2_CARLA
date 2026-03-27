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
LIDAR_HISTORY_LENGTH = 2

LIDAR_OFFSET= -0.0

VEH_X_MIN = -3.4*0.623
VEH_X_MAX = 3.4*0.623
VEH_Y_MIN = -2.6*0.623
VEH_Y_MAX = 2.6*0.623



LIDAR_LAYOUT = {
    'seyond6': {
        "xyz": (0.6, -0.20, 0.55),
        "rpy": (0.0, 0.0, 0.0),
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
            'dalsa2': '/sensors/dalsa2_rgb/image_raw',
            'leopard4': '/sensors/leopard4_rgb/image_raw',
            'leopard5': '/sensors/leopard5_rgb/image_raw',
        }

        self.lidar_topics = {
            'seyond6': '/sensors/seyond6/points',
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
            cv2.namedWindow(name, cv2.WINDOW_NORMAL)

        for name in self.lidar_topics.keys():
            cv2.namedWindow(f'{name}_lidar', cv2.WINDOW_AUTOSIZE)

        cv2.namedWindow('surround_lidar_fused', cv2.WINDOW_AUTOSIZE)

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


    def count_points_in_box_from_pts(self, pts: np.ndarray, x_min, x_max, y_min, y_max) -> int:
        if pts is None or len(pts) == 0:
            return 0

        x = pts[:, 0]
        y = pts[:, 1]

        mask = (x >= x_min) & (x <= x_max) & (y >= y_min) & (y <= y_max)
        return int(np.count_nonzero(mask))
    
    def draw_controller_safety_boxes(self, img: np.ndarray, pts: np.ndarray = None):
        

        zones = [
            ("front", VEH_X_MAX, VEH_X_MAX + 2.2, -3.0, 3.0, (0, 0, 255)),
            ("rear", VEH_X_MIN - 2.2, VEH_X_MIN, -3.0, 3.0, (255, 0, 0)),

            ("fr", 2.8, 5.0, 2.6, 3.8, (255, 255, 0)),
            ("fl", 2.8, 5.0, -3.8, -2.6, (0, 255, 255)),

            ("rr", -5.5, -2.0, 2.6, 3.8, (0, 255, 0)),
            ("rl", -5.5, -2.0, -3.8, -2.6, (255, 0, 255)),
        ]

        for name, x_min, x_max, y_min, y_max, color in zones:
            self.draw_box_on_bev(
                img,
                x_min=x_min,
                x_max=x_max,
                y_min=y_min,
                y_max=y_max,
                color=color,
                thickness=2
            )

            label = name
            if pts is not None:
                count = self.count_points_in_box_from_pts(pts, x_min, x_max, y_min, y_max)
                label = f'{name}:{count}'
                # print(label)

            

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
            # self.draw_ego_marker(bev)
            # self.draw_candidate_vehicle_boxes(bev)
            # self.draw_parking_safety_boxes(bev)
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

        # self.draw_ego_marker(bev)
        # self.draw_candidate_vehicle_boxes(bev)
        # self.draw_parking_safety_boxes(bev)

        return bev

    def fused_lidar_history_to_bev(self) -> np.ndarray:
        img_size = LIDAR_IMG_SIZE
        lidar_range = LIDAR_RANGE_M
        scale = img_size / (2.0 * lidar_range)

        bev = np.zeros((img_size, img_size, 3), dtype=np.uint8)

        sensor_name = 'seyond6'
        history_list = list(self.lidar_history.get(sensor_name, []))
        n = len(history_list)

        color = (255, 255, 255)

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

        self.draw_candidate_vehicle_boxes(bev)

        latest_pts = self.latest_lidar_points_vehicle.get('seyond6', None)
        # self.draw_controller_safety_boxes(bev, latest_pts)

        return bev

    def draw_ego_marker(self, img: np.ndarray):
        center = img.shape[0] // 2
        cv2.circle(img, (center, center), 6, (0, 0, 255), -1)
        cv2.arrowedLine(img, (center, center), (center, center - 70), (0, 255, 0), 2)

    def draw_box_on_bev(self, img, x_min, x_max, y_min, y_max, color=(0, 0, 255), thickness=2):
        img_size = img.shape[0]
        scale = img_size / (2.0 * LIDAR_RANGE_M)
        center = (img_size // 2)

        def to_px(x, y):
            px = int(center + y * scale)
            py = int(center - (x + LIDAR_OFFSET) * scale)
            return px, py

        p1 = to_px(x_min, y_min)
        p2 = to_px(x_min, y_max)
        p3 = to_px(x_max, y_max)
        p4 = to_px(x_max, y_min)

        cv2.line(img, p1, p2, color, thickness)
        cv2.line(img, p2, p3, color, thickness)
        cv2.line(img, p3, p4, color, thickness)
        cv2.line(img, p4, p1, color, thickness)

    def draw_text_at_vehicle_coords(self, img: np.ndarray, x: float, y: float, text: str, color=(255, 255, 255)):
        img_size = img.shape[0]
        scale = img_size / (2.0 * LIDAR_RANGE_M)
        center = img_size // 2

        px = int(center + y * scale)
        py = int(center - x * scale)

        cv2.putText(
            img,
            text,
            (px + 4, py - 4),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            color,
            1,
            cv2.LINE_AA
        )

    def draw_candidate_vehicle_boxes(self, img: np.ndarray):
        boxes = [
            (-3.4,  3.4, -2.6, 2.6, (0, 255, 0),     "ushift"),
        ]

        for x_min, x_max, y_min, y_max, color, label in boxes:
            self.draw_box_on_bev(
                img,
                x_min=x_min,
                x_max=x_max,
                y_min=y_min,
                y_max=y_max,
                color=color,
                thickness=2
            )

            self.draw_text_at_vehicle_coords(
                img,
                x=x_max,
                y=y_min,
                text=label,
                color=color
            )

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
            display_img = cv2.resize(img, (640, 480))
            cv2.imshow(name, display_img)
            # cv2.imshow(name, img)

        for name, bev in self.latest_lidar_bev.items():
            cv2.imshow(f'{name}_lidar', bev)

        fused = self.fused_lidar_history_to_bev()
        cv2.imshow('surround_lidar_fused', fused)

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