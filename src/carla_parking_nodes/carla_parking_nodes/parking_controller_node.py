import math
from typing import Optional

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from carla_parking_msgs.msg import VehicleControl
from carla_parking_nodes.parking_slots import (
    get_slot_by_id,
    make_target_pose,
    make_staging_pose,
)


COM_OFFSET = -0.5

VEH_X_MIN = -3.4
VEH_X_MAX = 3.4
VEH_Y_MIN = -2.6
VEH_Y_MAX = 2.6

LIDAR_XYZ = (2.4, 0.0, 1.65)
LIDAR_RPY = (0.0, 0.0, 0.0)


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def normalize_angle_deg(a):
    while a > 180.0:
        a -= 360.0
    while a < -180.0:
        a += 360.0
    return a


def quaternion_to_yaw_deg(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.degrees(math.atan2(siny_cosp, cosy_cosp))


class ParkingControllerNode(Node):
    def __init__(self):
        super().__init__('parking_controller_node')

        self.control_pub = self.create_publisher(VehicleControl, '/vehicle/control_cmd', 10)

        self.pose_sub = self.create_subscription(
            PoseStamped, '/ego/pose', self.pose_callback, 10
        )
        self.speed_sub = self.create_subscription(
            Float32, '/ego/speed', self.speed_callback, 10
        )
        self.lidar_sub = self.create_subscription(
            PointCloud2,
            '/sensors/front_lidar/points',
            self.lidar_callback,
            qos_profile_sensor_data
        )

        self.declare_parameter('target_slot_id', '3-15')
        self.declare_parameter('control_rate', 20.0)

        self.declare_parameter('com_offset', COM_OFFSET)

        # Only front / rear center boxes hard-stop
        self.declare_parameter('front_stop_min_points', 800)
        self.declare_parameter('rear_stop_min_points', 700)

        # Corners only caution
        self.declare_parameter('front_corner_caution_min_points', 120)
        self.declare_parameter('rear_corner_caution_min_points', 120)

        self.declare_parameter('recover_forward_distance', 2.0)
        self.declare_parameter('block_confirm_frames', 2)

        self.declare_parameter('debug_box_counts', False)

        self.target_slot_id = str(self.get_parameter('target_slot_id').value)
        self.control_rate = float(self.get_parameter('control_rate').value)

        self.com_offset = float(self.get_parameter('com_offset').value)

        self.front_stop_min_points = int(self.get_parameter('front_stop_min_points').value)
        self.rear_stop_min_points = int(self.get_parameter('rear_stop_min_points').value)

        self.front_corner_caution_min_points = int(self.get_parameter('front_corner_caution_min_points').value)
        self.rear_corner_caution_min_points = int(self.get_parameter('rear_corner_caution_min_points').value)

        self.recover_forward_distance = float(self.get_parameter('recover_forward_distance').value)
        self.block_confirm_frames = int(self.get_parameter('block_confirm_frames').value)

        self.debug_box_counts_enabled = bool(self.get_parameter('debug_box_counts').value)

        self.ego_ref_x: Optional[float] = None
        self.ego_ref_y: Optional[float] = None
        self.ego_ref_z: Optional[float] = None
        self.ego_x: Optional[float] = None
        self.ego_y: Optional[float] = None
        self.ego_z: Optional[float] = None
        self.ego_yaw: Optional[float] = None
        self.ego_speed: float = 0.0

        self.latest_lidar_points_vehicle: Optional[np.ndarray] = None

        self.slot = get_slot_by_id(self.target_slot_id)
        if self.slot is None:
            raise RuntimeError(f'Invalid target_slot_id: {self.target_slot_id}')

        self.target_pose = make_target_pose(self.slot)
        self.staging_pose = None

        self.state = 'WAIT_FOR_POSE'
        self.done = False

        self.approach_dist_thresh = 0.8
        self.approach_yaw_thresh = 10.0
        self.park_dist_thresh = 0.5
        self.park_yaw_thresh = 12.0

        self.recovery_start_x: Optional[float] = None
        self.recovery_start_y: Optional[float] = None

        self.last_block_log_time = 0.0

        self.front_block_counter = 0
        self.rear_block_counter = 0

        if self.debug_box_counts_enabled:
            self.debug_timer = self.create_timer(1.0, self.debug_box_counts)

        self.timer = self.create_timer(1.0 / self.control_rate, self.control_loop)

        self.get_logger().info(f'Parking controller started for slot {self.target_slot_id}')


    def pose_callback(self, msg: PoseStamped):
        self.ego_ref_x = float(msg.pose.position.x)
        self.ego_ref_y = float(msg.pose.position.y)
        self.ego_ref_z = float(msg.pose.position.z)

        q = msg.pose.orientation
        self.ego_yaw = quaternion_to_yaw_deg(q.x, q.y, q.z, q.w)

        yaw_rad = math.radians(self.ego_yaw)
        fx = math.cos(yaw_rad)
        fy = math.sin(yaw_rad)

        self.ego_x = self.ego_ref_x + self.com_offset * fx
        self.ego_y = self.ego_ref_y + self.com_offset * fy
        self.ego_z = self.ego_ref_z

        if self.state == 'WAIT_FOR_POSE':
            self.staging_pose = make_staging_pose(self.slot, ego_y=self.ego_y)
            self.state = 'GO_TO_STAGING'
            self.get_logger().info(
                f'Staging pose set to x={self.staging_pose["x"]:.2f}, '
                f'y={self.staging_pose["y"]:.2f}, yaw={self.staging_pose["yaw"]:.1f}'
            )

    def speed_callback(self, msg: Float32):
        self.ego_speed = float(msg.data)

    def lidar_callback(self, msg: PointCloud2):
        try:
            raw_points = list(point_cloud2.read_points(
                msg,
                field_names=('x', 'y', 'z', 'intensity'),
                skip_nans=True
            ))

            if len(raw_points) == 0:
                self.latest_lidar_points_vehicle = None
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

            self.latest_lidar_points_vehicle = self.transform_points_to_vehicle_frame(pts_sensor)

        except Exception as e:
            self.get_logger().warn(f'Lidar callback failed: {e}')
            self.latest_lidar_points_vehicle = None

    def urdf_pose_to_vehicle_frame(self, xyz, rpy):
        x_u, y_u, z_u = xyz
        roll_u, pitch_u, yaw_u = rpy

        x_v = x_u
        y_v = -y_u
        z_v = z_u

        roll_v = roll_u
        pitch_v = pitch_u
        yaw_v = -yaw_u

        return (x_v, y_v, z_v), (roll_v, pitch_v, yaw_v)

    def transform_points_to_vehicle_frame(self, pts: np.ndarray) -> np.ndarray:
        (tx, ty, tz), (_, _, yaw) = self.urdf_pose_to_vehicle_frame(LIDAR_XYZ, LIDAR_RPY)

        forward = np.array([np.cos(yaw), np.sin(yaw)], dtype=np.float32)
        right = np.array([-np.sin(yaw), np.cos(yaw)], dtype=np.float32)

        xs = pts[:, 0]
        ys = pts[:, 1]
        zs = pts[:, 2]
        intensity = pts[:, 3]

        xy_vehicle = np.outer(xs, forward) + np.outer(ys, right)

        x_vehicle = xy_vehicle[:, 0] + tx
        y_vehicle = xy_vehicle[:, 1] + ty
        z_vehicle = zs + tz

        return np.stack(
            [x_vehicle, y_vehicle, z_vehicle, intensity],
            axis=1
        ).astype(np.float32)




    def count_points_in_box(self, x_min, x_max, y_min, y_max) -> int:
        pts = self.latest_lidar_points_vehicle
        if pts is None or len(pts) == 0:
            return 0

        x = pts[:, 0]
        y = pts[:, 1]

        mask = (x >= x_min) & (x <= x_max) & (y >= y_min) & (y <= y_max)
        return int(np.count_nonzero(mask))

    def get_front_count(self) -> int:
        return self.count_points_in_box(
            x_min=VEH_X_MAX,
            x_max=VEH_X_MAX + 2.2,
            y_min=-3.0,
            y_max=3.0
        )

    def get_rear_count(self) -> int:
        return self.count_points_in_box(
            x_min=VEH_X_MIN - 2.2,
            x_max=VEH_X_MIN,
            y_min=-3.0,
            y_max=3.0
        )



    def get_front_right_count(self) -> int:
        return self.count_points_in_box(
            x_min=2.8,
            x_max=5.0,
            y_min=2.6,
            y_max=3.8
        )

    def get_front_left_count(self) -> int:
        return self.count_points_in_box(
            x_min=2.8,
            x_max=5.0,
            y_min=-3.8,
            y_max=-2.6
        )

    def get_rear_right_count(self) -> int:
        return self.count_points_in_box(
            x_min=-5.5,
            x_max=-2.0,
            y_min=2.6,
            y_max=3.8
        )

    def get_rear_left_count(self) -> int:
        return self.count_points_in_box(
            x_min=-5.5,
            x_max=-2.0,
            y_min=-3.8,
            y_max=-2.6
        )

    def is_front_hard_blocked(self) -> bool:
        return self.get_front_count() >= self.front_stop_min_points

    def is_rear_hard_blocked(self) -> bool:
        return self.get_rear_count() >= self.rear_stop_min_points

    def is_front_caution(self) -> bool:
        fl = self.get_front_left_count()
        fr = self.get_front_right_count()
        return (
            fl >= self.front_corner_caution_min_points or
            fr >= self.front_corner_caution_min_points
        )

    def is_rear_caution(self) -> bool:
        rl = self.get_rear_left_count()
        rr = self.get_rear_right_count()
        return (
            rl >= self.rear_corner_caution_min_points or
            rr >= self.rear_corner_caution_min_points
        )

    def log_block(self, text: str):
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_block_log_time > 1.0:
            self.get_logger().warn(text)
            self.last_block_log_time = now

    def debug_box_counts(self):
        front = self.get_front_count()
        rear = self.get_rear_count()
        fl = self.get_front_left_count()
        fr = self.get_front_right_count()
        rl = self.get_rear_left_count()
        rr = self.get_rear_right_count()

        self.get_logger().info(
            f'front:{front} rear:{rear} fl:{fl} fr:{fr} rl:{rl} rr:{rr}'
        )


    def publish_control(self, throttle=0.0, steer=0.0, brake=0.0, reverse=False, hand_brake=False):
        msg = VehicleControl()
        msg.throttle = float(throttle)
        msg.steer = float(steer)
        msg.brake = float(brake)
        msg.reverse = bool(reverse)
        msg.hand_brake = bool(hand_brake)
        msg.manual_gear_shift = False
        msg.gear = -1 if reverse else 0
        self.control_pub.publish(msg)




    def control_loop(self):
        if self.done:
            self.publish_control(brake=1.0, hand_brake=True)
            return

        if self.ego_x is None or self.ego_y is None or self.ego_yaw is None:
            return

        if self.state == 'WAIT_FOR_POSE':
            return

        if self.state == 'GO_TO_STAGING':
            if self.is_front_hard_blocked():
                self.front_block_counter += 1
            else:
                self.front_block_counter = 0

            if self.front_block_counter >= self.block_confirm_frames:
                self.publish_control(brake=1.0)
                self.log_block('Front hard-blocked while going to staging')
                return

            throttle, steer, brake, reached = self.drive_forward_to_pose(
                goal_x=self.staging_pose["x"],
                goal_y=self.staging_pose["y"],
                goal_yaw=self.staging_pose["yaw"]
            )

            if self.is_front_caution():
                throttle = min(throttle, 0.08)

            throttle = min(throttle, 0.30)

            self.publish_control(
                throttle=throttle,
                steer=steer,
                brake=brake,
                reverse=False
            )

            if reached:
                self.state = 'REVERSE_IN'
                self.front_block_counter = 0
                self.get_logger().info('Reached staging pose -> REVERSE_IN')

        elif self.state == 'REVERSE_IN':
            if self.is_rear_hard_blocked():
                self.rear_block_counter += 1
            else:
                self.rear_block_counter = 0

            if self.rear_block_counter >= self.block_confirm_frames:
                self.publish_control(brake=1.0)
                self.recovery_start_x = self.ego_x
                self.recovery_start_y = self.ego_y
                self.state = 'RECOVER_FORWARD'
                self.rear_block_counter = 0
                self.get_logger().warn('Rear hard-blocked -> RECOVER_FORWARD')
                return

            throttle, steer, brake, parked = self.reverse_to_pose(
                goal_x=self.target_pose["x"],
                goal_y=self.target_pose["y"],
                goal_yaw=self.target_pose["yaw"]
            )

            if self.is_rear_caution():
                throttle = min(throttle, 0.08)

            throttle = min(throttle, 0.30)

            self.publish_control(
                throttle=throttle,
                steer=steer,
                brake=brake,
                reverse=True
            )

            if parked:
                self.state = 'STOP'
                self.get_logger().info('Reached target pose -> STOP')

        elif self.state == 'RECOVER_FORWARD':
            if self.recovery_start_x is None or self.recovery_start_y is None:
                self.state = 'STOP'
                return

            if self.is_front_hard_blocked():
                self.front_block_counter += 1
            else:
                self.front_block_counter = 0

            if self.front_block_counter >= self.block_confirm_frames:
                self.publish_control(brake=1.0)
                self.log_block('Front hard-blocked during RECOVER_FORWARD')
                return

            travelled = math.sqrt(
                (self.ego_x - self.recovery_start_x) ** 2 +
                (self.ego_y - self.recovery_start_y) ** 2
            )

            if travelled >= self.recover_forward_distance:
                self.publish_control(brake=1.0)
                self.state = 'REVERSE_IN'
                self.front_block_counter = 0
                self.get_logger().info('Recovery finished -> REVERSE_IN')
                return

            throttle = 0.12
            if self.is_front_caution():
                throttle = 0.08

            throttle = min(throttle, 0.30)

            self.publish_control(
                throttle=throttle,
                steer=0.0,
                brake=0.0,
                reverse=False
            )

        elif self.state == 'STOP':
            if self.ego_speed < 0.05:
                self.publish_control(brake=1.0, hand_brake=True)
                self.state = 'DONE'
                self.done = True
                self.get_logger().info('Parking completed')
            else:
                self.publish_control(brake=1.0)




    def drive_forward_to_pose(self, goal_x, goal_y, goal_yaw):
        dx = goal_x - self.ego_x
        dy = goal_y - self.ego_y
        dist = math.sqrt(dx * dx + dy * dy)

        desired_yaw = math.degrees(math.atan2(dy, dx))
        yaw_err = normalize_angle_deg(desired_yaw - self.ego_yaw)
        final_yaw_err = normalize_angle_deg(goal_yaw - self.ego_yaw)

        steer = clamp(0.03 * yaw_err, -0.6, 0.6)

        if dist > 3.0:
            throttle = 0.30
        elif dist > 1.0:
            throttle = 0.20
        else:
            throttle = 0.10

        reached = (
            dist < self.approach_dist_thresh and
            abs(final_yaw_err) < self.approach_yaw_thresh
        )

        if reached:
            return 0.0, 0.0, 1.0, True

        return throttle, steer, 0.0, False

    def reverse_to_pose(self, goal_x, goal_y, goal_yaw):
        dx = goal_x - self.ego_x
        dy = goal_y - self.ego_y
        dist = math.sqrt(dx * dx + dy * dy)

        desired_yaw = math.degrees(math.atan2(dy, dx))
        yaw_err = normalize_angle_deg(desired_yaw - self.ego_yaw)
        final_yaw_err = normalize_angle_deg(goal_yaw - self.ego_yaw)

        steer = clamp(0.003 * yaw_err, -0.7, 0.7)

        parked = (
            dist < self.park_dist_thresh and
            abs(final_yaw_err) < self.park_yaw_thresh
        )

        if parked:
            return 0.0, 0.0, 1.0, True

        if dist > 2.0:
            throttle = 0.20
        elif dist > 0.8:
            throttle = 0.10
        else:
            throttle = 0.08

        return throttle, steer, 0.0, False


def main(args=None):
    rclpy.init(args=args)
    node = ParkingControllerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.publish_control(brake=1.0, hand_brake=True)
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()