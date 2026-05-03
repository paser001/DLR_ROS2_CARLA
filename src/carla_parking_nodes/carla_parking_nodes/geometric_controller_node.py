#!/usr/bin/env python3

import math
import numpy as np
import matplotlib.pyplot as plt
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32, UInt32
from geometry_msgs.msg import PoseStamped

from carla_parking_msgs.msg import VehicleControl

from carla_parking_nodes.reeds_shepp_path_planning import reeds_shepp_path_planning


GOAL_X = 289.0
GOAL_Y = -201.2
GOAL_YAW_DEG = 180.0


def wrap_to_pi(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class GeometricDockingControllerNode(Node):
    def __init__(self):
        super().__init__('geometric_docking_controller_node')

        self.declare_parameter('wheelbase', 2.7)
        self.declare_parameter('min_turning_radius', 8.5)
        self.declare_parameter('step_size', 0.1)
        self.declare_parameter('lookahead_distance', 2.5)
        self.declare_parameter('max_steer_angle_rad', 0.5185)

        self.declare_parameter('target_speed_forward', 1.0)
        self.declare_parameter('target_speed_reverse', -0.8)
        self.declare_parameter('kp_speed', 0.9)
        self.declare_parameter('kp_brake', 0.2)
        self.declare_parameter('max_throttle', 0.35)
        self.declare_parameter('max_brake', 0.8)

        self.declare_parameter('position_tolerance', 0.5)
        self.declare_parameter('yaw_tolerance_deg', 5.0)
        self.declare_parameter('control_rate_hz', 30.0)

        self.declare_parameter('debug_plot', True)
        self.declare_parameter('debug_plot_rate_hz', 5.0)

        self.debug_plot = bool(self.get_parameter('debug_plot').value)
        self.debug_plot_rate_hz = float(self.get_parameter('debug_plot_rate_hz').value)


        self.wheelbase = float(self.get_parameter('wheelbase').value)
        self.min_turning_radius = float(self.get_parameter('min_turning_radius').value)
        self.step_size = float(self.get_parameter('step_size').value)
        self.lookahead_distance = float(self.get_parameter('lookahead_distance').value)
        self.max_steer_angle = float(self.get_parameter('max_steer_angle_rad').value)

        self.target_speed_forward = float(self.get_parameter('target_speed_forward').value)
        self.target_speed_reverse = float(self.get_parameter('target_speed_reverse').value)
        self.kp_speed = float(self.get_parameter('kp_speed').value)
        self.kp_brake = float(self.get_parameter('kp_brake').value)
        self.max_throttle = float(self.get_parameter('max_throttle').value)
        self.max_brake = float(self.get_parameter('max_brake').value)

        self.position_tolerance = float(self.get_parameter('position_tolerance').value)
        self.yaw_tolerance = math.radians(
            float(self.get_parameter('yaw_tolerance_deg').value)
        )

        control_rate = float(self.get_parameter('control_rate_hz').value)

        # -----------------------------
        # ROS interfaces
        # -----------------------------
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/ego/pose',
            self.pose_cb,
            10
        )

        self.yaw_sub = self.create_subscription(
            Float32,
            '/ego/yaw',
            self.yaw_cb,
            10
        )

        self.speed_sub = self.create_subscription(
            Float32,
            '/ego/speed',
            self.speed_cb,
            10
        )

        self.reset_sub = self.create_subscription(
            UInt32,
            '/episode/reset',
            self.reset_cb,
            10
        )

        self.control_pub = self.create_publisher(
            VehicleControl,
            '/vehicle/control_cmd',
            10
        )

        # -----------------------------
        # State
        # -----------------------------
        self.x = None
        self.y = None
        self.yaw = None
        self.speed = 0.0

        self.path_x = None
        self.path_y = None
        self.path_yaw = None
        self.path_directions = None
        self.path_modes = None
        self.path_lengths = None

        self.planned = False
        self.finished = False
        self.last_target_index = 0

        self.debug_fig = None
        self.debug_ax = None
        self.last_debug_plot_time = 0.0

        self.last_replan_time = 0.0
        self.replan_interval = 1.5
        self.replan_error_threshold = 0.8

        self.timer = self.create_timer(
            1.0 / control_rate,
            self.control_loop
        )

        self.get_logger().info('Geometric docking controller started')

    # --------------------------------------------------
    # Callbacks
    # --------------------------------------------------
    def pose_cb(self, msg: PoseStamped):
        self.x = float(msg.pose.position.x)
        self.y = float(msg.pose.position.y)

    def yaw_cb(self, msg: Float32):
        self.yaw = math.radians(float(msg.data))

    def speed_cb(self, msg: Float32):
        self.speed = float(msg.data)

    def reset_cb(self, msg: UInt32):
        self.get_logger().info(f'Received episode reset {msg.data}. Replanning.')
        self.planned = False
        self.finished = False
        self.last_target_index = 0

    # --------------------------------------------------
    # Planning
    # --------------------------------------------------
    def have_pose(self):
        return self.x is not None and self.y is not None and self.yaw is not None

    def plan_path(self):
        if not self.have_pose():
            return False

        max_curvature = 1.0 / self.min_turning_radius

        gx = GOAL_X
        gy = GOAL_Y
        gyaw = math.radians(GOAL_YAW_DEG)

        sx, sy, _ = self.get_tracking_point(reverse=True)

        result = reeds_shepp_path_planning(
            sx,
            sy,
            self.yaw,
            gx,
            gy,
            gyaw,
            max_curvature,
            self.step_size
        )

        xs, ys, yaws, modes, lengths, directions = result

        if xs is None:
            self.get_logger().warn('Reeds-Shepp planner failed to find a path')
            return False

        self.path_x = np.array(xs)
        self.path_y = np.array(ys)
        self.path_yaw = np.array(yaws)
        self.path_directions = np.array(directions)
        self.path_modes = modes
        self.path_lengths = lengths

        self.planned = True
        self.finished = False
        self.last_target_index = 0

        self.get_logger().info(
            f'Planned path with {len(self.path_x)} points, '
            f'modes={modes}, lengths={["%.2f" % l for l in lengths]}'
        )
        self.plot_current_path()

        return True

    # --------------------------------------------------
    # Path tracking helpers
    # --------------------------------------------------
    def plot_current_path(self):
        if not self.debug_plot:
            return

        if self.path_x is None:
            return

        plt.ion()

        self.debug_fig, self.debug_ax = plt.subplots()

        self.debug_ax.set_title("Geometric Controller Debug")
        self.debug_ax.set_xlabel("x [m]")
        self.debug_ax.set_ylabel("y [m]")
        self.debug_ax.grid(True)
        self.debug_ax.axis("equal")
        self.debug_ax.invert_yaxis()

        self.update_debug_plot(
            closest_index=None,
            target_index=None,
            reverse=False,
            steer=0.0
        )

        self.debug_fig.canvas.draw()
        self.debug_fig.canvas.flush_events()
        plt.pause(0.001)

    def current_path_error(self, reverse):
        if self.path_x is None:
            return float("inf")

        tx, ty, _ = self.get_tracking_point(reverse)

        dx = self.path_x - tx
        dy = self.path_y - ty
        distances = np.hypot(dx, dy)

        return float(np.min(distances))
    
    def maybe_replan(self, reverse):
        now = time.time()

        if now - self.last_replan_time < self.replan_interval:
            return False

        error = self.current_path_error(reverse)

        if error > self.replan_error_threshold:
            self.get_logger().warn(
                f"Replanning: path error={error:.2f} m"
            )
            self.last_replan_time = now
            self.planned = False
            self.last_target_index = 0
            return True

        return False

    def update_debug_plot(self, closest_index=None, target_index=None, reverse=False, steer=0.0):
        if not self.debug_plot:
            return

        if self.path_x is None or self.x is None or self.y is None or self.yaw is None:
            return

        now = time.time()
        min_period = 1.0 / max(self.debug_plot_rate_hz, 1e-6)

        if now - self.last_debug_plot_time < min_period:
            return

        self.last_debug_plot_time = now

        if self.debug_fig is None or self.debug_ax is None:
            plt.ion()
            self.debug_fig, self.debug_ax = plt.subplots()

        ax = self.debug_ax
        ax.clear()

        directions = np.array(self.path_directions)
        forward = directions > 0
        reverse_mask = directions < 0

        # Full path
        ax.plot(self.path_x[forward], self.path_y[forward], ".b", markersize=4, label="path forward")
        ax.plot(self.path_x[reverse_mask], self.path_y[reverse_mask], ".r", markersize=4, label="path reverse")

        # Start/current planned first point
        ax.plot(self.path_x[0], self.path_y[0], "go", markersize=8, label="path start")

        # Pre-dock goal
        goal_yaw = math.radians(GOAL_YAW_DEG)
        ax.plot(GOAL_X, GOAL_Y, "ko", markersize=8, label="goal")
        ax.arrow(
            GOAL_X, GOAL_Y,
            0.8 * math.cos(goal_yaw),
            0.8 * math.sin(goal_yaw),
            head_width=0.25,
            head_length=0.3,
            fc="k",
            ec="k"
        )

        # Current vehicle actor pose
        ax.plot(self.x, self.y, "mo", markersize=8, label="actor pose")
        ax.arrow(
            self.x, self.y,
            0.9 * math.cos(self.yaw),
            0.9 * math.sin(self.yaw),
            head_width=0.25,
            head_length=0.3,
            fc="m",
            ec="m"
        )

        # Tracking point actually used by Pure Pursuit
        try:
            tx, ty, direction = self.get_tracking_point(reverse)
            ax.plot(tx, ty, "co", markersize=8, label="tracking point")
        except Exception:
            tx, ty = self.x, self.y

        # Closest point
        if closest_index is not None:
            closest_index = int(np.clip(closest_index, 0, len(self.path_x) - 1))
            ax.plot(
                self.path_x[closest_index],
                self.path_y[closest_index],
                "yo",
                markersize=10,
                label=f"closest idx {closest_index}"
            )

        # Lookahead target point
        if target_index is not None:
            target_index = int(np.clip(target_index, 0, len(self.path_x) - 1))
            target_x = self.path_x[target_index]
            target_y = self.path_y[target_index]

            ax.plot(
                target_x,
                target_y,
                "gx",
                markersize=12,
                markeredgewidth=3,
                label=f"target idx {target_index}"
            )

            # Line from tracking point to target
            ax.plot([tx, target_x], [ty, target_y], "g--", linewidth=1.5)

        ax.set_title(
            f"Pure Pursuit Debug | reverse={reverse} | steer={steer:.3f}"
        )
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.grid(True)
        ax.axis("equal")
        ax.invert_yaxis()
        ax.legend(loc="best")

        self.debug_fig.canvas.draw_idle()
        self.debug_fig.canvas.flush_events()
        plt.pause(0.001)

    def find_closest_index(self, reverse):
        tx, ty, _ = self.get_tracking_point(reverse)

        dx = self.path_x - tx
        dy = self.path_y - ty
        distances = np.hypot(dx, dy)

        return int(np.argmin(distances))

    def get_lookahead_index(self, start_index, reverse):
        tx, ty, _ = self.get_tracking_point(reverse)
        index = max(start_index, self.last_target_index)

        while index < len(self.path_x) - 1:
            dx = self.path_x[index] - tx
            dy = self.path_y[index] - ty

            if math.hypot(dx, dy) >= self.lookahead_distance:
                break

            index += 1

        return index
        
    def get_tracking_point(self, reverse):
        com_offset = -0.4

        # CARLA actor pose -> vehicle center/COM-corrected point
        cx = self.x + com_offset * math.cos(self.yaw)
        cy = self.y + com_offset * math.sin(self.yaw)

        direction = -1 if reverse else 1

        # Same logic as original Pure Pursuit:
        # forward: rear axle
        # reverse: front axle
        tx = cx - direction * (self.wheelbase / 2.0) * math.cos(self.yaw)
        ty = cy - direction * (self.wheelbase / 2.0) * math.sin(self.yaw)

        return tx, ty, direction

    def compute_pure_pursuit_steering(self, target_x, target_y, reverse):
        tx, ty, direction = self.get_tracking_point(reverse)

        alpha = math.atan2(target_y - ty, target_x - tx) - self.yaw
        alpha = wrap_to_pi(alpha)


        delta = -direction * math.atan2(
            2.0 * self.wheelbase * math.sin(alpha),
            self.lookahead_distance
        )

        steer_cmd = delta / self.max_steer_angle
        return float(np.clip(steer_cmd, -1.0, 1.0))

    def goal_reached(self):
        dx = GOAL_X - self.x
        dy = GOAL_Y - self.y
        dist = math.hypot(dx, dy)

        goal_yaw = math.radians(GOAL_YAW_DEG)
        yaw_error = wrap_to_pi(goal_yaw - self.yaw)

        return dist < self.position_tolerance and abs(yaw_error) < self.yaw_tolerance

    # --------------------------------------------------
    # Speed control
    # --------------------------------------------------
    def compute_speed_control(self, target_speed):
        speed_error = target_speed - self.speed

        if target_speed >= 0.0:
            # Forward driving
            if speed_error > 0.0:
                throttle = self.kp_speed * speed_error
                brake = 0.0
            else:
                throttle = 0.0
                brake = self.kp_brake * abs(speed_error)
        else:
            # Reverse driving: CARLA still needs positive throttle
            if speed_error < 0.0:
                throttle = self.kp_speed * abs(speed_error)
                brake = 0.0
            else:
                throttle = 0.0
                brake = self.kp_brake * abs(speed_error)

        throttle = float(np.clip(throttle, 0.0, self.max_throttle))
        brake = float(np.clip(brake, 0.0, self.max_brake))

        return throttle, brake, speed_error


    # --------------------------------------------------
    # Main loop
    # --------------------------------------------------
    def control_loop(self):
        if not self.have_pose():
            return

        if self.finished:
            self.publish_stop()
            return

        if not self.planned:
            ok = self.plan_path()
            if not ok:
                self.publish_stop()
                return

        if self.goal_reached():
            self.get_logger().info('Docking goal reached')
            self.finished = True
            self.publish_stop()
            return


        direction_guess = int(self.path_directions[self.last_target_index])
        reverse_guess = direction_guess < 0

        closest_index = self.find_closest_index(reverse_guess)
        target_index = self.get_lookahead_index(closest_index, reverse_guess)
        self.last_target_index = target_index

        target_x = float(self.path_x[target_index])
        target_y = float(self.path_y[target_index])

        direction = int(self.path_directions[target_index])
        reverse = direction < 0

        if self.maybe_replan(reverse):
            self.publish_stop()
            return

        steer = self.compute_pure_pursuit_steering(
            target_x,
            target_y,
            reverse
        )
        if abs(steer) > 0.8:
            target_speed = -0.4 if reverse else 0.4
        else:
            target_speed = self.target_speed_reverse if reverse else self.target_speed_forward
        throttle, brake, speed_error = self.compute_speed_control(target_speed)




        self.get_logger().info(
            f"idx closest={closest_index} target={target_index} "
            f"reverse={reverse} steer={steer:.2f} speed={self.speed:.2f} speed_error={speed_error:.2f}"
        )
        self.update_debug_plot(
            closest_index=closest_index,
            target_index=target_index,
            reverse=reverse,
            steer=steer
        )


        
        msg = VehicleControl()
        msg.throttle = throttle
        msg.steer = steer
        msg.brake = brake
        msg.reverse = bool(reverse)
        msg.hand_brake = False
        msg.manual_gear_shift = False
        msg.gear = -1 if reverse else 1

        self.control_pub.publish(msg)

    def publish_stop(self):
        msg = VehicleControl()
        msg.throttle = 0.0
        msg.steer = 0.0
        msg.brake = 0.8
        msg.reverse = True
        msg.hand_brake = False
        msg.manual_gear_shift = False
        msg.gear = -1
        self.control_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GeometricDockingControllerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()