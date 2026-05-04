#!/usr/bin/env python3
"""
Geometric docking controller using Reeds-Shepp planning + Stanley tracking.

Frame conventions
-----------------
CARLA publishes pose/yaw in a LEFT-HANDED frame (yaw decreases on left turns).
All planning and tracking math is done in a standard RIGHT-HANDED math frame.
Conversion happens at the input (pose callbacks) and output (steering command):

    Input:   x_math = x_carla,  y_math = -y_carla,  yaw_math = -yaw_carla
    Output:  steer_carla = -delta_math / max_steer_angle

Goal poses below are stored in CARLA frame (matching what you read off the sim)
and converted internally.

Tracking reference point
------------------------
For a bicycle model:
  - Forward driving:  rear axle is the tracked point (kinematic constraint).
  - Reverse driving:  front axle is the tracked point (the "leading" axle).

The actor pose is assumed to be at the geometric center of the vehicle.
VERIFY this in your CARLA interface node using:
    vehicle.get_physics_control().wheels  # gives wheel world positions
and adjust ACTOR_TO_REAR_AXLE if the actor pose is not at center.
"""

import math
import time

import numpy as np
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32, UInt32
from geometry_msgs.msg import PoseStamped

from carla_parking_msgs.msg import VehicleControl

from carla_parking_nodes.reeds_shepp_path_planning import reeds_shepp_path_planning


# ---------------------------------------------------------------------------
# Goal poses (in CARLA frame, as you read them off the simulator)
# ---------------------------------------------------------------------------
# Pre-dock pose: a few meters in front of the cargo box, aligned with dock heading.
PREDOCK_X_CARLA = 289.0
PREDOCK_Y_CARLA = -201.2
PREDOCK_YAW_DEG_CARLA = 180.0

# Final cargo box pose
CARGOBOX_X_CARLA = 290.9
CARGOBOX_Y_CARLA = -201.2
CARGOBOX_YAW_DEG_CARLA = 180.0


# ---------------------------------------------------------------------------
# Frame conversion helpers
# ---------------------------------------------------------------------------
def carla_to_math(x_c, y_c, yaw_c_rad):
    """CARLA (left-handed) -> math (right-handed)."""
    return x_c, -y_c, -yaw_c_rad


def math_to_carla_steer(delta_math, max_steer_angle):
    """Steering angle in math frame -> normalized CARLA steer command."""
    steer = -delta_math / max_steer_angle
    return float(np.clip(steer, -1.0, 1.0))


def wrap_to_pi(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


# ---------------------------------------------------------------------------
# Controller node
# ---------------------------------------------------------------------------
class GeometricDockingControllerNode(Node):
    def __init__(self):
        super().__init__('geometric_docking_controller_node')

        # --- Vehicle / planner parameters ---
        self.declare_parameter('wheelbase', 2.7)
        # Asymmetric axle offsets measured from CARLA wheel positions:
        #   actor pose is 1.98 m ahead of rear axle, 0.72 m behind front axle.
        # (forklift body has cab forward; actor frame sits near front axle.)
        self.declare_parameter('actor_to_rear_axle', 1.98)
        self.declare_parameter('actor_to_front_axle', 0.72)
        self.declare_parameter('min_turning_radius', 8.5)
        self.declare_parameter('step_size', 0.1)
        self.declare_parameter('max_steer_angle_rad', 0.5185)

        # --- Stanley parameters ---
        # Tuned for the working configuration (heading-flip-for-reverse):
        #   - stanley_k controls cross-track correction strength. Lower = gentler.
        #   - stanley_v_floor floors the velocity in atan2(k*cte, v). Higher
        #     means gentler CTE response at low speeds.
        #   - stanley_k_heading weights the heading-alignment term. Heading
        #     error is small once tracking converges, so this can be ~1.
        self.declare_parameter('stanley_k', 0.5)
        self.declare_parameter('stanley_v_floor', 1.5)
        self.declare_parameter('stanley_k_heading', 1.0)

        # --- Speed control parameters ---
        self.declare_parameter('target_speed_forward', 2.0)
        self.declare_parameter('target_speed_reverse', -1.5)
        self.declare_parameter('target_speed_final_approach', -0.5)
        self.declare_parameter('kp_speed', 1.2)
        self.declare_parameter('kp_brake', 0.3)
        self.declare_parameter('max_throttle', 0.6)
        self.declare_parameter('max_brake', 0.8)
        # Slow-down threshold: only reduce speed for *very* hard steering.
        self.declare_parameter('hard_steer_threshold', 0.85)
        self.declare_parameter('hard_steer_speed_forward', 1.0)
        self.declare_parameter('hard_steer_speed_reverse', -0.8)

        # --- Goal / mode-switch tolerances ---
        # Pre-dock is a path-tracking waypoint. Tight YAW tolerance because
        # any heading error at predock compounds into lateral drift during
        # the straight-line final approach. Position can be looser.
        self.declare_parameter('predock_position_tolerance', 0.5)
        self.declare_parameter('predock_yaw_tolerance_deg', 2.5)
        # Final docked-pose check.
        # The reset node uses pos_tol = 1.5m and yaw_tol = 5deg on actor pose.
        # The controller should aim for the CENTER of that envelope, not the
        # edge — otherwise "done" fires the moment the actor pose enters the
        # 1.5m circle, which happens well before docking is complete.
        # We check longitudinal and lateral separately:
        #   - longitudinal_tol: how close the actor pose must be to the cargo
        #     box pose ALONG the dock heading. Tight, because this is "depth".
        #   - lateral_tol: how close perpendicular to the dock heading.
        self.declare_parameter('final_longitudinal_tolerance', 0.30)
        self.declare_parameter('final_lateral_tolerance', 0.30)
        self.declare_parameter('final_yaw_tolerance_deg', 2.0)

        # --- Replan parameters ---
        # Replan should be rare. With a working tracker, path error stays
        # under ~30 cm. Threshold of 1.5 m means we only replan when something
        # has genuinely gone wrong (vehicle bumped, infeasible path, etc.).
        self.declare_parameter('replan_interval_s', 5.0)
        self.declare_parameter('replan_error_threshold_m', 1.5)
        self.declare_parameter('max_replans_per_episode', 3)
        # Reeds-Shepp sometimes outputs tiny terminal segments (e.g. 0.26 m)
        # as numerical artifacts. Segments shorter than this are dropped from
        # the segment list — they're not meaningfully trackable and cause
        # the controller to thrash trying to "track" them.
        self.declare_parameter('min_segment_length_m', 0.5)

        # --- Loop / debug ---
        self.declare_parameter('control_rate_hz', 30.0)
        self.declare_parameter('debug_plot', True)
        self.declare_parameter('debug_plot_rate_hz', 5.0)

        # Pull values
        gp = self.get_parameter
        self.wheelbase = float(gp('wheelbase').value)
        self.actor_to_rear_axle = float(gp('actor_to_rear_axle').value)
        self.actor_to_front_axle = float(gp('actor_to_front_axle').value)
        self.min_turning_radius = float(gp('min_turning_radius').value)
        self.step_size = float(gp('step_size').value)
        self.max_steer_angle = float(gp('max_steer_angle_rad').value)

        self.stanley_k = float(gp('stanley_k').value)
        self.stanley_v_floor = float(gp('stanley_v_floor').value)
        self.stanley_k_heading = float(gp('stanley_k_heading').value)

        self.target_speed_forward = float(gp('target_speed_forward').value)
        self.target_speed_reverse = float(gp('target_speed_reverse').value)
        self.target_speed_final_approach = float(gp('target_speed_final_approach').value)
        self.kp_speed = float(gp('kp_speed').value)
        self.kp_brake = float(gp('kp_brake').value)
        self.max_throttle = float(gp('max_throttle').value)
        self.max_brake = float(gp('max_brake').value)
        self.hard_steer_threshold = float(gp('hard_steer_threshold').value)
        self.hard_steer_speed_forward = float(gp('hard_steer_speed_forward').value)
        self.hard_steer_speed_reverse = float(gp('hard_steer_speed_reverse').value)

        self.predock_pos_tol = float(gp('predock_position_tolerance').value)
        self.predock_yaw_tol = math.radians(float(gp('predock_yaw_tolerance_deg').value))
        self.final_long_tol = float(gp('final_longitudinal_tolerance').value)
        self.final_lat_tol = float(gp('final_lateral_tolerance').value)
        self.final_yaw_tol = math.radians(float(gp('final_yaw_tolerance_deg').value))

        self.replan_interval = float(gp('replan_interval_s').value)
        self.replan_error_threshold = float(gp('replan_error_threshold_m').value)
        self.max_replans = int(gp('max_replans_per_episode').value)
        self.min_segment_length = float(gp('min_segment_length_m').value)
        self.replan_count = 0

        self.debug_plot = bool(gp('debug_plot').value)
        self.debug_plot_rate_hz = float(gp('debug_plot_rate_hz').value)
        control_rate = float(gp('control_rate_hz').value)

        # --- Goal poses converted to math frame ---
        self.predock_math = carla_to_math(
            PREDOCK_X_CARLA, PREDOCK_Y_CARLA, math.radians(PREDOCK_YAW_DEG_CARLA)
        )
        self.cargobox_math = carla_to_math(
            CARGOBOX_X_CARLA, CARGOBOX_Y_CARLA, math.radians(CARGOBOX_YAW_DEG_CARLA)
        )

        # --- ROS interfaces ---
        self.pose_sub = self.create_subscription(
            PoseStamped, '/ego/pose', self.pose_cb, 10)
        self.yaw_sub = self.create_subscription(
            Float32, '/ego/yaw', self.yaw_cb, 10)
        self.speed_sub = self.create_subscription(
            Float32, '/ego/speed', self.speed_cb, 10)
        self.reset_sub = self.create_subscription(
            UInt32, '/episode/reset', self.reset_cb, 10)
        self.control_pub = self.create_publisher(
            VehicleControl, '/vehicle/control_cmd', 10)

        # --- State (all in MATH frame) ---
        self.x = None
        self.y = None
        self.yaw = None
        self.speed = 0.0  # signed; sign convention is unchanged from CARLA

        self.path_x = None
        self.path_y = None
        self.path_yaw = None
        self.path_directions = None

        self.planned = False
        self.stage = 'PREDOCK'   # 'PREDOCK' -> 'FINAL_APPROACH' -> 'DONE'
        self.last_replan_time = 0.0

        # Cusp segment tracking for monotonic-within-segment indexing
        self.segment_starts = []  # list of indices where direction changes
        self.current_segment = 0
        self.segment_target_idx = 0  # monotonic only within current segment

        # Debug plot
        self.debug_fig = None
        self.debug_ax = None
        self.last_debug_plot_time = 0.0

        self.timer = self.create_timer(1.0 / control_rate, self.control_loop)
        self.get_logger().info('Geometric docking controller (Stanley) started')

    # -----------------------------------------------------------------------
    # Callbacks (convert CARLA -> math frame on entry)
    # -----------------------------------------------------------------------
    def pose_cb(self, msg: PoseStamped):
        self.x = float(msg.pose.position.x)
        # y converted when yaw arrives, since both must be consistent
        self._y_carla = float(msg.pose.position.y)
        self.y = -self._y_carla

    def yaw_cb(self, msg: Float32):
        # CARLA yaw is left-handed; flip sign for math frame
        self.yaw = -math.radians(float(msg.data))

    def speed_cb(self, msg: Float32):
        self.speed = float(msg.data)

    def reset_cb(self, msg: UInt32):
        self.get_logger().info(f'Episode reset {msg.data}: replanning.')
        self.planned = False
        self.stage = 'PREDOCK'
        self.current_segment = 0
        self.segment_target_idx = 0
        self.replan_count = 0

    # -----------------------------------------------------------------------
    # Geometry helpers (math frame)
    # -----------------------------------------------------------------------
    def have_pose(self):
        return self.x is not None and self.y is not None and self.yaw is not None

    def get_axle_position(self, reverse=None):
        """
        Returns the REAR axle position in math frame.

        Rationale: Reeds-Shepp paths are bicycle-model paths describing the
        trajectory of the rear axle. The kinematic constraint for a car-like
        vehicle is "rear axle moves along its heading direction" — this holds
        in both forward and reverse. Therefore the rear axle is the correct
        tracked point regardless of motion direction.

        The `reverse` parameter is accepted for backward compatibility but
        ignored; both forward and reverse segments track the rear axle.

        Rear axle is 1.98 m behind the actor along heading (measured from
        CARLA wheel positions on this forklift).
        """
        ax = self.x - self.actor_to_rear_axle * math.cos(self.yaw)
        ay = self.y - self.actor_to_rear_axle * math.sin(self.yaw)
        return ax, ay

    # -----------------------------------------------------------------------
    # Planning
    # -----------------------------------------------------------------------
    def plan_path(self):
        if not self.have_pose():
            return False

        # Plan from the rear axle (vehicle is currently stationary or about to
        # start; using rear axle keeps the bicycle-model frame consistent with
        # how Reeds-Shepp expects the start pose).
        # We use the rear axle for forward; for reverse we use the front axle.
        # At plan time we don't know yet, so we plan from rear axle and let
        # Stanley handle the offset when reversing — this is the standard
        # approach since Reeds-Shepp paths are bicycle-model paths from the
        # rear axle reference.
        sx, sy = self.get_axle_position(reverse=False)  # rear axle
        syaw = self.yaw

        gx, gy, gyaw = self.predock_math
        max_curvature = 1.0 / self.min_turning_radius

        result = reeds_shepp_path_planning(
            sx, sy, syaw, gx, gy, gyaw, max_curvature, self.step_size)
        xs, ys, yaws, modes, lengths, directions = result

        if xs is None or len(xs) < 2:
            self.get_logger().warn('Reeds-Shepp planner failed.')
            return False

        self.path_x = np.array(xs)
        self.path_y = np.array(ys)
        self.path_yaw = np.array(yaws)
        self.path_directions = np.array(directions)

        # Find segment boundaries (cusps where direction changes)
        raw_segment_starts = [0]
        for i in range(1, len(self.path_directions)):
            if self.path_directions[i] != self.path_directions[i - 1]:
                raw_segment_starts.append(i)
        raw_segment_starts.append(len(self.path_directions))  # sentinel

        # Drop segments shorter than min_segment_length (Reeds-Shepp artifacts).
        # Compute Euclidean arc length of each segment by summing point-to-point
        # distances. Keep segments at or above threshold.
        kept_starts = [raw_segment_starts[0]]
        for i in range(len(raw_segment_starts) - 1):
            seg_s = raw_segment_starts[i]
            seg_e = raw_segment_starts[i + 1]
            if seg_e - seg_s < 2:
                # Single-point segment, definitely drop
                continue
            seg_dx = np.diff(self.path_x[seg_s:seg_e])
            seg_dy = np.diff(self.path_y[seg_s:seg_e])
            arc_len = float(np.sum(np.hypot(seg_dx, seg_dy)))
            if arc_len >= self.min_segment_length:
                if seg_s != kept_starts[-1]:
                    kept_starts.append(seg_s)
            else:
                self.get_logger().info(
                    f'Dropping tiny segment {i} (arc length {arc_len:.2f} m '
                    f'< {self.min_segment_length:.2f} m)')

        # Sentinel = end of path
        if kept_starts[-1] != len(self.path_directions):
            kept_starts.append(len(self.path_directions))

        # Edge case: if we dropped everything except the sentinel, fall back
        # to the original segments (better to track a tiny segment than to
        # have no segments at all).
        if len(kept_starts) < 2:
            self.get_logger().warn(
                'All segments dropped as tiny; reverting to raw segments')
            kept_starts = raw_segment_starts

        self.segment_starts = kept_starts

        self.current_segment = 0
        self.segment_target_idx = self.segment_starts[0]
        self.planned = True

        self.get_logger().info(
            f'Planned: {len(self.path_x)} pts, modes={modes}, '
            f'lengths={["%.2f" % l for l in lengths]}, '
            f'segments={len(self.segment_starts) - 1}'
        )
        return True

    def current_path_error(self, reverse=None):
        """Min distance from rear axle to any point on path."""
        if self.path_x is None:
            return float('inf')
        ax, ay = self.get_axle_position()
        return float(np.min(np.hypot(self.path_x - ax, self.path_y - ay)))

    def maybe_replan(self, reverse):
        if self.replan_count >= self.max_replans:
            return False
        now = time.time()
        if now - self.last_replan_time < self.replan_interval:
            return False
        err = self.current_path_error(reverse)
        if err > self.replan_error_threshold:
            self.get_logger().warn(
                f'Replanning ({self.replan_count + 1}/{self.max_replans}): '
                f'path error {err:.2f} m > threshold {self.replan_error_threshold:.2f} m')
            self.last_replan_time = now
            self.replan_count += 1
            self.planned = False
            return True
        return False

    # -----------------------------------------------------------------------
    # Stanley controller (math frame)
    # -----------------------------------------------------------------------
    def stanley_control(self):
        """
        Compute steering for the current path segment.
        Returns: (delta_math, reverse_flag, segment_done_flag, debug_info)
        """
        seg_start = self.segment_starts[self.current_segment]
        seg_end = self.segment_starts[self.current_segment + 1]  # exclusive
        seg_direction = int(self.path_directions[seg_start])
        reverse = seg_direction < 0

        # Tracked rear axle for this segment (rear axle, regardless of direction)
        ax, ay = self.get_axle_position()

        # --- Find closest point WITHIN the current segment only ---
        seg_x = self.path_x[seg_start:seg_end]
        seg_y = self.path_y[seg_start:seg_end]
        d = np.hypot(seg_x - ax, seg_y - ay)
        local_idx = int(np.argmin(d))

        # Enforce monotonicity within segment (don't go backwards)
        local_min = self.segment_target_idx - seg_start
        if local_idx < local_min:
            local_idx = local_min
        self.segment_target_idx = seg_start + local_idx

        global_idx = seg_start + local_idx
        path_yaw = float(self.path_yaw[global_idx])

        # --- Cross-track error (signed, in vehicle frame) ---
        # Standard Stanley CTE: project (axle - path_point) onto path's left-normal.
        # Path normal pointing "left" of path direction in math frame:
        nx = -math.sin(path_yaw)
        ny = math.cos(path_yaw)
        cte = (ax - self.path_x[global_idx]) * nx + (ay - self.path_y[global_idx]) * ny

        # --- Heading error ---
        # Empirically, this Reeds-Shepp implementation stores path_yaw as the
        # direction of travel, not the vehicle's heading. So when reversing,
        # the vehicle is pointed OPPOSITE to path_yaw, and desired vehicle
        # heading is path_yaw + pi.
        desired_yaw = path_yaw if not reverse else wrap_to_pi(path_yaw + math.pi)
        heading_error = wrap_to_pi(desired_yaw - self.yaw)

        # --- Stanley law ---
        v_eff = max(abs(self.speed), self.stanley_v_floor)
        delta = self.stanley_k_heading * heading_error \
              + math.atan2(self.stanley_k * cte, v_eff)

        # NOTE: We do NOT flip delta as a whole for reverse. Reverse handling
        # is done in the heading-error term: desired_yaw = path_yaw + pi when
        # reversing, so heading_error already points in the correct direction.
        # Flipping delta on top of that would double-correct.

        delta = float(np.clip(delta, -self.max_steer_angle, self.max_steer_angle))

        # --- Segment-done check ---
        # Only advance to the next segment when the rear axle is genuinely
        # close to this segment's end point. The 40cm threshold is small
        # enough that we don't skip ahead, but large enough to handle the
        # vehicle stopping just short due to braking dynamics at cusps.
        seg_done = False
        last_idx_in_seg = seg_end - 1
        end_dx = self.path_x[last_idx_in_seg] - ax
        end_dy = self.path_y[last_idx_in_seg] - ay
        end_dist = math.hypot(end_dx, end_dy)

        # Only fire seg_done if there's actually a NEXT segment to advance to.
        # Otherwise we're on the last segment and should keep tracking until
        # FINAL_APPROACH takes over (via at_predock check in control_loop).
        has_next_segment = self.current_segment < len(self.segment_starts) - 2
        if has_next_segment and end_dist < 0.4:
            seg_done = True

        debug = {
            'global_idx': global_idx,
            'cte': cte,
            'heading_error': heading_error,
            'reverse': reverse,
            'segment': self.current_segment,
        }
        return delta, reverse, seg_done, debug

    # -----------------------------------------------------------------------
    # Final approach: straight-line reverse with heading + lateral correction
    # -----------------------------------------------------------------------
    def final_approach_control(self):
        """
        Pure straight-line reverse into the cargo box pose.
        Active once we're near the pre-dock pose and roughly aligned.
        Returns: (delta_math, done_flag)

        Steering is computed from the rear axle (kinematic reference).
        The DONE check uses ACTOR pose vs cargo box pose, decomposed into
        longitudinal (along dock heading) and lateral components. This avoids
        firing "done" prematurely when the actor pose is just passing through
        the goal circle on approach.
        """
        gx, gy, gyaw = self.cargobox_math
        cos_g, sin_g = math.cos(gyaw), math.sin(gyaw)

        # ---- Steering: from rear axle in goal frame ----
        ax, ay = self.get_axle_position()  # rear axle
        rdx = ax - gx
        rdy = ay - gy
        rear_long = rdx * cos_g + rdy * sin_g
        rear_lat = -rdx * sin_g + rdy * cos_g

        heading_error = wrap_to_pi(gyaw - self.yaw)

        k_h = 0.6
        k_l = 0.15
        delta = k_h * heading_error + k_l * math.atan2(rear_lat, max(abs(rear_long), 0.5))
        delta = float(np.clip(delta, -0.3 * self.max_steer_angle, 0.3 * self.max_steer_angle))

        # ---- Done check: ACTOR pose vs cargo box pose, split by axis ----
        actor_dx = self.x - gx
        actor_dy = self.y - gy
        actor_long = actor_dx * cos_g + actor_dy * sin_g
        actor_lat = -actor_dx * sin_g + actor_dy * cos_g
        yaw_err = abs(heading_error)

        long_ok = abs(actor_long) < self.final_long_tol
        lat_ok = abs(actor_lat) < self.final_lat_tol
        yaw_ok = yaw_err < self.final_yaw_tol
        done = long_ok and lat_ok and yaw_ok

        return delta, done

    # -----------------------------------------------------------------------
    # Speed control (unchanged)
    # -----------------------------------------------------------------------
    def compute_speed_control(self, target_speed):
        speed_error = target_speed - self.speed
        throttle = 0.0
        brake = 0.0

        if target_speed >= 0.0:
            if speed_error > 0.0:
                throttle = self.kp_speed * speed_error
            else:
                brake = self.kp_brake * abs(speed_error)
        else:
            if speed_error < 0.0:
                throttle = self.kp_speed * abs(speed_error)
            else:
                brake = self.kp_brake * abs(speed_error)

        throttle = float(np.clip(throttle, 0.0, self.max_throttle))
        brake = float(np.clip(brake, 0.0, self.max_brake))
        return throttle, brake, speed_error

    # -----------------------------------------------------------------------
    # Stage transitions
    # -----------------------------------------------------------------------
    def at_predock(self):
        """
        Check if the rear axle has reached the pre-dock pose.
        This is a path-tracking waypoint, so we use the rear axle (the tracked
        reference) — not the actor pose.
        """
        gx, gy, gyaw = self.predock_math
        ax, ay = self.get_axle_position()  # rear axle
        dist = math.hypot(ax - gx, ay - gy)
        yaw_err = abs(wrap_to_pi(gyaw - self.yaw))
        return dist < self.predock_pos_tol and yaw_err < self.predock_yaw_tol

    # -----------------------------------------------------------------------
    # Main loop
    # -----------------------------------------------------------------------
    def control_loop(self):
        if not self.have_pose():
            return

        if self.stage == 'DONE':
            self.publish_stop()
            return

        # ==========================================================
        # STAGE 1: Plan + Stanley-track Reeds-Shepp path to pre-dock
        # ==========================================================
        if self.stage == 'PREDOCK':
            if not self.planned:
                if not self.plan_path():
                    self.publish_stop()
                    return

            if self.at_predock():
                self.get_logger().info('Reached pre-dock. Switching to FINAL_APPROACH.')
                self.stage = 'FINAL_APPROACH'
                self.publish_stop()
                return

            # Compute Stanley command for current segment
            delta_math, reverse, seg_done, dbg = self.stanley_control()

            # Advance segment if done
            if seg_done and self.current_segment < len(self.segment_starts) - 2:
                self.current_segment += 1
                self.segment_target_idx = self.segment_starts[self.current_segment]
                self.get_logger().info(
                    f'Segment {self.current_segment - 1} done, '
                    f'advancing to segment {self.current_segment}')

            # Check for replan
            if self.maybe_replan(reverse):
                self.publish_stop()
                return

            # Speed: only slow down for VERY hard steering, not just any curve.
            if abs(delta_math) > self.hard_steer_threshold * self.max_steer_angle:
                target_speed = self.hard_steer_speed_reverse if reverse \
                               else self.hard_steer_speed_forward
            else:
                target_speed = self.target_speed_reverse if reverse \
                               else self.target_speed_forward

            throttle, brake, _ = self.compute_speed_control(target_speed)
            self.publish_control(throttle, brake, delta_math, reverse)

            self.update_debug_plot(dbg, delta_math)

        # ==========================================================
        # STAGE 2: Straight-line reverse into cargo box
        # ==========================================================
        elif self.stage == 'FINAL_APPROACH':
            delta_math, done = self.final_approach_control()
            if done:
                self.get_logger().info('Docking complete.')
                self.stage = 'DONE'
                self.publish_stop()
                return

            throttle, brake, _ = self.compute_speed_control(
                self.target_speed_final_approach)
            self.publish_control(throttle, brake, delta_math, reverse=True)

    # -----------------------------------------------------------------------
    # Output (convert math -> CARLA at the boundary)
    # -----------------------------------------------------------------------
    def publish_control(self, throttle, brake, delta_math, reverse):
        steer_carla = math_to_carla_steer(delta_math, self.max_steer_angle)

        msg = VehicleControl()
        msg.throttle = throttle
        msg.steer = steer_carla
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
        msg.brake = 1.0
        msg.reverse = False
        msg.hand_brake = False
        msg.manual_gear_shift = False
        msg.gear = 0
        self.control_pub.publish(msg)

    # -----------------------------------------------------------------------
    # Debug plot (math frame; flip y for visual to match CARLA top-down view)
    # -----------------------------------------------------------------------
    def update_debug_plot(self, dbg, delta_math):
        if not self.debug_plot or self.path_x is None:
            return
        now = time.time()
        if now - self.last_debug_plot_time < 1.0 / max(self.debug_plot_rate_hz, 1e-6):
            return
        self.last_debug_plot_time = now

        if self.debug_fig is None:
            plt.ion()
            self.debug_fig, self.debug_ax = plt.subplots()

        ax = self.debug_ax
        ax.clear()

        forward = self.path_directions > 0
        reverse_mask = self.path_directions < 0
        ax.plot(self.path_x[forward], self.path_y[forward], '.b',
                markersize=3, label='path forward')
        ax.plot(self.path_x[reverse_mask], self.path_y[reverse_mask], '.r',
                markersize=3, label='path reverse')

        # Goals
        gx_p, gy_p, gyaw_p = self.predock_math
        gx_c, gy_c, gyaw_c = self.cargobox_math
        ax.plot(gx_p, gy_p, 'ko', markersize=8, label='predock')
        ax.plot(gx_c, gy_c, 'ks', markersize=8, label='cargobox')

        # Vehicle
        ax.plot(self.x, self.y, 'mo', markersize=7, label='actor (math)')
        ax.arrow(self.x, self.y, math.cos(self.yaw), math.sin(self.yaw),
                 head_width=0.3, fc='m', ec='m')

        # Tracked axle (always rear axle now)
        rev = dbg['reverse']
        axx, axy = self.get_axle_position()
        ax.plot(axx, axy, 'co', markersize=7, label='rear axle (tracked)')

        # Closest point
        gi = dbg['global_idx']
        ax.plot(self.path_x[gi], self.path_y[gi], 'yo', markersize=9,
                label=f'closest idx {gi}')

        ax.set_title(
            f"stage={self.stage} seg={dbg['segment']} rev={rev} "
            f"delta={math.degrees(delta_math):+.1f}° "
            f"cte={dbg['cte']:+.2f}m he={math.degrees(dbg['heading_error']):+.1f}°"
        )
        ax.set_xlabel('x [m] (math)')
        ax.set_ylabel('y [m] (math, +y = south in CARLA)')
        ax.grid(True)
        ax.axis('equal')
        ax.legend(loc='best', fontsize=8)
        self.debug_fig.canvas.draw_idle()
        self.debug_fig.canvas.flush_events()
        plt.pause(0.001)


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