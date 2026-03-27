#!/usr/bin/env python3

import math
import pygame
import rclpy
from rclpy.node import Node

from carla_parking_msgs.msg import VehicleControl


class XboxControlNode(Node):
    def __init__(self):
        super().__init__('xbox_control_node')

        self.control_pub = self.create_publisher(
            VehicleControl,
            '/vehicle/control_cmd',
            10
        )

        # -----------------------------
        # Parameters
        # -----------------------------
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('deadzone', 0.08)
        self.declare_parameter('max_steer', 0.9)
        self.declare_parameter('steer_smoothing', 0.25)
        self.declare_parameter('steer_return_rate', 0.18)
        self.declare_parameter('throttle_gain', 1.0)
        self.declare_parameter('brake_gain', 1.0)

        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.deadzone = float(self.get_parameter('deadzone').value)
        self.max_steer = float(self.get_parameter('max_steer').value)
        self.steer_smoothing = float(self.get_parameter('steer_smoothing').value)
        self.steer_return_rate = float(self.get_parameter('steer_return_rate').value)
        self.throttle_gain = float(self.get_parameter('throttle_gain').value)
        self.brake_gain = float(self.get_parameter('brake_gain').value)

        # -----------------------------
        # Pygame / joystick init
        # -----------------------------
        pygame.init()
        pygame.joystick.init()

        count = pygame.joystick.get_count()
        if count == 0:
            raise RuntimeError("No controller detected")

        self.js = pygame.joystick.Joystick(0)
        self.js.init()

        self.get_logger().info(f'Controller: {self.js.get_name()}')
        self.get_logger().info(f'Axes: {self.js.get_numaxes()}, Buttons: {self.js.get_numbuttons()}, Hats: {self.js.get_numhats()}')

        # -----------------------------
        # Control state
        # -----------------------------
        self.throttle = 0.0
        self.brake = 0.0
        self.steer = 0.0
        self.reverse = False
        self.hand_brake = False
        self.manual_gear_shift = False
        self.gear = 0

        # Button edge detection
        self.prev_buttons = {}

        # Timer
        self.dt = 1.0 / self.publish_rate
        self.timer = self.create_timer(self.dt, self.update_and_publish)

        self.last_printed_state = None
        self.print_help()
        self.print_state(force=True)

    # --------------------------------------------------
    # Helpers
    # --------------------------------------------------
    @staticmethod
    def clamp(value, low, high):
        return max(low, min(high, value))

    def apply_deadzone(self, value):
        if abs(value) < self.deadzone:
            return 0.0
        # Optional rescale so motion starts smoothly outside deadzone
        sign = 1.0 if value >= 0.0 else -1.0
        scaled = (abs(value) - self.deadzone) / (1.0 - self.deadzone)
        return sign * scaled

    def normalize_trigger(self, raw_value):
        """
        Xbox triggers in pygame often come as:
            -1.0 -> released
            +1.0 -> fully pressed
        Convert to:
             0.0 -> released
             1.0 -> fully pressed
        """
        normalized = (raw_value + 1.0) * 0.5
        return self.clamp(normalized, 0.0, 1.0)

    def button_pressed_once(self, index):
        current = bool(self.js.get_button(index))
        prev = self.prev_buttons.get(index, False)
        self.prev_buttons[index] = current
        return current and not prev

    # --------------------------------------------------
    # Main control logic
    # --------------------------------------------------
    def process_controller_events(self):
        # Keep pygame state fresh
        pygame.event.pump()

        # Common Xbox mapping in pygame 2.x docs:
        # axis 0 = left stick x
        # axis 2 = LT
        # axis 5 = RT
        # button 0=A, 1=B, 2=X, 3=Y
        lx = self.apply_deadzone(self.js.get_axis(0))
        lt = self.normalize_trigger(self.js.get_axis(2))
        rt = self.normalize_trigger(self.js.get_axis(5))

        # Analog throttle/brake
        self.throttle = self.clamp(rt * self.throttle_gain, 0.0, 1.0)
        self.brake = self.clamp(lt * self.brake_gain, 0.0, 1.0)

        # If you want a stronger "either throttle or brake" behavior:
        # keep only the dominant one
        if self.throttle > 0.02 and self.brake > 0.02:
            if self.throttle >= self.brake:
                self.brake = 0.0
            else:
                self.throttle = 0.0

        # Steering: smooth toward target like CARLA keyboard steer cache,
        # but based on analog stick
        target_steer = self.clamp(lx * self.max_steer, -1.0, 1.0)

        if abs(target_steer) > 1e-3:
            self.steer += (target_steer - self.steer) * self.steer_smoothing
        else:
            # smooth return to center when stick released
            if self.steer > 0.0:
                self.steer = max(0.0, self.steer - self.steer_return_rate)
            elif self.steer < 0.0:
                self.steer = min(0.0, self.steer + self.steer_return_rate)

        self.steer = self.clamp(self.steer, -1.0, 1.0)

        # -----------------------------
        # Button mappings
        # -----------------------------
        # A -> hand brake hold
        self.hand_brake = bool(self.js.get_button(0))

        # B -> toggle reverse
        if self.button_pressed_once(1):
            self.reverse = not self.reverse
            self.gear = -1 if self.reverse else 0

        # X -> clear throttle/brake and center steering
        if self.button_pressed_once(2):
            self.throttle = 0.0
            self.brake = 0.0
            self.steer = 0.0
            self.hand_brake = False

        # Y -> emergency brake
        if self.button_pressed_once(3):
            self.throttle = 0.0
            self.brake = 1.0
            self.hand_brake = False

        # LB -> force neutral / forward
        if self.button_pressed_once(4):
            self.reverse = False
            self.gear = 0

        # RB -> toggle reverse as alternative
        if self.button_pressed_once(5):
            self.reverse = not self.reverse
            self.gear = -1 if self.reverse else 0

        # Back button -> full reset
        if self.js.get_numbuttons() > 6 and self.button_pressed_once(6):
            self.throttle = 0.0
            self.brake = 0.0
            self.steer = 0.0
            self.reverse = False
            self.hand_brake = False
            self.manual_gear_shift = False
            self.gear = 0

    def publish_control(self):
        msg = VehicleControl()
        msg.throttle = float(self.throttle)
        msg.steer = float(self.steer)
        msg.brake = float(self.brake)
        msg.reverse = bool(self.reverse)
        msg.hand_brake = bool(self.hand_brake)
        msg.manual_gear_shift = bool(self.manual_gear_shift)
        msg.gear = int(self.gear)

        self.control_pub.publish(msg)

    def print_help(self):
        print(
            "\nXbox controller mapping:\n"
            "  RT  : throttle\n"
            "  LT  : brake\n"
            "  LSX : steer\n"
            "  A   : hand brake (hold)\n"
            "  B   : toggle reverse\n"
            "  X   : center steer + zero throttle/brake\n"
            "  Y   : emergency brake\n"
            "  LB  : force forward/neutral\n"
            "  RB  : toggle reverse\n"
            "  Back: reset all\n"
        )

    def print_state(self, force=False):
        state = (
            round(self.throttle, 3),
            round(self.steer, 3),
            round(self.brake, 3),
            self.reverse,
            self.hand_brake,
            self.gear
        )

        if force or state != self.last_printed_state:
            print(
                f'\rthrottle={self.throttle:.2f}  '
                f'steer={self.steer:.2f}  '
                f'brake={self.brake:.2f}  '
                f'reverse={self.reverse}  '
                f'hand_brake={self.hand_brake}  '
                f'gear={self.gear}   ',
                end='',
                flush=True
            )
            self.last_printed_state = state

    def update_and_publish(self):
        self.process_controller_events()
        self.publish_control()
        self.print_state()

    def shutdown_safe_stop(self):
        stop_msg = VehicleControl()
        stop_msg.throttle = 0.0
        stop_msg.steer = 0.0
        stop_msg.brake = 1.0
        stop_msg.reverse = False
        stop_msg.hand_brake = False
        stop_msg.manual_gear_shift = False
        stop_msg.gear = 0

        try:
            self.control_pub.publish(stop_msg)
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = XboxControlNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.shutdown_safe_stop()
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

        try:
            pygame.joystick.quit()
            pygame.quit()
        except Exception:
            pass

        print()


if __name__ == '__main__':
    main()