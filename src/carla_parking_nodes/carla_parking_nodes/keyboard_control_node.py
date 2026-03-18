#!/usr/bin/env python3

import sys
import termios
import tty
import select

import rclpy
from rclpy.node import Node

from carla_parking_msgs.msg import VehicleControl


HELP_TEXT = """
Keyboard Control
----------------
w : increase throttle
s : increase brake
a : steer left
d : steer right
x : center steering
r : toggle reverse
space : toggle hand brake
q : zero throttle and brake
e : emergency brake
c : clear all control
CTRL+C : quit
"""


class KeyboardControlNode(Node):
    def __init__(self):
        super().__init__('keyboard_control_node')

        self.control_pub = self.create_publisher(
            VehicleControl,
            '/vehicle/control_cmd',
            10
        )

        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('throttle_step', 0.05)
        self.declare_parameter('brake_step', 0.10)
        self.declare_parameter('steer_step', 0.10)

        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.throttle_step = float(self.get_parameter('throttle_step').value)
        self.brake_step = float(self.get_parameter('brake_step').value)
        self.steer_step = float(self.get_parameter('steer_step').value)

        self.throttle = 0.0
        self.steer = 0.0
        self.brake = 0.0
        self.reverse = False
        self.hand_brake = False
        self.manual_gear_shift = False
        self.gear = 0

        self.last_printed_state = None

        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_control)

        print(HELP_TEXT)
        self.print_state(force=True)

    def clamp(self, value, low, high):
        return max(low, min(high, value))

    def get_key_nonblocking(self):
        dr, _, _ = select.select([sys.stdin], [], [], 0.0)
        if dr:
            return sys.stdin.read(1)
        return None

    def process_key(self, key: str):
        if key == 'w':
            self.throttle = self.clamp(self.throttle + self.throttle_step, 0.0, 1.0)
            self.brake = 0.0

        elif key == 's':
            self.brake = self.clamp(self.brake + self.brake_step, 0.0, 1.0)
            self.throttle = 0.0

        elif key == 'a':
            self.steer = self.clamp(self.steer - self.steer_step, -1.0, 1.0)

        elif key == 'd':
            self.steer = self.clamp(self.steer + self.steer_step, -1.0, 1.0)

        elif key == 'x':
            self.steer = 0.0

        elif key == 'r':
            self.reverse = not self.reverse
            self.gear = -1 if self.reverse else 0

        elif key == ' ':
            self.hand_brake = not self.hand_brake

        elif key == 'q':
            self.throttle = 0.0
            self.brake = 0.0

        elif key == 'e':
            self.throttle = 0.0
            self.brake = 1.0
            self.hand_brake = False

        elif key == 'c':
            self.throttle = 0.0
            self.steer = 0.0
            self.brake = 0.0
            self.reverse = False
            self.hand_brake = False
            self.manual_gear_shift = False
            self.gear = 0

        self.print_state()

    def print_state(self, force=False):
        state = (
            round(self.throttle, 3),
            round(self.steer, 3),
            round(self.brake, 3),
            self.reverse,
            self.hand_brake,
            self.gear
        )

        if (state != self.last_printed_state) or force:
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

    def publish_control(self):
        # Read as many queued keypresses as are available
        while True:
            key = self.get_key_nonblocking()
            if key is None:
                break
            self.process_key(key)

        msg = VehicleControl()
        msg.throttle = float(self.throttle)
        msg.steer = float(self.steer)
        msg.brake = float(self.brake)
        msg.reverse = bool(self.reverse)
        msg.hand_brake = bool(self.hand_brake)
        msg.manual_gear_shift = bool(self.manual_gear_shift)
        msg.gear = int(self.gear)

        self.control_pub.publish(msg)


def main(args=None):
    old_settings = termios.tcgetattr(sys.stdin)

    try:
        tty.setcbreak(sys.stdin.fileno())

        rclpy.init(args=args)
        node = KeyboardControlNode()

        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            # Publish a safe stop once before exit
            stop_msg = VehicleControl()
            stop_msg.throttle = 0.0
            stop_msg.steer = 0.0
            stop_msg.brake = 1.0
            stop_msg.reverse = False
            stop_msg.hand_brake = False
            stop_msg.manual_gear_shift = False
            stop_msg.gear = 0

            try:
                node.control_pub.publish(stop_msg)
            except Exception:
                pass

            node.destroy_node()

            if rclpy.ok():
                rclpy.shutdown()

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        print()


if __name__ == '__main__':
    main()