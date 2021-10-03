#!/usr/bin/env python

"""
ackermann_drive_keyop.py:
    A ros keyboard teleoperation script for ackermann steering based robots
"""

__author__ = "George Kouros"
__license__ = "GPLv3"
__maintainer__ = "George Kouros"
__email__ = "gkourosg@yahoo.gr"

import rclpy
from rclpy.node import Node

# from ackermann_msgs.msg import AckermannDriveStamped
import sys, select, termios, tty


from geometry_msgs.msg import Twist

try:
    import thread
except ImportError:
    # for python3 compatability
    import _thread
from numpy import clip

control_keys = {
    "up": "\x41",
    "down": "\x42",
    "right": "\x43",
    "left": "\x44",
    "space": "\x20",
    "tab": "\x09",
}

key_bindings = {
    "\x41": (1.0, 0.0),
    "\x42": (-1.0, 0.0),
    "\x43": (0.0, -1.0),
    "\x44": (0.0, 1.0),
    "\x20": (0.0, 0.0),
    "\x09": (0.0, 0.0),
}


class AckermannDriveKeyop(Node):
    def __init__(self, args):
        super().__init__("ackermann_drive_teleop")
        self.get_logger().info(f"nr args: {len(args)}")
        if len(args) == 1:
            max_speed = float(args[0])
            max_steering_angle = float(args[0])
        elif len(args) >= 2:
            max_speed = float(args[0])
            max_steering_angle = float(args[1])
        else:
            max_speed = 100
            max_steering_angle = 60

        if len(args) > 2:
            cmd_topic = "/" + args[2]
        else:
            cmd_topic = "ackermann_cmd"

        self.speed_range = [-float(max_speed), float(max_speed)]
        self.steering_angle_range = [
            -float(max_steering_angle),
            float(max_steering_angle),
        ]
        for key in key_bindings:
            key_bindings[key] = (
                key_bindings[key][0] * float(max_speed) / 5,
                key_bindings[key][1] * float(max_steering_angle) / 5,
            )

        self.speed = 0.0
        self.steering_angle = 0.0
        self.publisher = self.create_publisher(Twist, cmd_topic, 10)
        timer_period = 0.5  # Seconds
        self.timer = self.create_timer(timer_period, self.pub_callback)
        self.print_state()
        self.key_loop()

    def pub_callback(self):
        self.get_logger().info("Publishing new speed and steering angle...")
        twist = Twist()
        twist.linear.x = self.speed
        twist.linear.y = self.steering_angle
        self.publisher.publish(twist)

    def print_state(self):
        sys.stderr.write("\x1b[2J\x1b[H")
        self.get_logger().info("\x1b[1M\r*********************************************")
        self.get_logger().info("\x1b[1M\rUse arrows to change speed and steering angle")
        self.get_logger().info("\x1b[1M\rUse space to brake and tab to align wheels")
        self.get_logger().info("\x1b[1M\rPress <ctrl-c> or <q> to exit")
        self.get_logger().info("\x1b[1M\r*********************************************")
        self.get_logger().info(
            "\x1b[1M\r"
            "\033[34;1mSpeed: \033[32;1m%0.2f m/s, "
            "\033[34;1mSteer Angle: \033[32;1m%0.2f rad\033[0m"
            % (self.speed, self.steering_angle),
        )

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def key_loop(self):
        self.settings = termios.tcgetattr(sys.stdin)
        while True:
            twist = Twist()
            twist.linear.x = self.speed
            twist.linear.y = self.steering_angle
            self.publisher.publish(twist)
            key = self.get_key()
            if key in key_bindings.keys():
                if key == control_keys["space"]:
                    self.speed = 0.0
                elif key == control_keys["tab"]:
                    self.steering_angle = 0.0
                else:
                    self.speed = self.speed + key_bindings[key][0]
                    self.steering_angle = self.steering_angle + key_bindings[key][1]
                    self.speed = clip(
                        self.speed, self.speed_range[0], self.speed_range[1]
                    )
                    self.steering_angle = clip(
                        self.steering_angle,
                        self.steering_angle_range[0],
                        self.steering_angle_range[1],
                    )
                self.print_state()
            elif key == "\x03" or key == "\x71":  # ctr-c or q
                break
            else:
                continue
        self.finalize()

    def finalize(self):
        self.get_logger().info("Halting motors, aligning wheels and exiting...")
        self.settings = termios.tcgetattr(sys.stdin)
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        self.publisher.publish(twist)
        sys.exit()


def main(args=None):
    rclpy.init(args=args)
    keyop = AckermannDriveKeyop(sys.argv[1 : len(sys.argv)])
    rclpy.spin(keyop)
    keyop.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
