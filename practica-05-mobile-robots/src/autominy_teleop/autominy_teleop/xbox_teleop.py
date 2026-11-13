#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

from autominy_msgs.msg import NormalizedSpeedCommand, NormalizedSteeringCommand


def clamp(value: float, low: float = -1.0, high: float = 1.0) -> float:
    return max(low, min(high, value))


class XboxTeleop(Node):
    """
    Maps sensor_msgs/Joy to AutoMiny normalized speed and steering commands.

    Safety behavior:
    - Motion is enabled only while the configured dead-man button is held.
    - Releasing the button immediately publishes speed=0 and steering=0.
    - If /joy stops arriving for joy_timeout seconds, a stop command is sent.
    """

    def __init__(self):
        super().__init__("xbox_teleop")

        self.declare_parameter("joy_topic", "/joy")
        self.declare_parameter("speed_topic", "/actuators/speed_normalized")
        self.declare_parameter("steering_topic", "/actuators/steering_normalized")

        # Typical Xbox mapping with ROS 2 joy:
        # axis 0 = left stick horizontal, axis 1 = left stick vertical.
        # Verify on your controller with: ros2 topic echo /joy
        self.declare_parameter("speed_axis", 1)
        self.declare_parameter("steering_axis", 0)
        self.declare_parameter("enable_button", 0)  # A button on many Xbox mappings

        self.declare_parameter("speed_scale", 0.25)
        self.declare_parameter("steering_scale", 0.50)
        self.declare_parameter("speed_sign", 1.0)
        self.declare_parameter("steering_sign", 1.0)
        self.declare_parameter("deadzone", 0.08)
        self.declare_parameter("joy_timeout", 0.50)

        self.joy_topic = str(self.get_parameter("joy_topic").value)
        self.speed_topic = str(self.get_parameter("speed_topic").value)
        self.steering_topic = str(self.get_parameter("steering_topic").value)

        self.speed_axis = int(self.get_parameter("speed_axis").value)
        self.steering_axis = int(self.get_parameter("steering_axis").value)
        self.enable_button = int(self.get_parameter("enable_button").value)

        self.speed_scale = float(self.get_parameter("speed_scale").value)
        self.steering_scale = float(self.get_parameter("steering_scale").value)
        self.speed_sign = float(self.get_parameter("speed_sign").value)
        self.steering_sign = float(self.get_parameter("steering_sign").value)
        self.deadzone = float(self.get_parameter("deadzone").value)
        self.joy_timeout = float(self.get_parameter("joy_timeout").value)

        self.speed_pub = self.create_publisher(
            NormalizedSpeedCommand, self.speed_topic, 10
        )
        self.steering_pub = self.create_publisher(
            NormalizedSteeringCommand, self.steering_topic, 10
        )
        self.joy_sub = self.create_subscription(
            Joy, self.joy_topic, self.joy_callback, 10
        )

        self.last_joy_time = None
        self.was_enabled = False
        self.timer = self.create_timer(0.10, self.watchdog_callback)

        self.get_logger().info(
            f"Xbox teleop ready. Hold button index {self.enable_button} to enable motion. "
            f"speed_axis={self.speed_axis}, steering_axis={self.steering_axis}, "
            f"speed_scale={self.speed_scale}, steering_scale={self.steering_scale}"
        )

    def apply_deadzone(self, value: float) -> float:
        if abs(value) < self.deadzone:
            return 0.0
        return value

    def publish_commands(self, speed: float, steering: float):
        now = self.get_clock().now().to_msg()

        speed_msg = NormalizedSpeedCommand()
        speed_msg.header.stamp = now
        speed_msg.value = float(clamp(speed))

        steering_msg = NormalizedSteeringCommand()
        steering_msg.header.stamp = now
        steering_msg.value = float(clamp(steering))

        self.speed_pub.publish(speed_msg)
        self.steering_pub.publish(steering_msg)

    def stop_vehicle(self):
        self.publish_commands(0.0, 0.0)

    def joy_callback(self, msg: Joy):
        self.last_joy_time = self.get_clock().now()

        max_axis = max(self.speed_axis, self.steering_axis)
        if max_axis >= len(msg.axes):
            self.get_logger().error(
                f"Joystick only reports {len(msg.axes)} axes, but configuration "
                f"requires axis {max_axis}. Run 'ros2 topic echo /joy' and adjust config/xbox.yaml."
            )
            self.stop_vehicle()
            return

        if self.enable_button >= len(msg.buttons):
            self.get_logger().error(
                f"Joystick only reports {len(msg.buttons)} buttons, but enable_button="
                f"{self.enable_button}. Adjust config/xbox.yaml."
            )
            self.stop_vehicle()
            return

        enabled = bool(msg.buttons[self.enable_button])

        if not enabled:
            if self.was_enabled:
                self.get_logger().info("Dead-man released -> STOP")
            self.was_enabled = False
            self.stop_vehicle()
            return

        speed_raw = self.apply_deadzone(float(msg.axes[self.speed_axis]))
        steering_raw = self.apply_deadzone(float(msg.axes[self.steering_axis]))

        speed = speed_raw * self.speed_scale * self.speed_sign
        steering = steering_raw * self.steering_scale * self.steering_sign

        self.publish_commands(speed, steering)
        self.was_enabled = True

    def watchdog_callback(self):
        if self.last_joy_time is None:
            return

        elapsed = (self.get_clock().now() - self.last_joy_time).nanoseconds / 1e9
        if elapsed > self.joy_timeout and self.was_enabled:
            self.get_logger().warn(
                f"No /joy messages for {elapsed:.2f} s -> STOP"
            )
            self.was_enabled = False
            self.stop_vehicle()


def main(args=None):
    rclpy.init(args=args)
    node = XboxTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_vehicle()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
