#!/usr/bin/env python3

"""
Test publisher for motor speed topics.

Publishes Float32 values to /motor_speed/setpoint (and optionally /motor1/speed)
in a repeating pattern so you can verify:
  - "Set motor speed from topic" block (one-shot value)
  - "Subscribe to motor speed topic" block (continuous updates)

Usage:
  ros2 run assembly_line_control test_motor_speed_publisher

Then in the block UI:
  1. Use "Set motor speed from topic" with topic /motor_speed/setpoint and run the workflow;
     it should read one value and set the motor speed.
  2. Use "Subscribe to motor speed topic" with topic /motor_speed/setpoint; the Live speed
     in the Speed panel should cycle through the pattern.
  3. Use "Unsubscribe from motor speed topic" to stop following; speed stays at last value.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class TestMotorSpeedPublisher(Node):
    """Publishes a repeating speed pattern for testing motor speed blocks."""

    def __init__(self):
        super().__init__('test_motor_speed_publisher')

        self.declare_parameter('topic', 'motor_speed/setpoint')
        self.declare_parameter('rate_hz', 5)
        self.declare_parameter('also_publish_motor1_speed', True)

        self.topic_name = self.get_parameter('topic').value
        if not self.topic_name.startswith('/'):
            self.topic_name = '/' + self.topic_name
        self.rate_hz = self.get_parameter('rate_hz').value
        self.also_motor1 = self.get_parameter('also_publish_motor1_speed').value

        # Pattern: (speed in steps/sec, hold for this many publish cycles)
        self.pattern = [
            (100.0, 3),
            (250.0, 3),
            (500.0, 3),
            (250.0, 3),
            (100.0, 3),
        ]
        self.pattern_index = 0
        self.cycle_count = 0

        self.pub = self.create_publisher(Float32, self.topic_name.strip('/'), 10)
        self.motor1_pub = self.create_publisher(Float32, 'motor1/speed', 10) if self.also_motor1 else None

        period = 1.0 / self.rate_hz
        self.timer = self.create_timer(period, self.timer_callback)

        self.get_logger().info(
            f'Test motor speed publisher: {self.topic_name} at {self.rate_hz} Hz. '
            f'Pattern: 100 -> 250 -> 500 -> 250 -> 100 (steps/sec). also_publish_motor1_speed={self.also_motor1}'
        )

    def timer_callback(self):
        speed, hold = self.pattern[self.pattern_index]
        msg = Float32()
        msg.data = speed
        self.pub.publish(msg)
        if self.motor1_pub:
            self.motor1_pub.publish(msg)

        self.cycle_count += 1
        if self.cycle_count >= hold:
            self.cycle_count = 0
            self.pattern_index = (self.pattern_index + 1) % len(self.pattern)
            if self.pattern_index == 0:
                self.get_logger().info('Pattern cycle: 100 -> 250 -> 500 -> 250 -> 100')


def main(args=None):
    rclpy.init(args=args)
    node = TestMotorSpeedPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
