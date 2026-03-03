#!/usr/bin/env python3

"""
Test publisher for motor speed topics.

Publishes Float32 values to /motor_speed/setpoint (and optionally /motor1/speed)
in a smooth repeating ramp (min -> max -> min) so you can verify:
  - "Set motor speed from topic" block (one-shot value)
  - "Subscribe to motor speed topic" block (continuous updates)

Usage:
  ros2 run assembly_line_control test_motor_speed_publisher

Then in the block UI:
  1. Use "Set motor speed from topic" with topic /motor_speed/setpoint and run the workflow;
     it should read one value and set the motor speed.
  2. Use "Subscribe to motor speed topic" with topic /motor_speed/setpoint; the Live speed
     in the Speed panel should follow the smooth ramp.
  3. Use "Unsubscribe from motor speed topic" to stop following; speed stays at last value.
"""

import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class TestMotorSpeedPublisher(Node):
    """Publishes a smooth repeating speed ramp for testing motor speed blocks."""

    def __init__(self):
        super().__init__('test_motor_speed_publisher')

        self.declare_parameter('topic', 'motor_speed/setpoint')
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('also_publish_motor1_speed', True)
        self.declare_parameter('min_speed', 100.0)
        self.declare_parameter('max_speed', 800.0)
        self.declare_parameter('ramp_duration_sec', 5.0)  # Time to go from min to max (half cycle)

        self.topic_name = self.get_parameter('topic').value
        if not self.topic_name.startswith('/'):
            self.topic_name = '/' + self.topic_name
        self.rate_hz = self.get_parameter('rate_hz').value
        self.also_motor1 = self.get_parameter('also_publish_motor1_speed').value
        self.min_speed = float(self.get_parameter('min_speed').value)
        self.max_speed = float(self.get_parameter('max_speed').value)
        self.ramp_duration_sec = float(self.get_parameter('ramp_duration_sec').value)

        self.start_time = time.monotonic()

        self.pub = self.create_publisher(Float32, self.topic_name.strip('/'), 10)
        self.motor1_pub = self.create_publisher(Float32, 'motor1/speed', 10) if self.also_motor1 else None

        period = 1.0 / self.rate_hz
        self.timer = self.create_timer(period, self.timer_callback)

        self.get_logger().info(
            f'Test motor speed publisher: {self.topic_name} at {self.rate_hz} Hz. '
            f'Smooth ramp {self.min_speed:.0f} -> {self.max_speed:.0f} sps, ramp_duration={self.ramp_duration_sec}s. '
            f'also_publish_motor1_speed={self.also_motor1}'
        )

    def timer_callback(self):
        # One full cycle = ramp up (ramp_duration_sec) + ramp down (ramp_duration_sec)
        cycle_duration = 2.0 * self.ramp_duration_sec
        t = (time.monotonic() - self.start_time) % cycle_duration
        # 0..ramp_duration = up, ramp_duration..2*ramp_duration = down
        if t <= self.ramp_duration_sec:
            # Smooth ramp up: use sine for smooth start/end (no sharp corners)
            phase = math.pi * (t / self.ramp_duration_sec)
            frac = 0.5 * (1.0 - math.cos(phase))
            speed = self.min_speed + frac * (self.max_speed - self.min_speed)
        else:
            # Smooth ramp down
            t_local = t - self.ramp_duration_sec
            phase = math.pi * (t_local / self.ramp_duration_sec)
            frac = 0.5 * (1.0 - math.cos(phase))
            speed = self.max_speed - frac * (self.max_speed - self.min_speed)

        msg = Float32()
        msg.data = float(speed)
        self.pub.publish(msg)
        if self.motor1_pub:
            self.motor1_pub.publish(msg)


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
