#!/usr/bin/env python3

"""
Potentiometer Speed Node for Assembly Line OS

Subscribes to /potentiometer/raw (from Arduino via arduino_controller),
applies a user-defined mapping (raw 0-1023 -> motor speed in steps/sec),
and publishes to /motor_speed/setpoint. Optionally publishes to /motor1/speed
and /motor2/speed (default: off) so the pot does not drive the motor at
startup until the user subscribes to the setpoint topic in the UI.

Smoothing (EMA) and a speed deadband reduce jitter from ADC noise.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


def map_pot_to_speed(raw: float, min_speed: float, max_speed: float) -> float:
    """
    Map potentiometer raw value (0-1023) to motor speed (steps/sec).
    Override this function or use parameters to customize the curve.
    Default: linear mapping.
    """
    raw = max(0.0, min(1023.0, float(raw)))
    t = raw / 1023.0
    return min_speed + t * (max_speed - min_speed)


class PotentiometerSpeedNode(Node):
    """Maps potentiometer raw value to motor speed and publishes to ROS."""

    def __init__(self):
        super().__init__('potentiometer_speed_node')

        self.declare_parameter('min_speed', 1.0)
        self.declare_parameter('max_speed', 6500.0)
        self.declare_parameter('publish_motor1', False)  # False = don't drive motor at startup; UI subscribes to setpoint and publishes to motor1/speed when user enables it
        self.declare_parameter('publish_motor2', False)
        self.declare_parameter('publish_rate_hz', 15.0)
        self.declare_parameter('smoothing_alpha', 0.2)   # EMA: 0=heavy smooth, 1=no smooth; 0.2 reduces ADC jitter
        self.declare_parameter('speed_deadband', 15.0)   # Only publish if speed changes by more than this (steps/sec)

        self.min_speed = self.get_parameter('min_speed').value
        self.max_speed = self.get_parameter('max_speed').value
        self.publish_motor1 = self.get_parameter('publish_motor1').value
        self.publish_motor2 = self.get_parameter('publish_motor2').value
        self.min_interval = 1.0 / self.get_parameter('publish_rate_hz').value
        self.smoothing_alpha = self.get_parameter('smoothing_alpha').value
        self.speed_deadband = self.get_parameter('speed_deadband').value

        self.last_publish_time = 0.0
        self.smoothed_raw = None   # EMA of raw; None until first sample
        self.last_published_speed = None   # For deadband

        self.setpoint_pub = self.create_publisher(Float32, 'motor_speed/setpoint', 10)
        self.motor1_speed_pub = self.create_publisher(Float32, 'motor1/speed', 10) if self.publish_motor1 else None
        self.motor2_speed_pub = self.create_publisher(Float32, 'motor2/speed', 10) if self.publish_motor2 else None

        self.pot_sub = self.create_subscription(
            Float32, 'potentiometer/raw',
            self.pot_callback, 10)

        self.get_logger().info(
            f'Potentiometer speed node started: raw 0-1023 -> {self.min_speed}-{self.max_speed} sps, '
            f'motor1={self.publish_motor1}, motor2={self.publish_motor2}, '
            f'smoothing_alpha={self.smoothing_alpha}, deadband={self.speed_deadband}'
        )

    def pot_callback(self, msg):
        import time
        raw = max(0.0, min(1023.0, float(msg.data)))
        if self.smoothed_raw is None:
            self.smoothed_raw = raw
        else:
            self.smoothed_raw = (
                self.smoothing_alpha * raw + (1.0 - self.smoothing_alpha) * self.smoothed_raw
            )

        speed = map_pot_to_speed(self.smoothed_raw, self.min_speed, self.max_speed)
        speed = max(self.min_speed, min(self.max_speed, speed))

        # Deadband: only publish when speed changed meaningfully (or first time)
        if self.last_published_speed is not None and abs(speed - self.last_published_speed) < self.speed_deadband:
            return
        self.last_published_speed = speed

        now = time.time()
        if now - self.last_publish_time < self.min_interval:
            return
        self.last_publish_time = now

        setpoint_msg = Float32()
        setpoint_msg.data = speed
        self.setpoint_pub.publish(setpoint_msg)

        if self.motor1_speed_pub:
            self.motor1_speed_pub.publish(setpoint_msg)
        if self.motor2_speed_pub:
            self.motor2_speed_pub.publish(setpoint_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PotentiometerSpeedNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
