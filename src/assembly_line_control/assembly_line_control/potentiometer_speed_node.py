#!/usr/bin/env python3

"""
Potentiometer Speed Node for Assembly Line OS

Subscribes to /potentiometer/raw (from Arduino via arduino_controller),
applies a user-defined mapping (raw 0-1023 -> motor speed in steps/sec),
and publishes to /motor_speed/setpoint and /motor1/speed (and optionally
/motor2/speed) so the hardware and frontend reflect live variable speed.

You can change the mapping by editing the map_pot_to_speed() function below
or by adjusting the min_speed / max_speed parameters.
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
        self.declare_parameter('publish_motor1', True)
        self.declare_parameter('publish_motor2', False)
        self.declare_parameter('publish_rate_hz', 15.0)

        self.min_speed = self.get_parameter('min_speed').value
        self.max_speed = self.get_parameter('max_speed').value
        self.publish_motor1 = self.get_parameter('publish_motor1').value
        self.publish_motor2 = self.get_parameter('publish_motor2').value
        self.min_interval = 1.0 / self.get_parameter('publish_rate_hz').value

        self.last_publish_time = 0.0
        self.last_raw = 0.0

        self.setpoint_pub = self.create_publisher(Float32, 'motor_speed/setpoint', 10)
        self.motor1_speed_pub = self.create_publisher(Float32, 'motor1/speed', 10) if self.publish_motor1 else None
        self.motor2_speed_pub = self.create_publisher(Float32, 'motor2/speed', 10) if self.publish_motor2 else None

        self.pot_sub = self.create_subscription(
            Float32, 'potentiometer/raw',
            self.pot_callback, 10)

        self.get_logger().info(
            f'Potentiometer speed node started: raw 0-1023 -> {self.min_speed}-{self.max_speed} sps, '
            f'motor1={self.publish_motor1}, motor2={self.publish_motor2}'
        )

    def pot_callback(self, msg):
        import time
        now = time.time()
        if now - self.last_publish_time < self.min_interval:
            return
        self.last_publish_time = now
        self.last_raw = msg.data

        speed = map_pot_to_speed(msg.data, self.min_speed, self.max_speed)
        speed = max(self.min_speed, min(self.max_speed, speed))

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
