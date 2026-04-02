#!/usr/bin/env python3

"""
Potentiometer Speed Node for Assembly Line OS

Subscribes to /potentiometer/raw (from Arduino via arduino_controller),
scales motor speed by roll radius so material surface speed stays constant.

Calibration uses roll outer diameter vs pot raw (see od_min_inches, od_max_inches,
pot_raw_min, pot_raw_max). Assumes the dancer/pot is approximately linear in OD.

Baseline speed is taken from the last move block (e.g. M1 at 550 sps) via
motor1/speed or motor2/speed; that value is the step rate tuned for the empty
roll (od_min_inches). The param baseline_speed is used until the first such
message. Publishes to /motor_speed/setpoint. Optionally publishes to /motor1/speed
and /motor2/speed (default: off) so the pot does not drive the motor at
startup until the user subscribes to the setpoint topic in the UI.

Smoothing (EMA) and a speed deadband reduce jitter from ADC noise.
"""

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


def pot_calibration_from_roll_od(
    od_min_inches: float,
    od_max_inches: float,
    pot_raw_min: float,
    pot_raw_max: float,
):
    """
    Derive affine pot→radius parameters from physical OD endpoints.

    R(raw) = (od_min/2) + ((od_max - od_min) / 2) * (raw - pot_raw_min) / (pot_raw_max - pot_raw_min)
    """
    raw_span = pot_raw_max - pot_raw_min
    baseline_raw = pot_raw_min
    baseline_radius_inches = od_min_inches / 2.0
    radius_delta_total = (od_max_inches - od_min_inches) / 2.0
    inches_per_count = radius_delta_total / raw_span
    return baseline_raw, baseline_radius_inches, inches_per_count


def map_pot_to_radius(
    raw: float,
    baseline_raw: float,
    baseline_radius_inches: float,
    inches_per_count: float,
    min_radius_inches: float,
    raw_clamp_min: float,
    raw_clamp_max: float,
) -> float:
    """
    Map potentiometer raw value to roll outer radius in inches (affine model).

    R(raw) = R0 + k * (raw - baseline_raw)

    where R0 is outer radius at pot_raw_min (od_min/2) and k is inches per count.
    """
    raw = max(raw_clamp_min, min(raw_clamp_max, float(raw)))
    radius = baseline_radius_inches + inches_per_count * (raw - baseline_raw)
    return max(min_radius_inches, radius)


def map_radius_to_speed(
    radius_inches: float,
    baseline_radius_inches: float,
    baseline_speed: float,
    min_speed: float,
    max_speed: float,
) -> float:
    """
    Map roll radius to motor speed (steps/sec) so that surface speed
    remains approximately constant.

    For constant linear surface speed v at the roll:
        v = omega * R
      => omega ∝ 1 / R

    Since motor step rate is proportional to omega, we scale speed
    inversely with radius:

        speed(R) = baseline_speed * (R0 / R)

    where:
      - R0 is baseline_radius_inches (outer radius at baseline_speed, empty roll)
      - R is current radius_inches
    """
    if radius_inches <= 0.0:
        # Should be prevented by min_radius_inches, but guard anyway.
        radius_inches = baseline_radius_inches

    speed = baseline_speed * (baseline_radius_inches / radius_inches)
    # Clamp within configured min/max to avoid extreme values.
    return max(min_speed, min(max_speed, speed))


class PotentiometerSpeedNode(Node):
    """Maps potentiometer raw value to motor speed and publishes to ROS."""

    def __init__(self):
        super().__init__('potentiometer_speed_node')

        self.declare_parameter('min_speed', 1.0)
        self.declare_parameter('max_speed', 200.0)
        # Roll OD vs pot (defaults: 3.5" empty, +13" OD growth at raw 0→1023)
        self.declare_parameter('od_min_inches', 3.5)
        self.declare_parameter('od_max_inches', 16.5)
        self.declare_parameter('pot_raw_min', 40)
        self.declare_parameter('pot_raw_max', 1023.0)
        self.declare_parameter('baseline_speed', 200.0)  # steps/sec until first move-block speed
        self.declare_parameter('baseline_speed_motor_id', 1)
        self.declare_parameter('min_radius_inches', 1.75)  # floor ≈ empty spool radius (3.5" OD / 2)
        self.declare_parameter('publish_motor1', False)
        self.declare_parameter('publish_motor2', False)
        self.declare_parameter('publish_rate_hz', 15.0)
        self.declare_parameter('smoothing_alpha', 0.6)
        self.declare_parameter('speed_deadband', 15.0)

        self.min_speed = self.get_parameter('min_speed').value
        self.max_speed = self.get_parameter('max_speed').value
        od_min = float(self.get_parameter('od_min_inches').value)
        od_max = float(self.get_parameter('od_max_inches').value)
        pot_raw_min = float(self.get_parameter('pot_raw_min').value)
        pot_raw_max = float(self.get_parameter('pot_raw_max').value)

        if od_max <= od_min:
            self.get_logger().warn(
                f'od_max_inches ({od_max}) must exceed od_min_inches ({od_min}); using od_min + 13'
            )
            od_max = od_min + 13.0
        raw_span = pot_raw_max - pot_raw_min
        if raw_span <= 0.0:
            self.get_logger().warn(
                f'pot_raw_max must exceed pot_raw_min; using pot_raw_min + 1023'
            )
            pot_raw_max = pot_raw_min + 1023.0

        self.baseline_raw, self.baseline_radius_inches, self.inches_per_count = (
            pot_calibration_from_roll_od(od_min, od_max, pot_raw_min, pot_raw_max)
        )
        self.pot_raw_min = pot_raw_min
        self.pot_raw_max = pot_raw_max

        self.baseline_speed = float(self.get_parameter('baseline_speed').value)
        self.baseline_speed_motor_id = int(self.get_parameter('baseline_speed_motor_id').value)
        if self.baseline_speed_motor_id not in (1, 2):
            self.baseline_speed_motor_id = 1
        self.min_radius_inches = float(self.get_parameter('min_radius_inches').value)
        self.publish_motor1 = self.get_parameter('publish_motor1').value
        self.publish_motor2 = self.get_parameter('publish_motor2').value
        self.min_interval = 1.0 / self.get_parameter('publish_rate_hz').value
        self.smoothing_alpha = self.get_parameter('smoothing_alpha').value
        self.speed_deadband = self.get_parameter('speed_deadband').value

        self.last_publish_time = 0.0
        self.smoothed_raw = None
        self.last_published_speed = None

        self.setpoint_pub = self.create_publisher(Float32, 'motor_speed/setpoint', 10)
        self.motor1_speed_pub = self.create_publisher(Float32, 'motor1/speed', 10) if self.publish_motor1 else None
        self.motor2_speed_pub = self.create_publisher(Float32, 'motor2/speed', 10) if self.publish_motor2 else None

        self.pot_sub = self.create_subscription(
            Float32, 'potentiometer/raw',
            self.pot_callback, 10)
        self.motor1_speed_sub = self.create_subscription(
            Float32, 'motor1/speed',
            lambda msg: self._motor_speed_callback(1, msg), 10)
        self.motor2_speed_sub = self.create_subscription(
            Float32, 'motor2/speed',
            lambda msg: self._motor_speed_callback(2, msg), 10)

        r_max = od_max / 2.0
        self.get_logger().info(
            f'Potentiometer speed node: OD {od_min}"–{od_max}" (R {self.baseline_radius_inches:.3f}"–{r_max:.3f}"), '
            f'pot {self.pot_raw_min:.0f}–{self.pot_raw_max:.0f}, k={self.inches_per_count:.6f} in/count; '
            f'speed {self.min_speed}-{self.max_speed} sps, baseline from motor{self.baseline_speed_motor_id}/speed, '
            f'motor1={self.publish_motor1}, motor2={self.publish_motor2}, '
            f'smoothing_alpha={self.smoothing_alpha}, deadband={self.speed_deadband}'
        )

    def _motor_speed_callback(self, motor_id: int, msg):
        """Use speed from last move block (e.g. M1 at 550 sps) as baseline for radius scaling."""
        if motor_id != self.baseline_speed_motor_id:
            return
        speed = max(self.min_speed, min(self.max_speed, float(msg.data)))
        # Do not overwrite baseline with our own published value (avoid feedback)
        if self.last_published_speed is not None and abs(speed - self.last_published_speed) <= self.speed_deadband:
            return
        self.baseline_speed = speed
        self.get_logger().debug(f'Baseline speed set to {self.baseline_speed:.1f} sps from motor{motor_id}/speed')

    def pot_callback(self, msg):
        raw = max(self.pot_raw_min, min(self.pot_raw_max, float(msg.data)))
        if self.smoothed_raw is None:
            self.smoothed_raw = raw
        else:
            self.smoothed_raw = (
                self.smoothing_alpha * raw + (1.0 - self.smoothing_alpha) * self.smoothed_raw
            )

        radius = map_pot_to_radius(
            self.smoothed_raw,
            baseline_raw=self.baseline_raw,
            baseline_radius_inches=self.baseline_radius_inches,
            inches_per_count=self.inches_per_count,
            min_radius_inches=self.min_radius_inches,
            raw_clamp_min=self.pot_raw_min,
            raw_clamp_max=self.pot_raw_max,
        )
        speed = map_radius_to_speed(
            radius,
            baseline_radius_inches=self.baseline_radius_inches,
            baseline_speed=self.baseline_speed,
            min_speed=self.min_speed,
            max_speed=self.max_speed,
        )

        self.get_logger().debug(
            f'pot raw={self.smoothed_raw:.1f}, radius={radius:.3f} in, speed={speed:.1f} steps/sec'
        )

        now = time.time()
        if now - self.last_publish_time < self.min_interval:
            return

        if self.last_published_speed is not None and abs(speed - self.last_published_speed) < self.speed_deadband:
            return

        setpoint_msg = Float32()
        setpoint_msg.data = float(speed)
        self.setpoint_pub.publish(setpoint_msg)

        if self.motor1_speed_pub:
            self.motor1_speed_pub.publish(setpoint_msg)
        if self.motor2_speed_pub:
            self.motor2_speed_pub.publish(setpoint_msg)

        self.last_published_speed = speed
        self.last_publish_time = now


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
