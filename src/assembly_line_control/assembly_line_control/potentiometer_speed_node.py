#!/usr/bin/env python3

"""
Potentiometer Speed Node for Assembly Line OS

Subscribes to /potentiometer/raw (from Arduino via arduino_controller),
scales motor speed by roll radius so material surface speed stays constant.

Calibration: empty roll OD od_min at pot_raw_min (default 40); full roll is
od_min + roll_od_growth_inches (default +13" OD) at pot_raw_max (1023).
Radius growth is half the OD growth, spread over (pot_raw_max - pot_raw_min)
ADC counts — not over the full 0–1023 range unless pot_raw_min is 0.
Assumes the dancer/pot is approximately linear in OD. The 1/R speed law uses
R0 = outer radius at pot_raw_min after min_radius_inches clamp (same as the
pot→radius map), not od_min/2 when that affine value lies below the floor.

Baseline speed is taken only from /motor_speed/baseline (Float32). The web UI
publishes that once per motor block together with the step command, so the pot
node does not listen to /motor1/speed or /motor2/speed (those are also driven by
"follow setpoint" mode and would self-feed the baseline). Parameter baseline_speed
applies until the first /motor_speed/baseline message. Learned baseline is capped
at max_speed to match the published setpoint clamp.
Publishes to /motor_speed/setpoint. Optionally publishes to /motor1/speed
and /motor2/speed (default: off) so the pot does not drive the motor at
startup until the user subscribes to the setpoint topic in the UI.

Smoothing (EMA) and a speed deadband reduce jitter from ADC noise.

Publishing runs on a timer (publish_rate_hz), not only on pot messages, so
/motor_speed/setpoint still updates if telemetry is slow or missing: until the
first pot sample, raw is assumed at pot_raw_min (empty roll). Optional keepalive
re-publishes periodically so rosbridge/UI keep seeing the topic.
"""

import math
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
    inverse_radius_exponent: float = 1.0,
) -> float:
    """
    Map roll radius to motor speed (steps/sec) so that surface speed
    remains approximately constant.

    For constant linear surface speed v at the roll:
        v = omega * R
      => omega ∝ 1 / R

    Since motor step rate is proportional to omega, we scale speed
    inversely with radius:

        speed(R) = baseline_speed * (R0 / R) ** inverse_radius_exponent

    exponent 1 = constant web speed; values in (0,1) soften taper (motor slows
    less as the roll grows).

    where:
      - R0 is the reference outer radius at pot_raw_min (after min_radius clamp),
        same as map_pot_to_radius(..., pot_raw_min) — not raw od_min/2 if that
        is below min_radius_inches.
      - R is current radius_inches
    """
    if radius_inches <= 0.0:
        # Should be prevented by min_radius_inches, but guard anyway.
        radius_inches = baseline_radius_inches

    ratio = baseline_radius_inches / radius_inches
    exp = float(inverse_radius_exponent)
    if exp <= 0.0:
        exp = 1.0
    speed = baseline_speed * (ratio ** exp)
    # Clamp within configured min/max to avoid extreme values.
    return max(min_speed, min(max_speed, speed))


class PotentiometerSpeedNode(Node):
    """Maps potentiometer raw value to motor speed and publishes to ROS."""

    def __init__(self):
        super().__init__('potentiometer_speed_node')

        self.declare_parameter('min_speed', 1.0)
        # Upper clamp on published setpoint and on learned baseline from /motor_speed/baseline
        self.declare_parameter('max_speed', 5000.0)
        # Roll OD vs pot: od_min at pot_raw_min; od_max = od_min + roll_od_growth_inches
        # (set od_max_inches > od_min to override the sum).
        self.declare_parameter('od_min_inches', 3.5)
        self.declare_parameter('roll_od_growth_inches', 13.0)  # added to OD from empty to full roll
        self.declare_parameter('od_max_inches', 0.0)  # 0 or <= od_min => od_min + roll_od_growth_inches
        self.declare_parameter('pot_raw_min', 40.0)  # ADC at empty 3.5" OD roll (mechanical zero)
        self.declare_parameter('pot_raw_max', 1023.0)  # ADC when OD has grown by roll_od_growth_inches
        self.declare_parameter('baseline_speed', 200.0)  # steps/sec until first /motor_speed/baseline
        self.declare_parameter('baseline_speed_topic', 'motor_speed/baseline')
        self.declare_parameter('min_radius_inches', 1.75)  # floor ≈ empty spool radius (3.5" OD / 2)
        self.declare_parameter('publish_motor1', False)
        self.declare_parameter('publish_motor2', False)
        self.declare_parameter('publish_rate_hz', 15.0)
        self.declare_parameter('smoothing_alpha', 0.6)
        self.declare_parameter('speed_deadband', 15.0)
        # Re-publish same setpoint at least this often (0 = disable) for live UIs / rosbridge
        self.declare_parameter('setpoint_keepalive_sec', 1.0)
        # 1.0 = speed ∝ 1/R; e.g. 0.85 slows the motor less as radius grows
        self.declare_parameter('inverse_radius_exponent', 1.0)

        self.min_speed = self.get_parameter('min_speed').value
        self.max_speed = self.get_parameter('max_speed').value
        od_min = float(self.get_parameter('od_min_inches').value)
        od_max = float(self.get_parameter('od_max_inches').value)
        roll_od_growth = float(self.get_parameter('roll_od_growth_inches').value)
        pot_raw_min = float(self.get_parameter('pot_raw_min').value)
        pot_raw_max = float(self.get_parameter('pot_raw_max').value)

        if od_max <= od_min + 1e-9:
            od_max = od_min + max(roll_od_growth, 1e-6)
        elif roll_od_growth > 0.0 and abs(od_max - (od_min + roll_od_growth)) > 0.02:
            self.get_logger().warn(
                f'od_max_inches ({od_max}) != od_min + roll_od_growth_inches ({od_min + roll_od_growth}); '
                f'using declared od_max_inches'
            )
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
        baseline_topic = self.get_parameter('baseline_speed_topic').value
        self._baseline_topic = (
            baseline_topic if isinstance(baseline_topic, str) and baseline_topic.strip()
            else 'motor_speed/baseline'
        )
        self.min_radius_inches = float(self.get_parameter('min_radius_inches').value)
        self.publish_motor1 = self.get_parameter('publish_motor1').value
        self.publish_motor2 = self.get_parameter('publish_motor2').value
        rate_hz = float(self.get_parameter('publish_rate_hz').value)
        if rate_hz <= 0.0:
            rate_hz = 15.0
        self.min_interval = 1.0 / rate_hz
        self.smoothing_alpha = self.get_parameter('smoothing_alpha').value
        self.speed_deadband = self.get_parameter('speed_deadband').value
        self.setpoint_keepalive_sec = float(self.get_parameter('setpoint_keepalive_sec').value)
        self.inverse_radius_exponent = float(self.get_parameter('inverse_radius_exponent').value)

        self.last_publish_time = 0.0
        self.smoothed_raw = None
        self.last_published_speed = None

        self.setpoint_pub = self.create_publisher(Float32, 'motor_speed/setpoint', 10)
        self.motor1_speed_pub = self.create_publisher(Float32, 'motor1/speed', 10) if self.publish_motor1 else None
        self.motor2_speed_pub = self.create_publisher(Float32, 'motor2/speed', 10) if self.publish_motor2 else None

        self.pot_sub = self.create_subscription(
            Float32, 'potentiometer/raw',
            self.pot_callback, 10)
        self.baseline_speed_sub = self.create_subscription(
            Float32, self._baseline_topic,
            self._baseline_speed_callback, 10)

        self._publish_timer = self.create_timer(self.min_interval, self._timer_publish_setpoint)

        adc_span = pot_raw_max - pot_raw_min
        dradius = (od_max - od_min) / 2.0
        r_at_min = map_pot_to_radius(
            pot_raw_min, self.baseline_raw, self.baseline_radius_inches, self.inches_per_count,
            self.min_radius_inches, self.pot_raw_min, self.pot_raw_max,
        )
        r_at_max = map_pot_to_radius(
            pot_raw_max, self.baseline_raw, self.baseline_radius_inches, self.inches_per_count,
            self.min_radius_inches, self.pot_raw_min, self.pot_raw_max,
        )
        # Speed law R0 must match the radius we actually use at empty-roll ADC (after min_radius floor).
        self.reference_radius_inches = r_at_min
        if self.baseline_radius_inches + 1e-9 < self.min_radius_inches:
            self.get_logger().warn(
                f'od_min/2 ({self.baseline_radius_inches:g}") is below min_radius_inches ({self.min_radius_inches:g}"); '
                f'using reference R0={self.reference_radius_inches:g}" for 1/R speed (fix od_min or min_radius)'
            )
        self.get_logger().info(
            f'Pot calibration: pot {self.pot_raw_min:.0f}–{self.pot_raw_max:.0f} (span {adc_span:.0f} counts) '
            f'=> OD {od_min}"–{od_max}" (+{od_max - od_min:g}" OD), '
            f'outer R {r_at_min:.3f}"–{r_at_max:.3f}" (+{dradius:g}" radius); '
            f'1/R R0={self.reference_radius_inches:.3f}"; '
            f'k={self.inches_per_count:.6f} in/ADC; 1/R exp={self.inverse_radius_exponent:g}'
        )
        self.get_logger().info(
            f'Potentiometer speed node: speed {self.min_speed}-{self.max_speed} sps, baseline from '
            f'{self._baseline_topic} (move blocks only), '
            f'motor1={self.publish_motor1}, motor2={self.publish_motor2}, '
            f'publish @{rate_hz:.1f} Hz, deadband={self.speed_deadband}, '
            f'keepalive={self.setpoint_keepalive_sec}s'
        )

    def _baseline_speed_callback(self, msg):
        """Baseline steps/sec from UI motor blocks only (/motor_speed/baseline)."""
        try:
            speed = float(msg.data)
        except (TypeError, ValueError):
            return
        if not math.isfinite(speed) or speed < self.min_speed:
            return
        self.baseline_speed = min(self.max_speed, speed)
        self.get_logger().info(
            f'Baseline speed set to {self.baseline_speed:.1f} sps from {self._baseline_topic} (motor block)'
        )

    def pot_callback(self, msg):
        raw = max(self.pot_raw_min, min(self.pot_raw_max, float(msg.data)))
        if self.smoothed_raw is None:
            self.smoothed_raw = raw
        else:
            self.smoothed_raw = (
                self.smoothing_alpha * raw + (1.0 - self.smoothing_alpha) * self.smoothed_raw
            )

    def _compute_setpoint_speed(self) -> float:
        raw = self.smoothed_raw if self.smoothed_raw is not None else self.pot_raw_min
        radius = map_pot_to_radius(
            raw,
            baseline_raw=self.baseline_raw,
            baseline_radius_inches=self.baseline_radius_inches,
            inches_per_count=self.inches_per_count,
            min_radius_inches=self.min_radius_inches,
            raw_clamp_min=self.pot_raw_min,
            raw_clamp_max=self.pot_raw_max,
        )
        return map_radius_to_speed(
            radius,
            baseline_radius_inches=self.reference_radius_inches,
            baseline_speed=self.baseline_speed,
            min_speed=self.min_speed,
            max_speed=self.max_speed,
            inverse_radius_exponent=self.inverse_radius_exponent,
        )

    def _timer_publish_setpoint(self):
        speed = self._compute_setpoint_speed()
        now = time.time()

        raw_dbg = (
            f'{self.smoothed_raw:.1f}'
            if self.smoothed_raw is not None
            else '— (bootstrap pot_raw_min)'
        )
        self.get_logger().debug(f'pot raw={raw_dbg}, speed={speed:.1f} steps/sec')

        need_publish = self.last_published_speed is None
        if not need_publish and abs(speed - self.last_published_speed) >= self.speed_deadband:
            need_publish = True
        if (
            not need_publish
            and self.setpoint_keepalive_sec > 0.0
            and (now - self.last_publish_time) >= self.setpoint_keepalive_sec
        ):
            need_publish = True

        if not need_publish:
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
