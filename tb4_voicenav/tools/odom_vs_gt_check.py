import json
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy


def yaw_from_quat(z, w):
    return math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)


def wrap_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


class OdomGtCheck(Node):
    def __init__(self):
        super().__init__(
            'odom_gt_check',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        best_effort_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST, depth=10)
        self.odom = None
        self.gt = None
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.create_subscription(Odometry, '/sim_ground_truth_pose', self._gt_cb, best_effort_qos)
        self.samples = []

    def _odom_cb(self, msg):
        self.odom = msg

    def _gt_cb(self, msg):
        self.gt = msg

    def spin_until_ready(self, timeout_s=10.0):
        t0 = time.time()
        while (self.odom is None or self.gt is None) and time.time() - t0 < timeout_s:
            rclpy.spin_once(self, timeout_sec=0.1)

    def spin_for(self, sim_seconds, cmd=None, sample_interval_s=0.5, ramp_s=0.0):
        t0 = self.get_clock().now()
        last_sample = -sample_interval_s
        while (self.get_clock().now() - t0).nanoseconds / 1e9 < sim_seconds:
            elapsed = (self.get_clock().now() - t0).nanoseconds / 1e9
            if cmd is not None:
                if ramp_s > 0.0 and elapsed < ramp_s:
                    scaled = Twist()
                    frac = elapsed / ramp_s
                    scaled.linear.x = cmd.linear.x * frac
                    scaled.angular.z = cmd.angular.z * frac
                    self.cmd_pub.publish(scaled)
                else:
                    self.cmd_pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.05)
            elapsed = (self.get_clock().now() - t0).nanoseconds / 1e9
            if elapsed - last_sample >= sample_interval_s:
                last_sample = elapsed
                self._sample(elapsed)

    def _sample(self, t_s):
        o = None
        g = None
        if self.odom is not None:
            p = self.odom.pose.pose
            o = {'x': p.position.x, 'y': p.position.y,
                 'yaw': yaw_from_quat(p.orientation.z, p.orientation.w)}
        if self.gt is not None:
            p = self.gt.pose.pose
            g = {'x': p.position.x, 'y': p.position.y,
                 'yaw': yaw_from_quat(p.orientation.z, p.orientation.w)}
        self.samples.append({'t': t_s, 'odom': o, 'ground_truth': g})

    def stop(self):
        self.cmd_pub.publish(Twist())
        self.spin_for(1.0)


def run(out_json, log_fn=print):
    rclpy.init()
    node = OdomGtCheck()
    try:
        node.spin_until_ready(10.0)
        node.spin_for(1.0)
        start_o = node.samples[-1]['odom'] if node.samples else None
        start_g = node.samples[-1]['ground_truth'] if node.samples else None
        log_fn('start odom=%s ground_truth=%s' % (start_o, start_g))

        forward_speed = 0.15
        forward_duration_s = 20.0
        fwd_cmd = Twist()
        fwd_cmd.linear.x = forward_speed
        node.spin_for(forward_duration_s, cmd=fwd_cmd, ramp_s=2.0)
        node.stop()
        after_fwd_o = node.samples[-1]['odom']
        after_fwd_g = node.samples[-1]['ground_truth']
        log_fn('after forward odom=%s ground_truth=%s' % (after_fwd_o, after_fwd_g))

        angular_speed = 0.3
        rotate_duration_s = (2.0 * math.pi) / angular_speed
        rot_cmd = Twist()
        rot_cmd.angular.z = angular_speed
        node.spin_for(rotate_duration_s, cmd=rot_cmd, ramp_s=2.0)
        node.stop()
        after_rot_o = node.samples[-1]['odom']
        after_rot_g = node.samples[-1]['ground_truth']
        log_fn('after rotation odom=%s ground_truth=%s' % (after_rot_o, after_rot_g))

        odom_fwd_dist = math.hypot(
            after_fwd_o['x'] - start_o['x'], after_fwd_o['y'] - start_o['y'])
        gt_fwd_dist = math.hypot(
            after_fwd_g['x'] - start_g['x'], after_fwd_g['y'] - start_g['y'])
        fwd_ratio = odom_fwd_dist / gt_fwd_dist if gt_fwd_dist > 1e-6 else None

        odom_yaw_delta = wrap_angle(after_rot_o['yaw'] - after_fwd_o['yaw'])
        gt_yaw_delta = wrap_angle(after_rot_g['yaw'] - after_fwd_g['yaw'])
        odom_final_pos_delta = math.hypot(
            after_rot_o['x'] - after_fwd_o['x'], after_rot_o['y'] - after_fwd_o['y'])
        gt_final_pos_delta = math.hypot(
            after_rot_g['x'] - after_fwd_g['x'], after_rot_g['y'] - after_fwd_g['y'])

        report = {
            'forward_speed_mps': forward_speed,
            'forward_duration_s': forward_duration_s,
            'angular_speed_radps': angular_speed,
            'rotate_duration_s': rotate_duration_s,
            'start_odom': start_o, 'start_ground_truth': start_g,
            'after_forward_odom': after_fwd_o, 'after_forward_ground_truth': after_fwd_g,
            'after_rotation_odom': after_rot_o, 'after_rotation_ground_truth': after_rot_g,
            'odom_forward_distance_m': odom_fwd_dist,
            'ground_truth_forward_distance_m': gt_fwd_dist,
            'forward_distance_ratio_odom_over_gt': fwd_ratio,
            'odom_yaw_delta_after_one_turn_rad': odom_yaw_delta,
            'ground_truth_yaw_delta_after_one_turn_rad': gt_yaw_delta,
            'odom_position_delta_during_turn_m': odom_final_pos_delta,
            'ground_truth_position_delta_during_turn_m': gt_final_pos_delta,
            'num_samples': len(node.samples),
        }
        with open(out_json, 'w', encoding='utf-8') as f:
            json.dump({'report': report, 'samples': node.samples}, f, ensure_ascii=False, indent=2)
        log_fn(json.dumps(report, ensure_ascii=False, indent=2))
        log_fn('wrote %s' % out_json)
        return report
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    import sys
    run(sys.argv[1] if len(sys.argv) > 1 else '/tmp/tb4_odom_test/report.json')
