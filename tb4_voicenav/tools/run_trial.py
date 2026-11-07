import argparse
import json
import math
import time

import cv2
import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from tf2_ros import Buffer, TransformListener

SPAWN_WORLD_X = 1.0
SPAWN_WORLD_Y = 1.0


def world_to_map(x, y):
    return x - SPAWN_WORLD_X, y - SPAWN_WORLD_Y


def yaw_from_quat(z, w):
    return math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)


class TrialNode(Node):
    def __init__(self):
        super().__init__(
            'trial_node',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.path_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        self.num_recoveries = 0
        self.ground_truth = None
        self.odom = None
        self.global_costmap = None
        self.local_costmap = None
        best_effort_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(
            Odometry, '/sim_ground_truth_pose', self._gt_cb, best_effort_qos)
        self.create_subscription(Odometry, '/odom', self._odom_cb, best_effort_qos)
        self.create_subscription(
            OccupancyGrid, '/global_costmap/costmap', self._global_cb, 1)
        self.create_subscription(OccupancyGrid, '/local_costmap/costmap', self._local_cb, 1)
        self.pose_history = []

    def _gt_cb(self, msg):
        self.ground_truth = msg

    def _odom_cb(self, msg):
        self.odom = msg

    def _global_cb(self, msg):
        self.global_costmap = msg

    def _local_cb(self, msg):
        self.local_costmap = msg

    def _feedback_cb(self, feedback_msg):
        self.num_recoveries = feedback_msg.feedback.number_of_recoveries

    def spin_for(self, sim_seconds):
        t0 = self.get_clock().now()
        while (self.get_clock().now() - t0).nanoseconds / 1e9 < sim_seconds:
            rclpy.spin_once(self, timeout_sec=0.1)

    def amcl_pose_map(self):
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            x = tf.transform.translation.x
            y = tf.transform.translation.y
            yaw = yaw_from_quat(tf.transform.rotation.z, tf.transform.rotation.w)
            return x, y, yaw
        except Exception as exc:
            self.get_logger().warn('amcl tf lookup failed: %s' % exc)
            return None

    def ground_truth_pose_map(self):
        if self.ground_truth is None:
            return None
        p = self.ground_truth.pose.pose
        gx, gy = world_to_map(p.position.x, p.position.y)
        yaw = yaw_from_quat(p.orientation.z, p.orientation.w)
        return gx, gy, yaw

    def odom_pose_map(self):
        if self.odom is None:
            return None
        p = self.odom.pose.pose
        yaw = yaw_from_quat(p.orientation.z, p.orientation.w)
        return p.position.x, p.position.y, yaw

    def publish_initialpose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.orientation.w = 1.0
        cov = [0.0] * 36
        cov[0] = 0.25
        cov[7] = 0.25
        cov[35] = 0.07
        msg.pose.covariance = cov
        for _ in range(8):
            self.initialpose_pub.publish(msg)
            self.spin_for(1.0)

    def compute_path(self, x, y, timeout_s=15.0):
        if not self.path_client.wait_for_server(timeout_sec=timeout_s):
            return {'ok': False, 'reason': 'server_unavailable'}
        goal = ComputePathToPose.Goal()
        goal.goal.header.frame_id = 'map'
        goal.goal.pose.position.x = x
        goal.goal.pose.position.y = y
        goal.goal.pose.orientation.w = 1.0
        goal.use_start = False
        send_future = self.path_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=timeout_s)
        if not send_future.done():
            return {'ok': False, 'reason': 'send_goal_timeout'}
        handle = send_future.result()
        if handle is None or not handle.accepted:
            return {'ok': False, 'reason': 'goal_rejected'}
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout_s)
        if not result_future.done():
            return {'ok': False, 'reason': 'result_timeout'}
        result = result_future.result().result
        poses = result.path.poses
        if len(poses) == 0:
            return {'ok': False, 'reason': 'empty_path'}
        length_m = 0.0
        for i in range(1, len(poses)):
            dx = poses[i].pose.position.x - poses[i - 1].pose.position.x
            dy = poses[i].pose.position.y - poses[i - 1].pose.position.y
            length_m += math.hypot(dx, dy)
        return {'ok': True, 'num_poses': len(poses), 'path_length_m': length_m}

    def run_goal(self, x, y, max_sim_time_s, max_wall_time_s, sample_interval_s=10.0):
        self.num_recoveries = 0
        self.pose_history = []
        if not self.nav_client.wait_for_server(timeout_sec=15.0):
            return {'status': 'server_unavailable'}

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.w = 1.0

        wall_t0 = time.time()
        sim_t0 = self.get_clock().now()
        send_future = self.nav_client.send_goal_async(goal, feedback_callback=self._feedback_cb)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=15.0)
        if not send_future.done():
            return {'status': 'send_goal_timeout', 'wall_elapsed_s': time.time() - wall_t0}
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return {'status': 'goal_rejected', 'wall_elapsed_s': time.time() - wall_t0}

        result_future = goal_handle.get_result_async()
        last_sample_s = -sample_interval_s
        while not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            sim_elapsed_s = (self.get_clock().now() - sim_t0).nanoseconds / 1e9
            wall_elapsed_s = time.time() - wall_t0
            if sim_elapsed_s - last_sample_s >= sample_interval_s:
                last_sample_s = sim_elapsed_s
                amcl = self.amcl_pose_map()
                gt = self.ground_truth_pose_map()
                odom = self.odom_pose_map()
                self.pose_history.append({
                    'sim_t': sim_elapsed_s, 'amcl': amcl, 'ground_truth': gt, 'odom': odom,
                })
            if sim_elapsed_s >= max_sim_time_s or wall_elapsed_s >= max_wall_time_s:
                break

        sim_elapsed_s = (self.get_clock().now() - sim_t0).nanoseconds / 1e9
        wall_elapsed_s = time.time() - wall_t0
        timed_out = not result_future.done()
        if timed_out:
            cancel_future = goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=15.0)
            cancel_result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, cancel_result_future, timeout_sec=15.0)

        amcl = self.amcl_pose_map()
        gt = self.ground_truth_pose_map()
        odom = self.odom_pose_map()
        error_vs_goal_m = None
        gt_error_vs_goal_m = None
        amcl_vs_gt_error_m = None
        odom_vs_gt_error_m = None
        if amcl is not None:
            error_vs_goal_m = math.hypot(amcl[0] - x, amcl[1] - y)
        if gt is not None:
            gt_error_vs_goal_m = math.hypot(gt[0] - x, gt[1] - y)
        if amcl is not None and gt is not None:
            amcl_vs_gt_error_m = math.hypot(amcl[0] - gt[0], amcl[1] - gt[1])
        if odom is not None and gt is not None:
            odom_vs_gt_error_m = math.hypot(odom[0] - gt[0], odom[1] - gt[1])

        status = 'sim_time_timeout' if timed_out and sim_elapsed_s >= max_sim_time_s \
            else 'wall_time_timeout' if timed_out else None
        if not timed_out:
            wrapped = result_future.result()
            status = {
                GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
                GoalStatus.STATUS_ABORTED: 'ABORTED',
                GoalStatus.STATUS_CANCELED: 'CANCELED',
            }.get(wrapped.status, 'STATUS_%d' % wrapped.status)

        return {
            'status': status, 'sim_elapsed_s': sim_elapsed_s, 'wall_elapsed_s': wall_elapsed_s,
            'num_recoveries': self.num_recoveries,
            'final_amcl_pose_map': amcl, 'final_ground_truth_pose_map': gt,
            'final_odom_pose_map': odom,
            'position_error_vs_goal_m': error_vs_goal_m,
            'ground_truth_error_vs_goal_m': gt_error_vs_goal_m,
            'amcl_vs_ground_truth_error_m': amcl_vs_gt_error_m,
            'odom_vs_ground_truth_error_m': odom_vs_gt_error_m,
            'pose_history': self.pose_history,
        }

    def save_costmap_image(self, costmap, out_path, robot_amcl=None, robot_gt=None, goal=None):
        if costmap is None:
            return False
        w, h = costmap.info.width, costmap.info.height
        res = costmap.info.resolution
        ox, oy = costmap.info.origin.position.x, costmap.info.origin.position.y
        data = np.array(costmap.data, dtype=np.int16).reshape((h, w))
        img = np.zeros((h, w, 3), dtype=np.uint8)
        img[data == -1] = (120, 120, 120)
        free_or_cost = data >= 0
        scaled = np.clip(255 - (data.astype(np.float32) * 2.55), 0, 255).astype(np.uint8)
        img[free_or_cost] = np.stack(
            [scaled[free_or_cost]] * 3, axis=-1)
        img[data >= 99] = (0, 0, 180)
        img = np.ascontiguousarray(np.flipud(img))

        def to_px(x, y):
            px = int(round((x - ox) / res))
            py = int(round((y - oy) / res))
            return px, h - 1 - py

        if robot_gt is not None:
            cv2.circle(img, to_px(robot_gt[0], robot_gt[1]), 4, (0, 220, 0), -1)
        if robot_amcl is not None:
            cv2.circle(img, to_px(robot_amcl[0], robot_amcl[1]), 4, (255, 120, 0), 1)
        if goal is not None:
            cv2.drawMarker(img, to_px(goal[0], goal[1]), (0, 0, 255), cv2.MARKER_CROSS, 10, 2)
        cv2.imwrite(out_path, img)
        return True


def run_trial(name, goal_x, goal_y, max_sim_time_s, max_wall_time_s, out_prefix,
              log_fn=print):
    rclpy.init()
    node = TrialNode()
    try:
        node.spin_for(1.0)
        node.publish_initialpose()
        node.spin_for(3.0)

        start_amcl = node.amcl_pose_map()
        start_gt = node.ground_truth_pose_map()
        start_odom = node.odom_pose_map()
        start_error_m = None
        if start_amcl is not None and start_gt is not None:
            start_error_m = math.hypot(
                start_amcl[0] - start_gt[0], start_amcl[1] - start_gt[1])
        log_fn('%s start: amcl=%s ground_truth=%s odom=%s error_m=%s' % (
            name, start_amcl, start_gt, start_odom, start_error_m))

        path_check = node.compute_path(goal_x, goal_y)
        log_fn('%s compute_path_to_pose: %s' % (name, json.dumps(path_check)))

        result = node.run_goal(goal_x, goal_y, max_sim_time_s, max_wall_time_s)
        log_fn('%s run_goal result: %s' % (name, json.dumps(
            {k: v for k, v in result.items() if k != 'pose_history'})))

        node.spin_for(0.5)
        node.save_costmap_image(
            node.global_costmap, out_prefix + '_global_costmap.png',
            robot_amcl=result.get('final_amcl_pose_map'),
            robot_gt=result.get('final_ground_truth_pose_map'),
            goal=(goal_x, goal_y))
        node.save_costmap_image(
            node.local_costmap, out_prefix + '_local_costmap.png',
            robot_amcl=result.get('final_amcl_pose_map'),
            robot_gt=result.get('final_ground_truth_pose_map'),
            goal=(goal_x, goal_y))

        output = {
            'name': name, 'goal_m': [goal_x, goal_y],
            'start_amcl_pose_map': start_amcl, 'start_ground_truth_pose_map': start_gt,
            'start_odom_pose_map': start_odom,
            'start_localization_error_m': start_error_m,
            'compute_path_to_pose': path_check,
            'result': result,
        }
        with open(out_prefix + '_result.json', 'w', encoding='utf-8') as f:
            json.dump(output, f, ensure_ascii=False, indent=2)
        log_fn('wrote %s_result.json' % out_prefix)
        return output
    finally:
        node.destroy_node()
        rclpy.shutdown()


def build_arg_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('--name', required=True)
    parser.add_argument('--goal-x', type=float, required=True)
    parser.add_argument('--goal-y', type=float, required=True)
    parser.add_argument('--max-sim-time-s', type=float, default=180.0)
    parser.add_argument('--max-wall-time-s', type=float, default=600.0)
    parser.add_argument('--out-prefix', required=True)
    return parser


def main():
    parser = build_arg_parser()
    args = parser.parse_args()
    run_trial(
        args.name, args.goal_x, args.goal_y, args.max_sim_time_s, args.max_wall_time_s,
        args.out_prefix)


if __name__ == '__main__':
    main()
