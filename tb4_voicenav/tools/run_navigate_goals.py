import argparse
import json
import time

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from tf2_ros import Buffer, TransformListener


class GoalRunner(Node):
    def __init__(self):
        super().__init__(
            'goal_runner',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.num_recoveries = 0

    def _feedback_cb(self, feedback_msg):
        self.num_recoveries = feedback_msg.feedback.number_of_recoveries

    def robot_pose_in_map(self):
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            return tf.transform.translation.x, tf.transform.translation.y
        except Exception as exc:
            self.get_logger().warn('tf lookup failed: %s' % exc)
            return None

    def _wait_sim_seconds(self, min_sim_s):
        t0_wall = time.time()
        sim_t0 = self.get_clock().now()
        while True:
            rclpy.spin_once(self, timeout_sec=0.2)
            sim_elapsed_s = (self.get_clock().now() - sim_t0).nanoseconds / 1e9
            if sim_elapsed_s >= min_sim_s or (time.time() - t0_wall) > min_sim_s * 20 + 30:
                return

    def run_goal(self, x, y, max_sim_time_s, max_wall_time_s):
        self.num_recoveries = 0
        if not self.client.wait_for_server(timeout_sec=10.0):
            return {'status': 'server_unavailable'}

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.w = 1.0

        wall_t0 = time.time()
        sim_t0 = self.get_clock().now()
        send_future = self.client.send_goal_async(goal, feedback_callback=self._feedback_cb)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        if not send_future.done():
            return {'status': 'send_goal_timeout', 'wall_elapsed_s': time.time() - wall_t0}
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return {'status': 'goal_rejected', 'wall_elapsed_s': time.time() - wall_t0}

        result_future = goal_handle.get_result_async()
        while not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.2)
            sim_elapsed_s = (self.get_clock().now() - sim_t0).nanoseconds / 1e9
            wall_elapsed_s = time.time() - wall_t0
            if sim_elapsed_s >= max_sim_time_s or wall_elapsed_s >= max_wall_time_s:
                break

        sim_elapsed_s = (self.get_clock().now() - sim_t0).nanoseconds / 1e9
        wall_elapsed_s = time.time() - wall_t0
        if not result_future.done():
            cancel_future = goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=10.0)
            pose = self.robot_pose_in_map()
            return {
                'status': 'sim_time_timeout' if sim_elapsed_s >= max_sim_time_s
                else 'wall_time_timeout',
                'sim_elapsed_s': sim_elapsed_s, 'wall_elapsed_s': wall_elapsed_s,
                'num_recoveries': self.num_recoveries, 'robot_pose_m': pose,
            }

        wrapped = result_future.result()
        pose = self.robot_pose_in_map()
        error_m = None
        if pose is not None:
            error_m = ((pose[0] - x) ** 2 + (pose[1] - y) ** 2) ** 0.5
        status_name = {
            GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
            GoalStatus.STATUS_ABORTED: 'ABORTED',
            GoalStatus.STATUS_CANCELED: 'CANCELED',
        }.get(wrapped.status, 'STATUS_%d' % wrapped.status)
        return {
            'status': status_name, 'sim_elapsed_s': sim_elapsed_s,
            'wall_elapsed_s': wall_elapsed_s,
            'num_recoveries': self.num_recoveries, 'robot_pose_m': pose,
            'position_error_m': error_m,
        }


def run_all(regions_json_path, max_sim_time_s, max_wall_time_s, log_fn=print):
    with open(regions_json_path, 'r', encoding='utf-8') as f:
        data = json.load(f)

    rclpy.init()
    node = GoalRunner()
    results = []
    try:
        node._wait_sim_seconds(0.1)
        for region in data['regions']:
            x, y = region['goal_m']
            wall_t0 = time.time()
            outcome = node.run_goal(x, y, max_sim_time_s, max_wall_time_s)
            outcome['name'] = region['name']
            outcome['goal_m'] = [x, y]
            results.append(outcome)
            log_fn('%s goal=(%.2f,%.2f) -> %s' % (region['name'], x, y, json.dumps(outcome)))
            log_fn('wall_elapsed_since_start=%.1fs' % (time.time() - wall_t0))
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return results


def build_arg_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('--regions-json', required=True)
    parser.add_argument('--max-sim-time-s', type=float, default=180.0)
    parser.add_argument('--max-wall-time-s', type=float, default=1200.0)
    parser.add_argument('--out-json', default=None)
    return parser


def main():
    parser = build_arg_parser()
    args = parser.parse_args()
    results = run_all(args.regions_json, args.max_sim_time_s, args.max_wall_time_s)
    if args.out_json:
        with open(args.out_json, 'w', encoding='utf-8') as f:
            json.dump(results, f, ensure_ascii=False, indent=2)
        print('wrote', args.out_json)


if __name__ == '__main__':
    main()
