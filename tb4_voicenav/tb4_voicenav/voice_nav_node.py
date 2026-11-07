import functools
import os
import time

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray

from tb4_voicenav.command_parser import (
    NOT_COMMAND, UNKNOWN_REGION, normalize_text, parse_command, strip_diacritics)
from tb4_voicenav.region_store import RegionStore

DEFAULT_REGIONS_JSON = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', 'maps', 'regions.json')
MAP_FRAME_ID = 'map'
MIN_COMMAND_CHARS = 3  # bo qua am thanh vun vat nhu "di" tach rieng
PROGRESS_INTERVAL_S = 3.0


def build_goal(region):
    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id = MAP_FRAME_ID
    goal.pose.pose.position.x = float(region.goal_m[0])
    goal.pose.pose.position.y = float(region.goal_m[1])
    goal.pose.pose.orientation.w = 1.0
    return goal


class VoiceNavNode(Node):
    def __init__(self):
        super().__init__('voice_nav_node')
        self.declare_parameter('regions_json_path', DEFAULT_REGIONS_JSON)
        self.declare_parameter('fuzzy_match_threshold', 80)

        regions_json_path = self.get_parameter('regions_json_path').value or DEFAULT_REGIONS_JSON
        threshold = self.get_parameter('fuzzy_match_threshold').value / 100.0
        self.region_store = RegionStore(fuzzy_threshold=threshold).load(regions_json_path)

        self.goal_queue = []
        self.route = []
        self.visited_ids = set()
        self.current_target = None
        self.navigating = False
        self._last_progress_log = 0.0
        self._goal_seq = 0
        self.subscription = self.create_subscription(
            String, '/voice_text', self.on_voice_text, 10)
        self.status_publisher = self.create_publisher(String, '/voice_nav/status', 10)
        self.path_publisher = self.create_publisher(Path, '/voice_nav/path', 1)
        self.route_markers_publisher = self.create_publisher(
            MarkerArray, '/voice_nav/route_markers', 1)
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def publish_status(self, text):
        msg = String()
        msg.data = text
        self.status_publisher.publish(msg)
        self.get_logger().info(text)

    def on_voice_text(self, msg):
        now = self.get_clock().now().to_msg()
        self.get_logger().info(
            f"[/voice_text @ {now.sec}.{now.nanosec:09d}] '{msg.data}'")
        self.handle_command(msg.data)

    def handle_command(self, text):
        if len(normalize_text(text)) < MIN_COMMAND_CHARS:
            self.get_logger().info(f"Bo qua tin qua ngan/rong: '{text}'")
            return

        result = parse_command(text, self.region_store)
        if result.error == NOT_COMMAND:
            if self.navigating:
                self.get_logger().info(
                    f"Bo qua tin khong phai lenh trong luc dang di: '{text}'")
                return
            self.publish_status(f"Không hiểu lệnh: '{text}'")
            return
        if result.error == UNKNOWN_REGION:
            self.publish_status(f"Khu vực không tồn tại: '{result.error_detail}'")
            return

        self._goal_seq += 1
        self.route = list(result.targets)
        self.visited_ids = set()
        self.goal_queue = list(self.route)
        self.navigating = True
        self.publish_status(
            'Đã hiểu lệnh: đi qua ' + ' -> '.join(r.name for r in self.route))
        self.publish_status('ROUTE:' + '|'.join(r.name for r in self.route))
        self._publish_route_markers()
        self._send_next_goal()

    def _send_next_goal(self):
        if not self.goal_queue:
            self.current_target = None
            self.navigating = False
            self.publish_status('DONE')
            return
        region = self.goal_queue.pop(0)
        self.current_target = region
        self._last_progress_log = 0.0
        gen = self._goal_seq
        self.publish_status('GOTO:' + region.name)
        self.publish_status(f"Đang đi đến {region.name}...")
        send_future = self.action_client.send_goal_async(
            build_goal(region), feedback_callback=self._on_goal_feedback)
        send_future.add_done_callback(
            functools.partial(self._on_goal_response, gen=gen, target=region))

    def _on_goal_feedback(self, feedback_msg):
        if self.current_target is None:
            return
        now = time.time()
        if now - self._last_progress_log < PROGRESS_INTERVAL_S:
            return
        self._last_progress_log = now
        remaining = feedback_msg.feedback.distance_remaining
        self.publish_status(
            f"Còn khoảng {remaining:.1f} m đến {self.current_target.name}")

    def _on_goal_response(self, future, gen, target):
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            if gen == self._goal_seq:
                self.publish_status(f"Không đến được {target.name}")
                self.navigating = False
                self.current_target = None
                self.goal_queue = []
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            functools.partial(self._on_goal_result, gen=gen, target=target))

    def _on_goal_result(self, future, gen, target):
        if gen != self._goal_seq:
            # Goal nay da bi lenh moi thay the (preempt), bo qua ket qua cu.
            return
        result = future.result()
        if result is not None and result.status == GoalStatus.STATUS_SUCCEEDED:
            self.visited_ids.add(target.region_id)
            self.publish_status('ARRIVED:' + target.name)
            self.publish_status(f"Đã đến {target.name}")
            self._publish_route_markers()
            self._send_next_goal()
        else:
            self.publish_status(f"Không đến được {target.name}")
            self.navigating = False
            self.current_target = None
            self.goal_queue = []

    def _publish_route_markers(self):
        stamp = self.get_clock().now().to_msg()
        path = Path()
        path.header.frame_id = MAP_FRAME_ID
        path.header.stamp = stamp
        markers = MarkerArray()
        line = Marker()
        line.header.frame_id = MAP_FRAME_ID
        line.header.stamp = stamp
        line.ns = 'route_line'
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = 0.05
        line.color.r, line.color.g, line.color.b, line.color.a = 0.9, 0.5, 0.0, 1.0

        for idx, region in enumerate(self.route):
            pose = PoseStamped()
            pose.header.frame_id = MAP_FRAME_ID
            pose.header.stamp = stamp
            pose.pose.position.x = float(region.goal_m[0])
            pose.pose.position.y = float(region.goal_m[1])
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)

            point = Point()
            point.x = float(region.goal_m[0])
            point.y = float(region.goal_m[1])
            line.points.append(point)

            visited = region.region_id in self.visited_ids
            label = Marker()
            label.header.frame_id = MAP_FRAME_ID
            label.header.stamp = stamp
            label.ns = 'route_label'
            label.id = idx
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = float(region.goal_m[0])
            label.pose.position.y = float(region.goal_m[1])
            label.pose.position.z = 0.8
            label.scale.z = 0.35
            if visited:
                label.color.r, label.color.g, label.color.b, label.color.a = 0.2, 0.9, 0.2, 1.0
            else:
                label.color.r, label.color.g, label.color.b, label.color.a = 1.0, 0.8, 0.0, 1.0
            label.text = f'{idx + 1}. {strip_diacritics(region.name)}'
            markers.markers.append(label)

        markers.markers.append(line)
        self.path_publisher.publish(path)
        self.route_markers_publisher.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = VoiceNavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
