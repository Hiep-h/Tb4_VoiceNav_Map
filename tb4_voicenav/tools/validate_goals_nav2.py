import argparse
import json
import os
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import ComputePathToPose

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

from tb4_voicenav.segment_map import (  # noqa: E402
    draw_labeled_image,
    pixel_to_meters,
    top_clearance_candidates,
    segment_with_debug,
)


class GoalValidator(Node):
    def __init__(self):
        super().__init__('goal_validator')
        self.client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')

    def check_feasible(self, x, y, timeout_s=5.0):
        if not self.client.wait_for_server(timeout_sec=timeout_s):
            return False, 'server not available'
        goal = ComputePathToPose.Goal()
        goal.goal.header.frame_id = 'map'
        goal.goal.pose.position.x = x
        goal.goal.pose.position.y = y
        goal.goal.pose.orientation.w = 1.0
        goal.use_start = False

        send_future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=timeout_s)
        if not send_future.done():
            return False, 'send_goal timeout'
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return False, 'goal rejected'

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout_s)
        if not result_future.done():
            return False, 'result timeout'
        result = result_future.result().result
        num_poses = len(result.path.poses)
        if num_poses == 0:
            return False, 'empty path'
        return True, 'path_len=%d' % num_poses


def validate_and_update(map_yaml, out_dir, nav2_params, world_sdf, room_config, log_fn=print):
    debug = segment_with_debug(
        map_yaml, out_dir, nav2_params, world_sdf, room_config, write_outputs=False)
    regions = debug['regions']
    labels = debug['labels']
    clearance_dist = debug['clearance_dist']
    free_mask = debug['free_mask']
    resolution = debug['resolution']
    origin = debug['origin']
    height = debug['height']

    rclpy.init()
    node = GoalValidator()
    try:
        for region in regions:
            region_mask = (labels == region['id']) & free_mask
            centroid_px = (region['center_px'][0], region['center_px'][1])
            candidates = top_clearance_candidates(
                region_mask, clearance_dist, centroid_px, region['goal_radius_m_config'],
                resolution)
            min_clearance_m = region['min_required_clearance_m']
            ok_clearance = [c for c in candidates if c['clearance_m'] >= min_clearance_m]
            candidates = ok_clearance + [c for c in candidates if c not in ok_clearance]

            chosen = None
            for cand in candidates:
                x, y = pixel_to_meters(cand['px'], cand['py'], resolution, origin, height)
                ok, info = node.check_feasible(x, y)
                log_fn('%s candidate px=(%.0f,%.0f) m=(%.2f,%.2f) clearance=%.2fm -> %s (%s)' % (
                    region['name'], cand['px'], cand['py'], x, y, cand['clearance_m'],
                    'OK' if ok else 'FAIL', info))
                if ok:
                    chosen = (cand, x, y)
                    break

            if chosen is None:
                region['nav2_validated'] = False
                log_fn('%s: KHONG co diem dich kha thi trong cac ung vien da thu' % region['name'])
                continue

            cand, x, y = chosen
            region['goal_px'] = [cand['px'], cand['py']]
            region['goal_m'] = [x, y]
            region['goal_clearance_m'] = cand['clearance_m']
            region['nav2_validated'] = True
    finally:
        node.destroy_node()
        rclpy.shutdown()

    os.makedirs(out_dir, exist_ok=True)
    regions_json_path = os.path.join(out_dir, 'regions.json')
    with open(regions_json_path, 'w', encoding='utf-8') as f:
        json.dump({
            'map_yaml': os.path.abspath(map_yaml),
            'resolution': resolution,
            'origin': list(origin),
            'regions': regions,
        }, f, ensure_ascii=False, indent=2)

    image_out_path = os.path.join(out_dir, 'map_segmented.png')
    raw_image_out_path = os.path.join(out_dir, 'map_segmented_raw.png')
    draw_labeled_image(debug['occupancy'], labels, regions, image_out_path,
                       seeds=debug['seeds'], raw_out_path=raw_image_out_path)

    return regions, regions_json_path, image_out_path


def build_arg_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('--map-yaml', required=True)
    parser.add_argument('--out-dir', required=True)
    parser.add_argument(
        '--nav2-params', default='/opt/ros/humble/share/turtlebot4_navigation/config/nav2.yaml')
    parser.add_argument(
        '--world-sdf',
        default='/opt/ros/humble/share/turtlebot4_ignition_bringup/worlds/small_house.sdf')
    parser.add_argument('--room-config', default='config/room_furniture.yaml')
    return parser


def main():
    parser = build_arg_parser()
    args = parser.parse_args()
    t0 = time.time()
    regions, regions_json_path, image_out_path = validate_and_update(
        args.map_yaml, args.out_dir, args.nav2_params, args.world_sdf, args.room_config)
    for region in regions:
        print(region['name'], 'validated=%s' % region.get('nav2_validated'),
              'goal_m=%s' % region['goal_m'])
    print('elapsed_s=%.1f' % (time.time() - t0))
    print('wrote', regions_json_path)
    print('wrote', image_out_path)


if __name__ == '__main__':
    main()
