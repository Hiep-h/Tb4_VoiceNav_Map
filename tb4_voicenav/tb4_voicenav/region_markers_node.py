import os

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

from tb4_voicenav.region_store import RegionStore

DEFAULT_REGIONS_JSON = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', 'maps', 'regions.json')


class RegionMarkersNode(Node):
    def __init__(self):
        super().__init__('region_markers_node')
        self.declare_parameter('regions_json_path', DEFAULT_REGIONS_JSON)
        regions_json_path = self.get_parameter('regions_json_path').value or DEFAULT_REGIONS_JSON
        self.region_store = RegionStore().load(regions_json_path)
        self.publisher = self.create_publisher(MarkerArray, '/voice_nav/region_markers', 1)
        self.timer = self.create_timer(1.0, self.publish_markers)

    def publish_markers(self):
        stamp = self.get_clock().now().to_msg()
        array = MarkerArray()
        for region in self.region_store.regions.values():
            goal = Marker()
            goal.header.frame_id = 'map'
            goal.header.stamp = stamp
            goal.ns = 'region_goal'
            goal.id = region.region_id
            goal.type = Marker.SPHERE
            goal.action = Marker.ADD
            goal.pose.position.x = float(region.goal_m[0])
            goal.pose.position.y = float(region.goal_m[1])
            goal.pose.orientation.w = 1.0
            goal.scale.x = goal.scale.y = goal.scale.z = 0.3
            goal.color.r, goal.color.g, goal.color.b, goal.color.a = 0.1, 0.6, 1.0, 1.0
            array.markers.append(goal)

            label = Marker()
            label.header.frame_id = 'map'
            label.header.stamp = stamp
            label.ns = 'region_label'
            label.id = region.region_id
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = float(region.goal_m[0])
            label.pose.position.y = float(region.goal_m[1])
            label.pose.position.z = 0.5
            label.scale.z = 0.4
            label.color.r, label.color.g, label.color.b, label.color.a = 1.0, 1.0, 1.0, 1.0
            label.text = region.name
            array.markers.append(label)
        self.publisher.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = RegionMarkersNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
