import os
import tempfile

import numpy as np

from tb4_voicenav.segment_map import (
    meters_to_pixel,
    pixel_to_meters,
    segment,
)
from tb4_voicenav.region_store import RegionStore

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MAP_YAML = os.path.join(PKG_DIR, 'maps', 'map.yaml')
NAV2_PARAMS = '/opt/ros/humble/share/turtlebot4_navigation/config/nav2.yaml'
WORLD_SDF = '/opt/ros/humble/share/turtlebot4_ignition_bringup/worlds/small_house.sdf'
ROOM_CONFIG = os.path.join(PKG_DIR, 'config', 'room_furniture.yaml')


def test_pixel_meter_round_trip():
    resolution = 0.05
    origin = (-10.4, -6.63)
    height = 224
    px, py = 123.0, 45.0
    x, y = pixel_to_meters(px, py, resolution, origin, height)
    px2, py2 = meters_to_pixel(x, y, resolution, origin, height)
    assert abs(px2 - px) < 1e-6
    assert abs(py2 - py) < 1e-6


def test_pixel_meter_known_point():
    resolution = 0.05
    origin = (-10.4, -6.63)
    height = 224
    x, y = pixel_to_meters(0, height - 1, resolution, origin, height)
    assert abs(x - origin[0]) < 1e-9
    assert abs(y - origin[1]) < 1e-9


def _run_segmentation(tmp_dir):
    return segment(MAP_YAML, tmp_dir, NAV2_PARAMS, WORLD_SDF, ROOM_CONFIG)


def test_segment_region_count_is_three():
    with tempfile.TemporaryDirectory() as tmp_dir:
        regions, _, _, _ = _run_segmentation(tmp_dir)
        assert len(regions) == 3
        names = {region['name'] for region in regions}
        assert names == {'Phòng ngủ', 'Phòng khách', 'Phòng bếp'}


def test_segment_goal_points_meet_clearance():
    with tempfile.TemporaryDirectory() as tmp_dir:
        regions, _, _, _ = _run_segmentation(tmp_dir)
        for region in regions:
            assert region['goal_clearance_m'] >= region['min_required_clearance_m']


def test_segment_regions_json_round_trip():
    with tempfile.TemporaryDirectory() as tmp_dir:
        regions, _, regions_json_path, _ = _run_segmentation(tmp_dir)
        store = RegionStore().load(regions_json_path)
        assert len(store.regions) == len(regions)
        for region in regions:
            loaded = store.get(region['id'])
            assert loaded is not None
            assert loaded.name == region['name']


def test_segment_regions_do_not_overlap():
    with tempfile.TemporaryDirectory() as tmp_dir:
        regions, labels, _, _ = _run_segmentation(tmp_dir)
        label_counts = {}
        for region in regions:
            mask = labels == region['id']
            label_counts[region['id']] = int(np.sum(mask))
        total_labeled = sum(label_counts.values())
        unique_labels_px = int(np.sum(np.isin(labels, [r['id'] for r in regions])))
        assert total_labeled == unique_labels_px
