import argparse
import heapq
import json
import math
import os
import re
import unicodedata

import cv2
import numpy as np
import yaml
from PIL import Image, ImageDraw, ImageFont

_VN_FONT_PATH = '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'

FREE = 0
OCCUPIED = 1
UNKNOWN = 2

ROOM_DISPLAY_NAMES = {
    'Phong ngu': 'Phòng ngủ',
    'Phong khach': 'Phòng khách',
    'Phong bep': 'Phòng bếp',
}

SPAWN_WORLD_OFFSET_M = (1.0, 1.0)

_SQRT2 = math.sqrt(2.0)
_NEIGHBORS = [
    (-1, -1, _SQRT2), (0, -1, 1.0), (1, -1, _SQRT2),
    (-1, 0, 1.0), (1, 0, 1.0),
    (-1, 1, _SQRT2), (0, 1, 1.0), (1, 1, _SQRT2),
]


def strip_diacritics(text):
    text = text.replace('đ', 'd').replace('Đ', 'D')
    normalized = unicodedata.normalize('NFD', text)
    return ''.join(c for c in normalized if unicodedata.category(c) != 'Mn')


def load_map(map_yaml_path):
    with open(map_yaml_path, 'r', encoding='utf-8') as f:
        meta = yaml.safe_load(f)
    map_dir = os.path.dirname(os.path.abspath(map_yaml_path))
    image_path = os.path.join(map_dir, meta['image'])
    img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise FileNotFoundError(image_path)
    negate = bool(meta.get('negate', 0))
    occupied_thresh = float(meta['occupied_thresh'])
    free_thresh = float(meta['free_thresh'])
    if negate:
        value = img.astype(np.float32) / 255.0
    else:
        value = (255.0 - img.astype(np.float32)) / 255.0
    occupancy = np.full(img.shape, UNKNOWN, dtype=np.uint8)
    occupancy[value > occupied_thresh] = OCCUPIED
    occupancy[value < free_thresh] = FREE
    return {
        'occupancy': occupancy,
        'resolution': float(meta['resolution']),
        'origin': tuple(meta['origin'][:2]),
        'height': img.shape[0],
        'width': img.shape[1],
    }


def pixel_to_meters(px, py, resolution, origin, height):
    x = origin[0] + px * resolution
    y = origin[1] + (height - 1 - py) * resolution
    return x, y


def meters_to_pixel(x, y, resolution, origin, height):
    px = (x - origin[0]) / resolution
    py = (height - 1) - (y - origin[1]) / resolution
    return px, py


def find_nav2_inflation_radius(nav2_params_path):
    with open(nav2_params_path, 'r', encoding='utf-8') as f:
        text = f.read()
    match = re.search(r'inflation_radius:\s*([0-9.]+)', text)
    if not match:
        raise ValueError('inflation_radius not found in ' + nav2_params_path)
    return float(match.group(1))


def parse_world_objects(world_sdf_path):
    text = open(world_sdf_path, 'r', encoding='utf-8').read()
    objects = {}
    for block in re.findall(r'<include>(.*?)</include>', text, re.S):
        name_m = re.search(r'<name>([^<]+)</name>', block)
        pose_m = re.search(r'<pose[^>]*>([^<]+)</pose>', block)
        if not (name_m and pose_m):
            continue
        vals = pose_m.group(1).split()
        objects[name_m.group(1)] = (float(vals[0]), float(vals[1]))
    for match in re.finditer(r'<model name=[\'"]([^\'"]+)[\'"][^>]*>(.*?)</model>', text, re.S):
        name, body = match.groups()
        if name in objects:
            continue
        pose_m = re.search(r'<pose[^>]*>([^<]+)</pose>', body)
        if not pose_m:
            continue
        vals = pose_m.group(1).split()
        objects[name] = (float(vals[0]), float(vals[1]))
    ox, oy = SPAWN_WORLD_OFFSET_M
    return {name: (x - ox, y - oy) for name, (x, y) in objects.items()}


def load_room_furniture_config(room_config_path):
    with open(room_config_path, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f)
    return data['rooms']


def furniture_centroid_m(model_names, world_objects):
    pts = [world_objects[name] for name in model_names]
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    return (sum(xs) / len(xs), sum(ys) / len(ys))


def nearest_free_pixel(free_mask, px, py, max_radius=200):
    h, w = free_mask.shape
    px_i, py_i = int(round(px)), int(round(py))
    if 0 <= py_i < h and 0 <= px_i < w and free_mask[py_i, px_i]:
        return px_i, py_i
    for r in range(1, max_radius + 1):
        best, best_d2 = None, None
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                if max(abs(dx), abs(dy)) != r:
                    continue
                nx, ny = px_i + dx, py_i + dy
                if 0 <= ny < h and 0 <= nx < w and free_mask[ny, nx]:
                    d2 = dx * dx + dy * dy
                    if best_d2 is None or d2 < best_d2:
                        best_d2, best = d2, (nx, ny)
        if best is not None:
            return best
    raise ValueError(
        'khong tim thay pixel free gan (%d,%d) trong ban kinh %d px' % (px_i, py_i, max_radius))


def build_seeds(rooms_cfg, world_objects, free_mask, resolution, origin, height):
    seeds = []
    for label, room in enumerate(rooms_cfg, start=1):
        for model_name in room['model_names']:
            if model_name not in world_objects:
                raise KeyError('model "%s" khong co trong world SDF' % model_name)
            x, y = world_objects[model_name]
            raw_px, raw_py = meters_to_pixel(x, y, resolution, origin, height)
            snap_px, snap_py = nearest_free_pixel(free_mask, raw_px, raw_py)
            seeds.append({
                'label': label,
                'model_name': model_name,
                'point_m': [x, y],
                'raw_px': [raw_px, raw_py],
                'px': snap_px,
                'py': snap_py,
                'snapped': (int(round(raw_px)), int(round(raw_py))) != (snap_px, snap_py),
            })
    return seeds


def geodesic_partition(free_mask, seeds):
    h, w = free_mask.shape
    dist = np.full((h, w), np.inf, dtype=np.float64)
    labels = np.zeros((h, w), dtype=np.int32)
    heap = []
    for seed in seeds:
        px, py, label = seed['px'], seed['py'], seed['label']
        if dist[py, px] > 0.0:
            dist[py, px] = 0.0
            labels[py, px] = label
            heapq.heappush(heap, (0.0, px, py, label))
    while heap:
        d, x, y, label = heapq.heappop(heap)
        if d > dist[y, x]:
            continue
        for dx, dy, cost in _NEIGHBORS:
            nx, ny = x + dx, y + dy
            if 0 <= nx < w and 0 <= ny < h and free_mask[ny, nx]:
                nd = d + cost
                if nd < dist[ny, nx]:
                    dist[ny, nx] = nd
                    labels[ny, nx] = label
                    heapq.heappush(heap, (nd, nx, ny, label))
    return labels, dist


def _circle_mask(shape, center_px, radius_px):
    cx, cy = center_px
    ys, xs = np.mgrid[0:shape[0], 0:shape[1]]
    return ((xs - cx) ** 2 + (ys - cy) ** 2) <= radius_px ** 2


def find_goal_point(
        region_mask, clearance_dist, centroid_px, goal_radius_m, resolution,
        min_clearance_m, max_radius_m=8.0, radius_step_m=1.0):
    radius_m = goal_radius_m
    while True:
        radius_px = radius_m / resolution
        search_mask = region_mask & _circle_mask(region_mask.shape, centroid_px, radius_px)
        region_clearance = np.where(search_mask, clearance_dist, 0.0)
        if np.any(region_clearance * resolution >= min_clearance_m):
            flat_idx = int(np.argmax(region_clearance))
            py, px = np.unravel_index(flat_idx, region_clearance.shape)
            return int(px), int(py), float(clearance_dist[py, px]) * resolution, radius_m
        if radius_m >= max_radius_m:
            region_clearance_full = np.where(region_mask, clearance_dist, 0.0)
            flat_idx = int(np.argmax(region_clearance_full))
            py, px = np.unravel_index(flat_idx, region_clearance_full.shape)
            return int(px), int(py), float(clearance_dist[py, px]) * resolution, None
        radius_m += radius_step_m


def build_regions(
        labels, clearance_dist, free_mask, resolution, origin, height,
        rooms_cfg, world_objects, inflation_radius, margin_m):
    min_clearance_m = inflation_radius + margin_m
    regions = []
    for label, room in enumerate(rooms_cfg, start=1):
        region_mask = (labels == label) & free_mask
        area_px = int(np.sum(region_mask))
        centroid_m = furniture_centroid_m(room['model_names'], world_objects)
        centroid_px_f = meters_to_pixel(centroid_m[0], centroid_m[1], resolution, origin, height)
        centroid_px = (
            int(round(np.clip(centroid_px_f[0], 0, free_mask.shape[1] - 1))),
            int(round(np.clip(centroid_px_f[1], 0, free_mask.shape[0] - 1))),
        )
        goal_radius_m = float(room.get('goal_radius_m', 2.0))

        goal_px, goal_py, goal_clearance_m, radius_used_m = find_goal_point(
            region_mask, clearance_dist, centroid_px, goal_radius_m, resolution, min_clearance_m)
        goal_x_m, goal_y_m = pixel_to_meters(goal_px, goal_py, resolution, origin, height)

        raw_name = room['name']
        name = ROOM_DISPLAY_NAMES.get(raw_name, raw_name)
        aliases = sorted(set(room.get('aliases', []) + [strip_diacritics(name).lower()]))

        regions.append({
            'id': label,
            'name': name,
            'aliases': aliases,
            'member_models': room['model_names'],
            'furniture_centroid_m': [centroid_m[0], centroid_m[1]],
            'center_px': [float(centroid_px[0]), float(centroid_px[1])],
            'goal_px': [float(goal_px), float(goal_py)],
            'goal_m': [goal_x_m, goal_y_m],
            'goal_clearance_m': goal_clearance_m,
            'min_required_clearance_m': min_clearance_m,
            'goal_radius_m_config': goal_radius_m,
            'goal_radius_m_used': radius_used_m,
            'area_px': area_px,
            'area_m2': area_px * (resolution ** 2),
        })
    return regions


def _region_colors(regions):
    rng = np.random.default_rng(42)
    return {
        region['id']: tuple(int(c) for c in rng.integers(60, 230, size=3))
        for region in regions
    }


def _render_region_canvas(occupancy, labels, regions, colors):
    free_mask = occupancy == FREE
    canvas = np.zeros((*occupancy.shape, 3), dtype=np.uint8)
    canvas[occupancy == OCCUPIED] = (0, 0, 0)
    canvas[occupancy == UNKNOWN] = (128, 128, 128)
    canvas[free_mask] = (255, 255, 255)

    overlay = canvas.copy()
    for region in regions:
        mask = labels == region['id']
        overlay[mask] = colors[region['id']]
    return cv2.addWeighted(canvas, 0.4, overlay, 0.6, 0)


def _boundary_mask(labels, free_mask):
    boundary = np.zeros(labels.shape, dtype=bool)
    diff_v = (labels[1:, :] != labels[:-1, :]) & free_mask[1:, :] & free_mask[:-1, :]
    boundary[1:, :] |= diff_v
    boundary[:-1, :] |= diff_v
    diff_h = (labels[:, 1:] != labels[:, :-1]) & free_mask[:, 1:] & free_mask[:, :-1]
    boundary[:, 1:] |= diff_h
    boundary[:, :-1] |= diff_h
    return boundary


def draw_labeled_image(
        occupancy, labels, regions, out_path, seeds=None, scale=4, raw_out_path=None):
    colors = _region_colors(regions)

    raw_canvas = _render_region_canvas(occupancy, labels, regions, colors)
    if raw_out_path:
        cv2.imwrite(raw_out_path, raw_canvas)

    h, w = occupancy.shape
    labels_up = cv2.resize(
        labels.astype(np.int32), (w * scale, h * scale), interpolation=cv2.INTER_NEAREST)
    occupancy_up = cv2.resize(
        occupancy.astype(np.uint8), (w * scale, h * scale), interpolation=cv2.INTER_NEAREST)
    free_up = occupancy_up == FREE

    canvas = _render_region_canvas(occupancy_up, labels_up, regions, colors)
    canvas[_boundary_mask(labels_up, free_up)] = (40, 40, 40)

    if seeds:
        for seed in seeds:
            cv2.drawMarker(
                canvas, (seed['px'] * scale, seed['py'] * scale), (0, 128, 0),
                cv2.MARKER_TILTED_CROSS, 6 * scale, max(1, scale // 2))

    for region in regions:
        gx, gy = int(region['goal_px'][0]) * scale, int(region['goal_px'][1]) * scale
        cv2.drawMarker(canvas, (gx, gy), (0, 0, 255), cv2.MARKER_CROSS, 10 * scale, scale)

    pil_img = Image.fromarray(cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB))
    draw = ImageDraw.Draw(pil_img, 'RGBA')
    name_font = ImageFont.truetype(_VN_FONT_PATH, 11 * scale)
    coord_font = ImageFont.truetype(_VN_FONT_PATH, 7 * scale)
    img_w, img_h = pil_img.size

    for region in regions:
        cx, cy = int(region['center_px'][0]) * scale, int(region['center_px'][1]) * scale
        name_text = region['name']
        bbox = draw.textbbox((0, 0), name_text, font=name_font)
        text_w, text_h = bbox[2] - bbox[0], bbox[3] - bbox[1]
        tx = min(max(cx - text_w / 2, 1), img_w - text_w - 1)
        ty = min(max(cy - text_h - 2 * scale, 1), img_h - text_h - 1)
        draw.text((tx, ty), name_text, font=name_font, fill=(200, 0, 0))

        gx, gy = int(region['goal_px'][0]) * scale, int(region['goal_px'][1]) * scale
        coord_text = '(%.2f, %.2f)' % (region['goal_m'][0], region['goal_m'][1])
        cbbox = draw.textbbox((0, 0), coord_text, font=coord_font)
        coord_w, coord_h = cbbox[2] - cbbox[0], cbbox[3] - cbbox[1]
        ctx = min(max(gx + 3 * scale, 1), img_w - coord_w - 1)
        cty = min(max(gy + 3 * scale, 1), img_h - coord_h - 1)
        draw.text((ctx, cty), coord_text, font=coord_font, fill=(200, 0, 0))

    legend_font = ImageFont.truetype(_VN_FONT_PATH, 8 * scale)
    pad = 3 * scale
    row_h = 9 * scale
    swatch = 7 * scale
    legend_rows = [(colors[r['id']], r['name']) for r in regions]
    legend_rows.append(((0, 0, 255), 'Diem dich (+)'))
    legend_rows.append(((0, 128, 0), 'Do dac (x)'))
    legend_w = 0
    for _, text in legend_rows:
        bbox = draw.textbbox((0, 0), text, font=legend_font)
        legend_w = max(legend_w, bbox[2] - bbox[0])
    legend_w += swatch + 3 * pad
    legend_h = pad * 2 + row_h * len(legend_rows)
    lx0, ly0 = img_w - legend_w - pad, pad
    draw.rectangle([lx0, ly0, lx0 + legend_w, ly0 + legend_h], fill=(255, 255, 255, 220),
                   outline=(0, 0, 0, 255))
    for i, (color_bgr, text) in enumerate(legend_rows):
        ry = ly0 + pad + i * row_h
        color_rgb = (color_bgr[2], color_bgr[1], color_bgr[0])
        draw.rectangle([lx0 + pad, ry, lx0 + pad + swatch, ry + swatch], fill=color_rgb,
                       outline=(0, 0, 0, 255))
        draw.text((lx0 + pad * 2 + swatch, ry - scale), text, font=legend_font, fill=(0, 0, 0))

    canvas = cv2.cvtColor(np.array(pil_img.convert('RGB')), cv2.COLOR_RGB2BGR)
    cv2.imwrite(out_path, canvas)


def segment_with_debug(
        map_yaml_path, out_dir, nav2_params_path, world_sdf_path, room_config_path,
        margin_m=0.05, write_outputs=True):
    map_data = load_map(map_yaml_path)
    occupancy = map_data['occupancy']
    resolution = map_data['resolution']
    origin = map_data['origin']
    height = map_data['height']

    free_mask = occupancy == FREE
    inflation_radius = find_nav2_inflation_radius(nav2_params_path)
    world_objects = parse_world_objects(world_sdf_path)
    rooms_cfg = load_room_furniture_config(room_config_path)

    seeds = build_seeds(rooms_cfg, world_objects, free_mask, resolution, origin, height)
    labels, _geodesic_dist = geodesic_partition(free_mask, seeds)

    clearance_dist = cv2.distanceTransform((free_mask * 255).astype(np.uint8), cv2.DIST_L2, 5)

    regions = build_regions(
        labels, clearance_dist, free_mask, resolution, origin, height,
        rooms_cfg, world_objects, inflation_radius, margin_m)

    regions_json_path = None
    image_out_path = None
    if write_outputs:
        os.makedirs(out_dir, exist_ok=True)
        regions_json_path = os.path.join(out_dir, 'regions.json')
        with open(regions_json_path, 'w', encoding='utf-8') as f:
            json.dump({
                'map_yaml': os.path.abspath(map_yaml_path),
                'resolution': resolution,
                'origin': list(origin),
                'inflation_radius_m': inflation_radius,
                'regions': regions,
            }, f, ensure_ascii=False, indent=2)

        image_out_path = os.path.join(out_dir, 'map_segmented.png')
        raw_image_out_path = os.path.join(out_dir, 'map_segmented_raw.png')
        draw_labeled_image(occupancy, labels, regions, image_out_path, seeds=seeds,
                           raw_out_path=raw_image_out_path)

    return {
        'regions': regions,
        'labels': labels,
        'clearance_dist': clearance_dist,
        'free_mask': free_mask,
        'resolution': resolution,
        'origin': origin,
        'height': height,
        'occupancy': occupancy,
        'seeds': seeds,
        'regions_json_path': regions_json_path,
        'image_out_path': image_out_path,
    }


def segment(map_yaml_path, out_dir, nav2_params_path, world_sdf_path, room_config_path,
            margin_m=0.05):
    result = segment_with_debug(
        map_yaml_path, out_dir, nav2_params_path, world_sdf_path, room_config_path,
        margin_m=margin_m, write_outputs=True)
    return (result['regions'], result['labels'],
            result['regions_json_path'], result['image_out_path'])


def top_clearance_candidates(
        region_mask, clearance_dist, centroid_px, goal_radius_m,
        resolution, radii_m=(1.0, 2.0, 3.0, 4.0, 6.0, 8.0)):
    candidates = []
    seen = set()
    for radius_m in sorted(set([goal_radius_m] + list(radii_m))):
        radius_px = radius_m / resolution
        mask = region_mask & _circle_mask(region_mask.shape, centroid_px, radius_px)
        region_clearance = np.where(mask, clearance_dist, 0.0)
        if not np.any(region_clearance > 0):
            continue
        flat_idx = int(np.argmax(region_clearance))
        py, px = np.unravel_index(flat_idx, region_clearance.shape)
        key = (int(px), int(py))
        if key in seen:
            continue
        seen.add(key)
        candidates.append({
            'px': float(px), 'py': float(py),
            'clearance_m': float(clearance_dist[py, px]) * resolution,
            'radius_m': radius_m,
        })
    return candidates


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
    parser.add_argument('--margin-m', type=float, default=0.05)
    return parser


def main():
    parser = build_arg_parser()
    args = parser.parse_args()
    regions, _, regions_json_path, image_out_path = segment(
        args.map_yaml, args.out_dir, args.nav2_params, args.world_sdf, args.room_config,
        margin_m=args.margin_m)
    print('regions:', len(regions))
    for region in regions:
        print(region['id'], region['name'], 'area_m2=%.2f' % region['area_m2'],
              'goal_m=%s' % region['goal_m'], 'goal_clearance_m=%.3f' % region['goal_clearance_m'])
    print('wrote', regions_json_path)
    print('wrote', image_out_path)


if __name__ == '__main__':
    main()
