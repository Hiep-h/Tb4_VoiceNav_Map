import argparse
import json
import math
import os
import threading

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

FONT_PATHS = [
    '/usr/share/fonts/truetype/dejavu/DejaVuSans{b}.ttf',
    '/usr/share/fonts/TTF/DejaVuSans{b}.ttf',
]
FONTS = {}
ROOM_COLORS = [(190, 40, 40), (30, 100, 200), (30, 140, 60), (160, 60, 160)]


def font(size, bold=False):
    key = (size, bold)
    if key not in FONTS:
        FONTS[key] = ImageFont.load_default()
        for path in FONT_PATHS:
            p = path.format(b='-Bold' if bold else '')
            if os.path.exists(p):
                FONTS[key] = ImageFont.truetype(p, size)
                break
    return FONTS[key]


def read_map_yaml(path):
    res, origin = 0.05, (0.0, 0.0)
    for line in open(path, encoding='utf-8'):
        s = line.strip()
        if s.startswith('resolution:'):
            res = float(s.split(':', 1)[1])
        elif s.startswith('origin:'):
            vals = s.split(':', 1)[1].strip().strip('[]').split(',')
            origin = (float(vals[0]), float(vals[1]))
    return res, origin


class View:
    def __init__(self, image_path, res, origin, regions, scale):
        base = cv2.imread(image_path)
        self.h, self.w = base.shape[:2]
        self.scale = scale
        self.res = res
        self.origin = origin
        self.regions = regions
        self.base = cv2.resize(
            base, (self.w * scale, self.h * scale), interpolation=cv2.INTER_NEAREST)
        self.bar_h = 96
        self.robot = None
        self.trail = []
        self.route = []
        self.plan = []
        self.voice = ''
        self.status = ''

    def to_px(self, x, y):
        px = (x - self.origin[0]) / self.res * self.scale
        py = (self.h - (y - self.origin[1]) / self.res) * self.scale
        return int(round(px)), int(round(py))

    def render(self, show_bar=True):
        img = self.base.copy()
        if len(self.plan) > 1:
            pts = np.array([self.to_px(x, y) for x, y in self.plan], np.int32)
            cv2.polylines(img, [pts], False, (200, 120, 0), 2, cv2.LINE_AA)
        if len(self.route) > 1:
            pts = np.array([self.to_px(x, y) for x, y in self.route], np.int32)
            cv2.polylines(img, [pts], False, (0, 200, 255), 4, cv2.LINE_AA)
        if len(self.trail) > 1:
            pts = np.array([self.to_px(x, y) for x, y, _ in self.trail], np.int32)
            cv2.polylines(img, [pts], False, (120, 120, 120), 2, cv2.LINE_AA)
        for i, r in enumerate(self.regions):
            gx, gy = self.to_px(*r['goal_m'])
            cv2.drawMarker(img, (gx, gy), (0, 0, 220), cv2.MARKER_CROSS, 22, 3)
        if self.robot is not None:
            rx, ry = self.to_px(self.robot[0], self.robot[1])
            cv2.circle(img, (rx, ry), 12, (255, 80, 0), -1)
            cv2.circle(img, (rx, ry), 12, (255, 255, 255), 2)
            hx = int(rx + 22 * math.cos(self.robot[2]))
            hy = int(ry - 22 * math.sin(self.robot[2]))
            cv2.arrowedLine(img, (rx, ry), (hx, hy), (255, 255, 255), 3, tipLength=0.5)
        pil = Image.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
        d = ImageDraw.Draw(pil)
        f = font(22, True)
        for r in self.regions:
            gx, gy = self.to_px(*r['goal_m'])
            text = r['name']
            box = d.textbbox((0, 0), text, font=f)
            tw = box[2] - box[0]
            tx = min(max(gx - tw / 2, 4), pil.size[0] - tw - 4)
            ty = max(gy - 40, 4)
            d.rectangle([tx - 4, ty - 2, tx + tw + 4, ty + 28], fill=(255, 255, 255))
            d.text((tx, ty), text, font=f, fill=(150, 0, 0))
        if not show_bar:
            return cv2.cvtColor(np.array(pil), cv2.COLOR_RGB2BGR)
        bar = Image.new('RGB', (pil.size[0], self.bar_h), (31, 47, 153))
        bd = ImageDraw.Draw(bar)
        bd.text((14, 8), 'Bạn nói: ' + (self.voice or '-')[:80], font=font(24, True),
                fill=(255, 255, 255))
        bd.text((14, 46), 'Robot: ' + (self.status or '-')[:100], font=font(22),
                fill=(255, 235, 150))
        out = Image.new('RGB', (pil.size[0], pil.size[1] + self.bar_h))
        out.paste(pil, (0, 0))
        out.paste(bar, (0, pil.size[1]))
        return cv2.cvtColor(np.array(out), cv2.COLOR_RGB2BGR)


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def path_points(msg):
    return [(p.pose.position.x, p.pose.position.y) for p in msg.poses]


def run_ros(view, args, node=None):
    import rclpy
    from nav_msgs.msg import Path
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformListener

    owns_node = node is None
    if owns_node:
        rclpy.init()
        node = rclpy.create_node('map_view')
    buf = Buffer()
    TransformListener(buf, node)
    subs = {}

    def on_voice(m):
        view.voice = m.data

    def on_status(m):
        view.status = m.data

    def on_plan(m):
        view.plan = path_points(m)

    def on_route(m):
        view.route = path_points(m)

    node.create_subscription(String, '/voice_text', on_voice, 10)
    node.create_subscription(String, '/voice_nav/status', on_status, 10)
    node.create_subscription(Path, '/plan', on_plan, 10)

    def discover():
        for name, types in node.get_topic_names_and_types():
            if 'nav_msgs/msg/Path' in types and name not in subs and name != '/plan':
                low = name.lower()
                matched = args.route_topic and name == args.route_topic
                matched = matched or (
                    not args.route_topic and ('voice' in low or 'route' in low))
                if matched:
                    subs[name] = node.create_subscription(Path, name, on_route, 10)
                    print('route topic:', name)

    def on_tick():
        try:
            t = buf.lookup_transform('map', 'base_link', rclpy.time.Time())
        except Exception:
            return
        p = t.transform.translation
        view.robot = (p.x, p.y, yaw_of(t.transform.rotation))
        if not view.trail or math.hypot(p.x - view.trail[-1][0], p.y - view.trail[-1][1]) > 0.1:
            view.trail.append(view.robot)
            del view.trail[:-400]

    node.create_timer(0.2, on_tick)
    node.create_timer(2.0, discover)
    if owns_node:
        threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    return rclpy, node, owns_node


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    maps = os.path.join(os.path.dirname(here), 'maps')
    ap = argparse.ArgumentParser()
    ap.add_argument('--image', default=os.path.join(maps, 'map_segmented_raw.png'))
    ap.add_argument('--map-yaml', default=os.path.join(maps, 'map.yaml'))
    ap.add_argument('--regions', default=os.path.join(maps, 'regions.json'))
    ap.add_argument('--scale', type=int, default=3)
    ap.add_argument('--route-topic', default='')
    ap.add_argument('--no-ros', action='store_true')
    args = ap.parse_args()
    res, origin = read_map_yaml(args.map_yaml)
    data = json.load(open(args.regions, encoding='utf-8'))
    regions = data['regions'] if isinstance(data, dict) and 'regions' in data else data
    view = View(args.image, res, origin, regions, args.scale)
    rclpy = None
    node = None
    owns_node = False
    if not args.no_ros:
        rclpy, node, owns_node = run_ros(view, args)
    cv2.namedWindow('Ban do', cv2.WINDOW_AUTOSIZE)
    while True:
        cv2.imshow('Ban do', view.render())
        k = cv2.waitKey(100) & 0xFF
        if k in (27, ord('q')):
            break
        if k == ord('c'):
            view.trail.clear()
    cv2.destroyAllWindows()
    if node is not None and owns_node:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
