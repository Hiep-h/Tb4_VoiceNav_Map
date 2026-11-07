import argparse
import csv
import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Logger(Node):
    def __init__(self, path, pose_period):
        super().__init__('run_logger')
        self.path = path
        self.t0 = time.time()
        self.sim = 0.0
        self.file = open(path, 'w', newline='', encoding='utf-8-sig')
        self.w = csv.writer(self.file)
        self.w.writerow(['wall_s', 'sim_s', 'event', 'text', 'x', 'y', 'yaw_deg'])
        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)
        self.create_subscription(Clock, '/clock', self.on_clock, 10)
        self.create_subscription(String, '/voice_text', self.on_voice, 10)
        self.create_subscription(String, '/voice_nav/status', self.on_status, 10)
        self.create_timer(pose_period, self.on_pose)

    def row(self, event, text='', x='', y='', yaw=''):
        self.w.writerow(['%.2f' % (time.time() - self.t0), '%.2f' % self.sim, event, text, x, y, yaw])
        self.file.flush()

    def on_clock(self, msg):
        self.sim = msg.clock.sec + msg.clock.nanosec * 1e-9

    def on_voice(self, msg):
        self.row('voice', msg.data)

    def on_status(self, msg):
        self.row('status', msg.data)

    def on_pose(self):
        try:
            t = self.buf.lookup_transform('map', 'base_link', Time())
        except Exception:
            return
        p = t.transform.translation
        self.row('pose', '', '%.3f' % p.x, '%.3f' % p.y, '%.1f' % math.degrees(yaw_of(t.transform.rotation)))


def summarize(path):
    rows = list(csv.DictReader(open(path, encoding='utf-8-sig')))
    if not rows:
        print('log trong')
        return
    poses = [r for r in rows if r['event'] == 'pose']
    events = [r for r in rows if r['event'] in ('voice', 'status')]
    print('tong so dong: %d, so lan lay vi tri: %d' % (len(rows), len(poses)))
    print('-' * 78)
    print('%-8s %-8s %-7s %s' % ('wall_s', 'sim_s', 'loai', 'noi dung'))
    for r in events:
        print('%-8s %-8s %-7s %s' % (r['wall_s'], r['sim_s'], r['event'], r['text'][:60]))
    if poses:
        def nearest(sim):
            return min(poses, key=lambda p: abs(float(p['sim_s']) - sim))
        print('-' * 78)
        print('vi tri robot (map) tai moi su kien trang thai:')
        for r in events:
            if r['event'] != 'status':
                continue
            p = nearest(float(r['sim_s']))
            print('  sim %-8s x=%-7s y=%-7s | %s' % (r['sim_s'], p['x'], p['y'], r['text'][:50]))
    if len(poses) >= 2:
        d = 0.0
        for a, b in zip(poses, poses[1:]):
            d += math.hypot(float(b['x']) - float(a['x']), float(b['y']) - float(a['y']))
        print('-' * 78)
        print('quang duong robot da di (AMCL): %.2f m' % d)
        print('thoi gian: %s s mo phong, %s s thuc' % (rows[-1]['sim_s'], rows[-1]['wall_s']))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default=os.path.expanduser('~/tb4_voicenav_data/run_log.csv'))
    ap.add_argument('--pose-period', type=float, default=1.0)
    ap.add_argument('--summary', default='')
    args = ap.parse_args()
    if args.summary:
        summarize(args.summary)
        return
    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    rclpy.init()
    node = Logger(args.out, args.pose_period)
    print('dang ghi log vao %s (Ctrl+C de dung)' % args.out)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.file.close()
    print('xong. Tom tat: python3 run_logger.py --summary %s' % args.out)
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0)


if __name__ == '__main__':
    main()
