import argparse
import audioop
import collections
import math
import os
import sys
import threading
import time

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from map_view import View, read_map_yaml, run_ros as run_map_ros

VW = 480
CONTENT_H = 540
BG = (245, 247, 250)
TITLE_BG = (31, 47, 153)
BTN_CX, BTN_CY, BTN_R = VW // 2, 210, 80
FONT_PATHS = [
    '/usr/share/fonts/truetype/dejavu/DejaVuSans{b}.ttf',
    '/usr/share/fonts/TTF/DejaVuSans{b}.ttf',
]
DEFAULT_VOSK = os.path.expanduser('~/tb4_voicenav_data/models/vosk-model-small-vn-0.4')
GRAMMAR_WORDS = [
    'đi', 'đến', 'tới', 'từ', 'qua', 'rồi', 'di', 'chuyển', 'khu', 'vực', 'phòng',
    'khách', 'ngủ', 'bếp', 'a', 'b', 'c', 'hành', 'lang', 'thí', 'nghiệm', '[unk]',
]
GOOGLE_LANG = 'vi-VN'
DEFAULT_REGIONS = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', 'maps', 'regions.json')
WHISPER_MODELS_DIR = os.path.expanduser('~/tb4_voicenav_data/models')
HERE = os.path.dirname(os.path.abspath(__file__))
MAPS_DIR = os.path.join(os.path.dirname(HERE), 'maps')
MAX_PHRASE_S = 8.0
SILENCE_S = 0.8


def load_font(size, bold=False):
    for path in FONT_PATHS:
        p = path.format(b='-Bold' if bold else '')
        if os.path.exists(p):
            return ImageFont.truetype(p, size)
    return ImageFont.load_default()


FONTS = {}


def font(size, bold=False):
    key = (size, bold)
    if key not in FONTS:
        FONTS[key] = load_font(size, bold)
    return FONTS[key]


def draw_center(draw, text, y, size, fill, bold=False):
    f = font(size, bold)
    box = draw.textbbox((0, 0), text, font=f)
    draw.text(((VW - (box[2] - box[0])) / 2, y), text, font=f, fill=fill)


def wrap(draw, text, size, max_w):
    f = font(size, True)
    words = text.split()
    lines, cur = [], ''
    for w in words:
        trial = (cur + ' ' + w).strip()
        box = draw.textbbox((0, 0), trial, font=f)
        if box[2] - box[0] > max_w and cur:
            lines.append(cur)
            cur = w
        else:
            cur = trial
    if cur:
        lines.append(cur)
    return lines


def render_voice(state, height):
    img = Image.new('RGB', (VW, max(height, CONTENT_H)), BG)
    d = ImageDraw.Draw(img)
    d.rectangle([0, 0, VW, 64], fill=TITLE_BG)
    for i, line in enumerate(wrap(d, 'Điều khiển robot bằng giọng nói', 22, VW - 24)[:2]):
        d.text((12, 8 + i * 26), line, font=font(22, True), fill=(255, 255, 255))
    phase = state['phase']
    if phase == 'calibrate':
        draw_center(d, 'Giữ im lặng', 150, 40, (60, 60, 60), True)
        draw_center(d, 'Đang đo tiếng ồn nền', 210, 24, (110, 110, 110))
        return cv2.cvtColor(np.array(img), cv2.COLOR_RGB2BGR)
    if phase == 'listening':
        pulse = 0.5 + 0.5 * math.sin(state['t'] * 6)
        r = int(BTN_R * (0.85 + 0.15 * pulse))
        d.ellipse([BTN_CX - r, BTN_CY - r, BTN_CX + r, BTN_CY + r], fill=(220, 50, 50))
        draw_center(d, 'Đang nghe...', BTN_CY + BTN_R + 20, 26, (190, 40, 40), True)
        if state['debug']:
            draw_center(d, 'RMS: %d / nguong %.0f' % (state['rms'], state['threshold']),
                        BTN_CY + BTN_R + 50, 18, (120, 120, 120))
    elif phase == 'recognizing':
        d.ellipse([BTN_CX - BTN_R, BTN_CY - BTN_R, BTN_CX + BTN_R, BTN_CY + BTN_R],
                  fill=(180, 110, 0))
        draw_center(d, 'Đang nhận dạng...', BTN_CY + BTN_R + 20, 24, (180, 110, 0), True)
    else:
        d.ellipse([BTN_CX - BTN_R, BTN_CY - BTN_R, BTN_CX + BTN_R, BTN_CY + BTN_R],
                  fill=(140, 140, 140))
        draw_center(d, 'Nhấn để nói', BTN_CY + BTN_R + 20, 24, (90, 90, 90), True)
    y = BTN_CY + BTN_R + 60
    if state['error']:
        for line in wrap(d, 'Lỗi: ' + state['error'], 22, VW - 40)[:3]:
            d.text((20, y), line, font=font(22, True), fill=(190, 60, 60))
            y += 28
    elif state['text']:
        d.text((20, y), 'Bạn đã nói:', font=font(22), fill=(110, 110, 110))
        y += 26
        for line in wrap(d, '"%s"' % state['text'], 24, VW - 40)[:3]:
            d.text((20, y), line, font=font(24, True), fill=(31, 47, 153))
            y += 30
        if state['parsed']:
            for line in wrap(d, 'Lệnh hiểu: ' + state['parsed'], 20, VW - 40)[:2]:
                d.text((20, y), line, font=font(20, True), fill=(27, 120, 60))
                y += 24
        if state['debug'] and state['latency']:
            d.text((20, y), 'Độ trễ: %.2f s' % state['latency'], font=font(18),
                   fill=(90, 90, 90))
            y += 22
    if state['debug'] and state['mic_info']:
        d.text((20, y), state['mic_info'], font=font(16), fill=(130, 130, 130))
        y += 20
    y = max(y, height - 90)
    route_text = 'Lộ trình: ' + (' -> '.join(state['route']) if state['route'] else '-')
    for line in wrap(d, route_text, 20, VW - 40)[:2]:
        d.text((20, y), line, font=font(20, True), fill=(31, 47, 153))
        y += 24
    status_text = 'Robot: ' + (state['status'] or '-')
    for line in wrap(d, status_text, 20, VW - 40)[:2]:
        d.text((20, y), line, font=font(20), fill=(27, 120, 60))
        y += 24
    return cv2.cvtColor(np.array(img), cv2.COLOR_RGB2BGR)


def compose(voice_img, map_img):
    if map_img is None:
        return voice_img
    h = max(voice_img.shape[0], map_img.shape[0])

    def pad(im):
        if im.shape[0] == h:
            return im
        out = np.full((h, im.shape[1], 3), BG[::-1], np.uint8)
        out[:im.shape[0]] = im
        return out
    return np.hstack([pad(voice_img), pad(map_img)])


def build_whisper_prompt(regions_path):
    import json
    try:
        data = json.load(open(regions_path or DEFAULT_REGIONS, encoding='utf-8'))
        regions = data['regions'] if isinstance(data, dict) and 'regions' in data else data
        names = [r['name'] for r in regions]
    except Exception:
        names = ['phòng khách', 'phòng ngủ', 'phòng bếp']
    sentences = ['Đi đến %s.' % n for n in names]
    if len(names) >= 2:
        sentences.append('Từ %s đến %s.' % (names[0], names[-1]))
    return ' '.join(sentences)


class Recognizer:
    def __init__(self, backend, device, vosk_path, grammar, whisper_model='base',
                 energy=None, debug=False, regions_path=None):
        import speech_recognition as sr
        self.sr = sr
        if backend == 'auto':
            backend = 'vosk' if os.path.isdir(vosk_path) else 'google'
        self.backend = backend
        self.rec = sr.Recognizer()
        self.rec.pause_threshold = SILENCE_S
        self.mic = sr.Microphone(device_index=device, sample_rate=16000)
        self.mic_name, self.mic_index = self._describe_mic(sr, device)
        self.energy_override = energy
        self.median_rms = 0.0
        self.debug = debug
        self.vosk_model = None
        self.whisper = None
        self.whisper_model_name = whisper_model
        self.grammar = grammar
        if debug:
            print('[debug] backend: %s' % self.backend)
        if backend == 'vosk':
            import vosk
            vosk.SetLogLevel(-1)
            self.vosk = vosk
            self.vosk_model = vosk.Model(vosk_path)
        elif backend == 'whisper':
            from faster_whisper import WhisperModel
            os.makedirs(WHISPER_MODELS_DIR, exist_ok=True)
            if debug:
                print('[debug] whisper model: %s' % whisper_model)
            self.whisper = WhisperModel(
                whisper_model, device='cpu', compute_type='int8',
                download_root=WHISPER_MODELS_DIR)
            self.whisper_prompt = build_whisper_prompt(regions_path)

    @staticmethod
    def _describe_mic(sr, device):
        try:
            names = sr.Microphone.list_microphone_names()
        except Exception:
            names = []
        if device is None:
            try:
                import pyaudio
                pa = pyaudio.PyAudio()
                info = pa.get_default_input_device_info()
                pa.terminate()
                return info.get('name', '?'), info.get('index', -1)
            except Exception:
                return (names[0] if names else '?'), 0
        name = names[device] if 0 <= device < len(names) else str(device)
        return name, device

    def calibrate(self, seconds=1.5):
        if self.debug:
            print('[debug] mic: %s (index %s)' % (self.mic_name, self.mic_index))
        with self.mic as src:
            seconds_per_buffer = float(src.CHUNK) / src.SAMPLE_RATE
            n_buffers = max(1, int(math.ceil(seconds / seconds_per_buffer)))
            samples = []
            for _ in range(n_buffers):
                buf = src.stream.pyaudio_stream.read(src.CHUNK, exception_on_overflow=False)
                samples.append(audioop.rms(buf, src.SAMPLE_WIDTH))
        samples.sort()
        mid = len(samples) // 2
        median = samples[mid] if len(samples) % 2 else (samples[mid - 1] + samples[mid]) / 2
        self.median_rms = median
        if self.energy_override is not None:
            self.rec.energy_threshold = self.energy_override
        else:
            self.rec.energy_threshold = min(1000, max(300, 2.5 * median))
        self.rec.dynamic_energy_threshold = False
        self.rec.pause_threshold = SILENCE_S
        if self.debug:
            print('[debug] median_rms=%.1f, energy_threshold=%.1f' % (
                median, self.rec.energy_threshold))

    def recognize(self, audio):
        if self.backend == 'google':
            res = self.rec.recognize_google(audio, language=GOOGLE_LANG, show_all=True)
            alts = res.get('alternative', []) if isinstance(res, dict) else []
            texts = [a.get('transcript', '') for a in alts]
            return (texts[0] if texts else ''), texts[1:]
        if self.backend == 'whisper':
            raw = audio.get_raw_data(convert_rate=16000, convert_width=2)
            samples = np.frombuffer(raw, dtype=np.int16).astype(np.float32) / 32768.0
            segments, _ = self.whisper.transcribe(
                samples, language='vi', beam_size=5, temperature=0.0,
                condition_on_previous_text=False, initial_prompt=self.whisper_prompt,
                hotwords=self.whisper_prompt)
            text = ' '.join(seg.text.strip() for seg in segments).strip()
            return text, []
        raw = audio.get_raw_data(convert_rate=16000, convert_width=2)
        if self.grammar:
            import json
            grammar_json = json.dumps(GRAMMAR_WORDS, ensure_ascii=False)
            rec = self.vosk.KaldiRecognizer(self.vosk_model, 16000, grammar_json)
        else:
            rec = self.vosk.KaldiRecognizer(self.vosk_model, 16000)
        rec.AcceptWaveform(raw)
        import json
        text = json.loads(rec.FinalResult()).get('text', '').replace('[unk]', '').strip()
        return text, []


def listen_for_phrase(mic, rec, state, stop_event, max_phrase_s, done_cb):
    debug = state['debug']
    last_print = [0.0]

    def tick(energy):
        state['rms'] = energy
        state['threshold'] = rec.energy_threshold
        now = time.time()
        if debug and now - last_print[0] > 0.3:
            print('[debug] rms=%d nguong=%.1f' % (energy, rec.energy_threshold))
            last_print[0] = now

    def adjust(energy, seconds_per_buffer):
        if rec.dynamic_energy_threshold:
            damping = rec.dynamic_energy_adjustment_damping ** seconds_per_buffer
            target = energy * rec.dynamic_energy_ratio
            rec.energy_threshold = rec.energy_threshold * damping + target * (1 - damping)

    try:
        with mic as source:
            seconds_per_buffer = float(source.CHUNK) / source.SAMPLE_RATE
            pause_buffer_count = int(math.ceil(rec.pause_threshold / seconds_per_buffer))
            phrase_buffer_count = int(math.ceil(rec.phrase_threshold / seconds_per_buffer))
            non_speaking_buffer_count = int(
                math.ceil(rec.non_speaking_duration / seconds_per_buffer))
            elapsed = 0.0
            frames = collections.deque()
            heard_speech = False
            while not stop_event.is_set():
                elapsed += seconds_per_buffer
                if elapsed > max_phrase_s:
                    done_cb(None, timeout=True)
                    return
                buffer = source.stream.pyaudio_stream.read(
                    source.CHUNK, exception_on_overflow=False)
                if not buffer:
                    done_cb(None, timeout=True)
                    return
                frames.append(buffer)
                if len(frames) > non_speaking_buffer_count:
                    frames.popleft()
                energy = audioop.rms(buffer, source.SAMPLE_WIDTH)
                tick(energy)
                if energy > rec.energy_threshold:
                    heard_speech = True
                    break
                adjust(energy, seconds_per_buffer)
            if stop_event.is_set():
                done_cb(None, timeout=False)
                return
            if not heard_speech:
                done_cb(None, timeout=True)
                return
            pause_count, phrase_count = 0, 0
            phrase_start = elapsed
            while not stop_event.is_set():
                elapsed += seconds_per_buffer
                if elapsed - phrase_start > max_phrase_s:
                    break
                buffer = source.stream.pyaudio_stream.read(
                    source.CHUNK, exception_on_overflow=False)
                if not buffer:
                    break
                frames.append(buffer)
                phrase_count += 1
                energy = audioop.rms(buffer, source.SAMPLE_WIDTH)
                tick(energy)
                if energy > rec.energy_threshold:
                    pause_count = 0
                else:
                    pause_count += 1
                if pause_count > pause_buffer_count:
                    break
                adjust(energy, seconds_per_buffer)
            if stop_event.is_set():
                done_cb(None, timeout=False)
                return
            phrase_count -= pause_count
            if phrase_count < phrase_buffer_count:
                done_cb(None, timeout=True)
                return
            for _ in range(pause_count - non_speaking_buffer_count):
                if frames:
                    frames.pop()
            frame_data = b''.join(frames)
            import speech_recognition as sr
            audio = sr.AudioData(frame_data, source.SAMPLE_RATE, source.SAMPLE_WIDTH)
            done_cb(audio, timeout=False)
    except Exception as e:
        state['error'] = '%s: %s' % (type(e).__name__, e)
        done_cb(None, timeout=False)


def load_parser(regions_path):
    try:
        from tb4_voicenav.command_parser import parse_command
        from tb4_voicenav.region_store import RegionStore
    except Exception:
        return None
    region_store = RegionStore().load(regions_path)

    def run(text):
        result = parse_command(text, region_store)
        if result.error:
            return 'LOI (%s): %s' % (result.error, result.error_detail or '')
        return 'di qua: ' + ' -> '.join(r.name for r in result.targets)
    return run


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--stt', default='google', choices=['google', 'vosk', 'whisper', 'auto'])
    ap.add_argument('--mic', type=int, default=None)
    ap.add_argument('--vosk-model', default=DEFAULT_VOSK)
    ap.add_argument('--whisper-model', default='small')
    ap.add_argument('--grammar', action='store_true')
    ap.add_argument('--parse', action='store_true')
    ap.add_argument('--regions', default=DEFAULT_REGIONS)
    ap.add_argument('--debug', action='store_true')
    ap.add_argument('--energy', type=float, default=None, help='Dat tay energy_threshold')
    ap.add_argument('--ros', action='store_true')
    ap.add_argument('--hold-to-talk', action='store_true')
    ap.add_argument('--no-map', action='store_true')
    ap.add_argument('--map-scale', type=int, default=2)
    ap.add_argument('--map-image', default=os.path.join(MAPS_DIR, 'map_segmented_raw.png'))
    ap.add_argument('--map-yaml', default=os.path.join(MAPS_DIR, 'map.yaml'))
    ap.add_argument('--route-topic', default='')
    args = ap.parse_args()

    if args.hold_to_talk:
        print('Luu y: --hold-to-talk khong bat duoc su kien tha phim dang tin cay qua OpenCV; '
              'mic se tu dung khi im lang hoac du 8s, khong phu thuoc luc tha phim.')

    state = {'phase': 'calibrate', 'text': '', 'error': '', 'latency': 0.0, 'alts': [],
             'parsed': '', 'status': '', 'route': [], 'current': '', 't': 0.0,
             'debug': args.debug, 'rms': 0, 'threshold': 0.0, 'mic_info': ''}
    parser = load_parser(args.regions) if args.parse else None
    holder = {}

    ros_node = None
    ros_pub = None
    rclpy_mod = None
    if args.ros:
        import rclpy
        from std_msgs.msg import String
        rclpy.init()
        ros_node = rclpy.create_node('voice_gui')
        ros_pub = ros_node.create_publisher(String, '/voice_text', 10)
        rclpy_mod = rclpy

        def on_status(msg):
            text = msg.data
            if text.startswith('ROUTE:'):
                state['route'] = text[len('ROUTE:'):].split('|')
                state['current'] = state['route'][0] if state['route'] else ''
            elif text.startswith('GOTO:'):
                state['current'] = text[len('GOTO:'):]
            elif text == 'DONE':
                state['current'] = ''
            elif not text.startswith('ARRIVED:'):
                state['status'] = text

        ros_node.create_subscription(String, '/voice_nav/status', on_status, 10)

    view = None
    if not args.no_map:
        res, origin = read_map_yaml(args.map_yaml)
        import json
        data = json.load(open(args.regions, encoding='utf-8'))
        regions = data['regions'] if isinstance(data, dict) and 'regions' in data else data
        view = View(args.map_image, res, origin, regions, args.map_scale)
        if args.ros:
            run_map_ros(view, args, node=ros_node)

    if args.ros:
        threading.Thread(target=rclpy_mod.spin, args=(ros_node,), daemon=True).start()

    finish_lock = threading.Lock()
    listen = {'active': False, 'stop_event': None}

    def finish(audio, cancelled=False, timeout=False):
        with finish_lock:
            if not listen['active']:
                return
            listen['active'] = False
            stop_event = listen['stop_event']
            listen['stop_event'] = None
        if stop_event:
            stop_event.set()
        if cancelled or audio is None:
            state['phase'] = 'off'
            if timeout:
                state['text'] = ''
                state['error'] = 'Không nghe thấy tiếng nói, kiểm tra mic'
            return
        state['error'] = ''
        state['phase'] = 'recognizing'

        def recognize_worker():
            t0 = time.time()
            try:
                text, alts = holder['rec'].recognize(audio)
            except Exception as e:
                state['error'] = '%s: %s' % (type(e).__name__, e)
                state['phase'] = 'off'
                return
            state['latency'] = time.time() - t0
            state['text'] = text
            state['error'] = ''
            state['alts'] = alts
            state['parsed'] = parser(text) if (parser and text) else ''
            if ros_pub is not None and text:
                from std_msgs.msg import String
                print('[voice_gui] publish /voice_text (1 lan): %r' % text)
                ros_pub.publish(String(data=text))
            state['phase'] = 'off'
        threading.Thread(target=recognize_worker, daemon=True).start()

    def start_listening():
        if state['phase'] != 'off' or 'rec' not in holder:
            return
        rec = holder['rec'].rec
        stop_event = threading.Event()
        listen['stop_event'] = stop_event
        listen['active'] = True
        state['error'] = ''
        state['rms'] = 0
        state['threshold'] = rec.energy_threshold
        state['phase'] = 'listening'

        def on_done(audio, timeout):
            finish(audio, cancelled=(audio is None), timeout=timeout)

        threading.Thread(
            target=listen_for_phrase,
            args=(holder['rec'].mic, rec, state, stop_event, MAX_PHRASE_S, on_done),
            daemon=True).start()

    def toggle_mic():
        if state['phase'] == 'off':
            start_listening()
        elif state['phase'] == 'listening':
            finish(None, cancelled=True)

    def calib_worker():
        try:
            holder['rec'] = Recognizer(
                args.stt, args.mic, args.vosk_model, args.grammar, args.whisper_model,
                energy=args.energy, debug=args.debug, regions_path=args.regions)
            holder['rec'].calibrate(1.5)
            state['mic_info'] = 'Mic: %s (idx %s) | median=%.0f nguong=%.0f | backend=%s' % (
                holder['rec'].mic_name, holder['rec'].mic_index, holder['rec'].median_rms,
                holder['rec'].rec.energy_threshold, holder['rec'].backend)
            state['threshold'] = holder['rec'].rec.energy_threshold
        except Exception as e:
            state['error'] = '%s: %s' % (type(e).__name__, e)
        state['phase'] = 'off'
    threading.Thread(target=calib_worker, daemon=True).start()

    def on_mouse(event, x, y, flags, _userdata):
        if event == cv2.EVENT_LBUTTONDOWN and x < VW:
            if math.hypot(x - BTN_CX, y - BTN_CY) <= BTN_R:
                toggle_mic()

    win = 'Dieu khien robot bang giong noi'
    cv2.namedWindow(win, cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback(win, on_mouse)
    t0 = time.time()
    while True:
        state['t'] = time.time() - t0
        map_img = view.render(show_bar=False) if view is not None else None
        voice_img = render_voice(state, map_img.shape[0] if map_img is not None else 0)
        cv2.imshow(win, compose(voice_img, map_img))
        k = cv2.waitKey(30) & 0xFF
        if k == 27:
            break
        if k == 32:
            if args.hold_to_talk:
                if state['phase'] == 'off':
                    start_listening()
            else:
                toggle_mic()
    finish(None, cancelled=True)
    cv2.destroyAllWindows()
    if ros_node is not None:
        ros_node.destroy_node()
        rclpy_mod.shutdown()
    sys.exit(0)


if __name__ == '__main__':
    main()
