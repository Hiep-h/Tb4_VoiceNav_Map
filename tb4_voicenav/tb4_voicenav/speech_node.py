import json
import os
import socket
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from tb4_voicenav.command_parser import build_command_phrases, normalize_text
from tb4_voicenav.region_store import RegionStore

DEFAULT_REGIONS_JSON = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', 'maps', 'regions.json')
DEFAULT_VOSK_MODEL_PATH = os.path.expanduser('~/tb4_voicenav_data/models/vosk-model-small-vn-0.4')
MODELS_DIR = os.path.expanduser('~/tb4_voicenav_data/models')
GOOGLE_LANG = {'vi': 'vi-VN', 'en': 'en-US'}
WHISPER_LANG = {'vi': 'vi', 'en': 'en'}


def google_reachable(timeout=1.5):
    try:
        socket.create_connection(('www.google.com', 443), timeout=timeout)
        return True
    except OSError:
        return False


def match_score(text, grammar):
    import difflib
    norm_text = normalize_text(text)
    best = 0.0
    for phrase in grammar:
        score = difflib.SequenceMatcher(None, norm_text, normalize_text(phrase)).ratio()
        if score > best:
            best = score
    return best


class SpeechNode(Node):
    def __init__(self):
        super().__init__('speech_node')
        self.declare_parameter('stt', 'auto')
        self.declare_parameter('device', -1)
        self.declare_parameter('lang', 'vi')
        self.declare_parameter('regions_json_path', DEFAULT_REGIONS_JSON)
        self.declare_parameter('vosk_model_path', DEFAULT_VOSK_MODEL_PATH)
        self.declare_parameter('noise_calib_sec', 1.5)

        self.backend_requested = self.get_parameter('stt').value
        device = self.get_parameter('device').value
        self.device = None if device < 0 else device
        self.lang = self.get_parameter('lang').value
        self.vosk_model_path = os.path.expanduser(self.get_parameter('vosk_model_path').value)
        self.noise_calib_sec = self.get_parameter('noise_calib_sec').value

        regions_json_path = self.get_parameter('regions_json_path').value or DEFAULT_REGIONS_JSON
        self.region_store = RegionStore().load(regions_json_path)
        self.grammar = build_command_phrases(self.region_store)

        self.publisher = self.create_publisher(String, '/voice_text', 10)

        self.active_backend = self._resolve_backend(self.backend_requested)
        self.get_logger().info(
            f"speech_node: backend='{self.active_backend}' (yeu cau='{self.backend_requested}')")
        threading.Thread(target=self._run, daemon=True).start()

    def _resolve_backend(self, requested):
        if requested != 'auto':
            return requested
        if google_reachable():
            return 'google'
        self.get_logger().warn("auto: khong ket noi duoc Google, chuyen sang whisper")
        return 'whisper'

    def publish_text(self, text):
        msg = String()
        msg.data = text
        self.publisher.publish(msg)
        self.get_logger().info(f"[{self.active_backend}] nghe: '{text}'")

    def _run(self):
        try:
            if self.active_backend == 'google':
                self._run_google()
            elif self.active_backend == 'vosk':
                self._run_vosk()
            elif self.active_backend == 'whisper':
                self._run_whisper()
            elif self.active_backend == 'keyboard':
                self._run_keyboard()
            else:
                self.get_logger().error(f"backend khong hop le: {self.active_backend}")
        except Exception as exc:
            self.get_logger().error(f"loi backend '{self.active_backend}': {exc}")
            if self.backend_requested == 'auto' and self.active_backend == 'google':
                self.get_logger().warn("auto: google loi khi khoi dong, chuyen sang whisper")
                self.active_backend = 'whisper'
                self._run_whisper()

    def _run_google(self):
        import speech_recognition as sr
        recognizer = sr.Recognizer()
        recognizer.pause_threshold = 0.5
        recognizer.non_speaking_duration = 0.3
        recognizer.phrase_threshold = 0.2
        mic = sr.Microphone(device_index=self.device, sample_rate=16000)
        with mic as source:
            self.get_logger().info(
                f"Do tieng on nen {self.noise_calib_sec}s - giu im lang...")
            recognizer.adjust_for_ambient_noise(source, duration=self.noise_calib_sec)
        recognizer.energy_threshold = max(120, recognizer.energy_threshold * 0.85)
        recognizer.dynamic_energy_threshold = True
        self.get_logger().info(
            f"Google STT ({GOOGLE_LANG[self.lang]}) dang nghe, "
            f"nguong am {recognizer.energy_threshold:.0f}")

        def on_audio(rec, audio):
            try:
                result = rec.recognize_google(
                    audio, language=GOOGLE_LANG[self.lang], show_all=True)
            except sr.RequestError as exc:
                self.get_logger().error(f"Google STT loi mang: {exc}")
                if self.backend_requested == 'auto' and self.active_backend == 'google':
                    self.get_logger().warn("auto: mat mang luc dang nghe, chuyen sang whisper")
                    self.active_backend = 'whisper'
                    threading.Thread(target=self._run_whisper, daemon=True).start()
                return
            except Exception as exc:
                self.get_logger().error(f"Google STT loi: {exc}")
                return
            alternatives = result.get('alternative', []) if isinstance(result, dict) else []
            if not alternatives:
                return
            scored = [
                (match_score(alt.get('transcript', ''), self.grammar), -i,
                 alt.get('transcript', ''))
                for i, alt in enumerate(alternatives)
            ]
            _, _, text = max(scored)
            if text:
                self.publish_text(text)

        stop_fn = recognizer.listen_in_background(mic, on_audio, phrase_time_limit=5)
        while rclpy.ok() and self.active_backend == 'google':
            time.sleep(0.5)
        stop_fn(wait_for_stop=False)

    def _run_vosk(self):
        import sounddevice as sd
        import vosk
        vosk.SetLogLevel(-1)
        model = vosk.Model(self.vosk_model_path)
        sample_rate = int(sd.query_devices(self.device, 'input')['default_samplerate'])
        recognizer = vosk.KaldiRecognizer(model, sample_rate, json.dumps(self.grammar + ['[unk]']))
        recognizer.SetWords(True)
        self.get_logger().info(f"Vosk ({self.lang}) dang nghe @ {sample_rate} Hz")

        def callback(indata, frames, time_info, status):
            if recognizer.AcceptWaveform(bytes(indata)):
                res = json.loads(recognizer.Result())
                text = res.get('text', '').strip()
                if text and text != '[unk]':
                    self.publish_text(text)

        with sd.RawInputStream(
                samplerate=sample_rate, blocksize=int(sample_rate / 4), dtype='int16',
                channels=1, device=self.device, callback=callback):
            while rclpy.ok() and self.active_backend == 'vosk':
                time.sleep(0.2)

    def _run_whisper(self):
        import numpy as np
        import speech_recognition as sr
        from faster_whisper import WhisperModel
        os.makedirs(MODELS_DIR, exist_ok=True)
        model = WhisperModel('base', device='cpu', compute_type='int8', download_root=MODELS_DIR)
        recognizer = sr.Recognizer()
        recognizer.pause_threshold = 0.5
        mic = sr.Microphone(device_index=self.device, sample_rate=16000)
        with mic as source:
            recognizer.adjust_for_ambient_noise(source, duration=self.noise_calib_sec)
        self.get_logger().info('Whisper (base) dang nghe')

        def on_audio(rec, audio):
            raw = audio.get_raw_data(convert_rate=16000, convert_width=2)
            samples = np.frombuffer(raw, dtype=np.int16).astype(np.float32) / 32768.0
            segments, _ = model.transcribe(samples, language=WHISPER_LANG[self.lang])
            text = ' '.join(seg.text.strip() for seg in segments).strip()
            if text:
                self.publish_text(text)

        stop_fn = recognizer.listen_in_background(mic, on_audio, phrase_time_limit=5)
        while rclpy.ok() and self.active_backend == 'whisper':
            time.sleep(0.5)
        stop_fn(wait_for_stop=False)

    def _run_keyboard(self):
        self.get_logger().info('Che do ban phim: go cau lenh roi Enter')
        while rclpy.ok():
            try:
                line = input('> ')
            except EOFError:
                return
            if line.strip():
                self.publish_text(line.strip())


def main(args=None):
    rclpy.init(args=args)
    node = SpeechNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
