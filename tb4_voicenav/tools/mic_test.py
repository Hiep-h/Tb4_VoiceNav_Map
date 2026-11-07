#!/usr/bin/env python3
"""Kiem tra micro + nhan dang giong noi cho tb4_voicenav, khong can ROS.

    python3 tools/mic_test.py --stt vosk --lang vi
    python3 tools/mic_test.py --stt google --lang vi --device 9
"""
import argparse
import json
import queue
import time

import numpy as np

from tb4_voicenav.command_parser import parse_command, build_command_phrases
from tb4_voicenav.region_store import RegionStore

DEFAULT_REGIONS_JSON = '/home/hiep/Tb4_VoiceNav/tb4_voicenav/maps/regions.json'
DEFAULT_VOSK_MODEL_PATH = '/home/hiep/tb4_voicenav_data/models/vosk-model-small-vn-0.4'


def explain(text, region_store):
    result = parse_command(text, region_store)
    if result.error:
        return f"LOI ({result.error}): {result.error_detail or ''}"
    return 'lenh hop le -> ' + ' -> '.join(r.name for r in result.targets)


def run_vosk(args, region_store, grammar):
    import sounddevice as sd
    import vosk

    print('=== THIET BI MICRO ===')
    print(sd.query_devices())
    dev = sd.query_devices(args.device, 'input')
    sample_rate = int(dev['default_samplerate'])
    print(f"=> Dung: [{args.device}] {dev['name']} @ {sample_rate} Hz")

    vosk.SetLogLevel(-1)
    model = vosk.Model(args.vosk_model)
    recognizer = vosk.KaldiRecognizer(model, sample_rate, json.dumps(grammar + ['[unk]']))
    recognizer.SetWords(True)

    print(f"Noi tung cau lenh trong {args.seconds}s (Ctrl+C de dung som)...")
    peak = 0.0
    q = queue.Queue()

    def callback(indata, frames, time_info, status):
        q.put(bytes(indata))

    with sd.RawInputStream(
            samplerate=sample_rate, blocksize=int(sample_rate / 4), dtype='int16',
            channels=1, device=args.device, callback=callback):
        t_end = time.time() + args.seconds
        while time.time() < t_end:
            data = q.get()
            level = float(np.abs(np.frombuffer(data, np.int16)).mean())
            peak = max(peak, level)
            if recognizer.AcceptWaveform(data):
                res = json.loads(recognizer.Result())
                text = res.get('text', '').strip()
                if text:
                    print(f"  nghe: '{text}'  => {explain(text, region_store)}")
    print(f"Am luong lon nhat do duoc trong phien: {peak:.0f}")


def run_google(args, region_store):
    import speech_recognition as sr

    print('=== THIET BI MICRO ===')
    for i, name in enumerate(sr.Microphone.list_microphone_names()):
        print(f"  [{i}] {name}")

    recognizer = sr.Recognizer()
    recognizer.pause_threshold = 0.5
    with sr.Microphone(device_index=args.device, sample_rate=16000) as source:
        print('Do tieng on nen 1.5s - giu im lang...')
        recognizer.adjust_for_ambient_noise(source, duration=1.5)
        print(f"Nguong am: {recognizer.energy_threshold:.0f}")
        t_end = time.time() + args.seconds
        while time.time() < t_end:
            try:
                audio = recognizer.listen(
                    source, timeout=max(1, t_end - time.time()), phrase_time_limit=5)
            except sr.WaitTimeoutError:
                break
            try:
                result = recognizer.recognize_google(
                    audio, language='vi-VN' if args.lang == 'vi' else 'en-US', show_all=True)
            except sr.RequestError as exc:
                print(f"Loi mang: {exc}")
                break
            alternatives = result.get('alternative', []) if isinstance(result, dict) else []
            if alternatives:
                text = alternatives[0].get('transcript', '')
                print(f"  nghe: '{text}'  => {explain(text, region_store)}")
            else:
                print('  (co tieng nhung khong ra chu)')


def main():
    ap = argparse.ArgumentParser(
        description='Kiem tra micro va nhan dang giong noi cho tb4_voicenav')
    ap.add_argument('--stt', default='vosk', choices=['vosk', 'google'])
    ap.add_argument('--device', type=int, default=None)
    ap.add_argument('--lang', default='vi', choices=['vi', 'en'])
    ap.add_argument('--seconds', type=int, default=15)
    ap.add_argument('--regions', default=DEFAULT_REGIONS_JSON)
    ap.add_argument('--vosk-model', default=DEFAULT_VOSK_MODEL_PATH)
    args = ap.parse_args()

    region_store = RegionStore().load(args.regions)
    grammar = build_command_phrases(region_store)

    if args.stt == 'vosk':
        run_vosk(args, region_store, grammar)
    else:
        run_google(args, region_store)


if __name__ == '__main__':
    main()
