#!/usr/bin/env python3
"""Do do chinh xac + do tre nhan dang giong noi cho tb4_voicenav bang micro thuc.

Chay va NOI theo huong dan tren man hinh (dem nguoc 3-2-1 roi noi ngay):

    python3 tools/eval_voice.py --stt vosk --reps 10
    python3 tools/eval_voice.py --stt google --reps 10

Ghi ket qua (do chinh xac + do tre trung binh) ra man hinh de dan vao
docs/EVAL_SUMMARY.md - khong tu ghi file.
"""
import argparse
import io
import time
import wave

import sounddevice as sd

from tb4_voicenav.command_parser import parse_command
from tb4_voicenav.region_store import RegionStore

DEFAULT_REGIONS_JSON = '/home/hiep/Tb4_VoiceNav/tb4_voicenav/maps/regions.json'
DEFAULT_VOSK_MODEL_PATH = '/home/hiep/tb4_voicenav_data/models/vosk-model-small-vn-0.4'
SAMPLE_RATE = 16000
RECORD_SECONDS = 3.0

PHRASES = [
    ('đi đến phòng khách', ['Phòng khách']),
    ('đi đến phòng ngủ', ['Phòng ngủ']),
    ('từ phòng khách đến phòng bếp', ['Phòng khách', 'Phòng bếp']),
    ('đi đến hành lang', None),  # khu vuc khong ton tai -> phai bi loi
]


def countdown_and_record(prompt):
    print(f"\nNoi: '{prompt}'")
    for n in (3, 2, 1):
        print(n, end=' ', flush=True)
        time.sleep(0.6)
    print('NOI NGAY!')
    t_start = time.time()
    audio = sd.rec(
        int(RECORD_SECONDS * SAMPLE_RATE), samplerate=SAMPLE_RATE,
        channels=1, dtype='int16')
    sd.wait()
    return audio, t_start


def recognize_vosk(recognizer, audio):
    recognizer.AcceptWaveform(audio.tobytes())
    import json
    res = json.loads(recognizer.FinalResult())
    return res.get('text', '').strip()


def recognize_google(sr_module, recognizer, audio, lang):
    buf = io.BytesIO()
    with wave.open(buf, 'wb') as wav_file:
        wav_file.setnchannels(1)
        wav_file.setsampwidth(2)
        wav_file.setframerate(SAMPLE_RATE)
        wav_file.writeframes(audio.tobytes())
    buf.seek(0)
    with sr_module.AudioFile(buf) as source:
        audio_data = recognizer.record(source)
    try:
        return recognizer.recognize_google(
            audio_data, language='vi-VN' if lang == 'vi' else 'en-US')
    except (sr_module.UnknownValueError, sr_module.RequestError) as exc:
        print(f"  (loi nhan dang: {exc})")
        return ''


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--stt', required=True, choices=['vosk', 'google'])
    ap.add_argument('--reps', type=int, default=10)
    ap.add_argument('--lang', default='vi')
    ap.add_argument('--regions', default=DEFAULT_REGIONS_JSON)
    ap.add_argument('--vosk-model', default=DEFAULT_VOSK_MODEL_PATH)
    args = ap.parse_args()

    region_store = RegionStore().load(args.regions)

    if args.stt == 'vosk':
        import vosk
        vosk.SetLogLevel(-1)
        model = vosk.Model(args.vosk_model)
    else:
        import speech_recognition as sr
        recognizer = sr.Recognizer()

    print(f"=== DO BACKEND: {args.stt} ({args.reps} lan / cau) ===")
    results = []
    for phrase, expected_names in PHRASES:
        n_correct = 0
        latencies = []
        for i in range(args.reps):
            if args.stt == 'vosk':
                recognizer_v = vosk.KaldiRecognizer(model, SAMPLE_RATE)
                recognizer_v.SetWords(True)
                audio, t_start = countdown_and_record(phrase)
                text = recognize_vosk(recognizer_v, audio)
            else:
                audio, t_start = countdown_and_record(phrase)
                text = recognize_google(sr, recognizer, audio, args.lang)
            t_end = time.time()
            latency = t_end - t_start
            result = parse_command(text, region_store) if text else None
            if expected_names is None:
                ok = (result is not None and not result.ok)
            else:
                ok = (result is not None and result.ok
                      and [r.name for r in result.targets] == expected_names)
            n_correct += int(ok)
            latencies.append(latency)
            status = 'DUNG' if ok else 'SAI'
            print(f"  [{i + 1}/{args.reps}] nghe='{text}' -> {status} "
                  f"(do tre {latency:.2f}s)")
        avg_latency = sum(latencies) / len(latencies)
        results.append((phrase, n_correct, args.reps, avg_latency))

    print('\n=== KET QUA ===')
    print(f"{'Cau lenh':<40} {'Dung/Tong':<10} {'Do tre TB (s)':<15}")
    for phrase, n_correct, reps, avg_latency in results:
        print(f"{phrase:<40} {n_correct}/{reps:<8} {avg_latency:.2f}")


if __name__ == '__main__':
    main()
