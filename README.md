# Tb4_VoiceNav_Map

Phân vùng bản đồ và điều hướng TurtleBot 4 bằng giọng nói tiếng Việt, chạy trên ROS 2 Humble, Ignition Gazebo Fortress, Nav2.

## Cấu trúc

- `tb4_voicenav/` — gói ROS 2 (ament_python)
  - `tb4_voicenav/segment_map.py` — phân vùng bản đồ `.pgm` bằng OpenCV, xuất ảnh nhãn + `regions.json`
  - `tb4_voicenav/region_store.py` — nạp và tra cứu vùng từ `regions.json`
  - `tb4_voicenav/command_parser.py` — chuẩn hóa văn bản, tách khu vực xuất phát/đích
  - `tb4_voicenav/speech_node.py` — micro → văn bản, publish `/voice_text`
  - `tb4_voicenav/voice_nav_node.py` — nhận `/voice_text`, gửi `NavigateToPose`
  - `launch/`, `config/`, `maps/`, `test/`, `tools/`
- `scripts/stop_all.sh`, `scripts/pin_cpus.sh`
- `docs/EVAL_SUMMARY.md` — số liệu đo thực tế

## Chạy test Python thuần (không cần ROS)

```
cd tb4_voicenav
python3 -m pytest test/test_command_parser.py
```

## Dừng tiến trình

```
scripts/stop_all.sh
```

## Dữ liệu

Âm thanh thô, video demo và model nhận dạng giọng nói không lưu trong repo này.
