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

## Chạy demo

Cần ROS 2 Humble, Nav2, TurtleBot 4 (mô phỏng Ignition Gazebo). Mỗi lệnh chạy trong một terminal riêng, đã `source /opt/ros/humble/setup.bash`.

```
bash scripts/start_demo.sh
python3 tb4_voicenav/tools/voice_gui.py --stt whisper --parse --ros
rviz2 -d tb4_voicenav/config/voice_demo.rviz
```

- `start_demo.sh` khởi động Gazebo, Nav2 và các node, tự thử lại nếu mô phỏng chưa sẵn sàng.
- `voice_gui.py`: bấm nút mic để nói, nhả để dừng. Cửa sổ vẽ bản đồ đã tô vùng, lộ trình và vị trí robot. `--stt` nhận `whisper`, `vosk`, `google`; `--whisper-model` mặc định `small`.
- Ví dụ lệnh: "đi đến phòng ngủ", "đi đến phòng khách rồi đến phòng bếp". Tên khu vực không có trong `regions.json` được báo "Khu vực không tồn tại".

## Kết quả phân vùng

- Bản đồ gốc: `tb4_voicenav/maps/map.pgm`, `map.yaml`
- Bản đồ phân vùng: `tb4_voicenav/maps/map_segmented.png`
- Vùng và goal (tọa độ mét): `tb4_voicenav/maps/regions.json`

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
