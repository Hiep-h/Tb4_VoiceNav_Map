# voice_gui.py

Giao diện điều khiển bằng giọng nói: nút mic bên trái, bản đồ phân vùng bên phải.

    python3 tb4_voicenav/tools/voice_gui.py --stt whisper --parse --ros

- Bấm nút mic để bắt đầu nói, bấm lần nữa hoặc ngừng nói để kết thúc. Nút xám: chờ; đỏ: đang nghe; vàng: đang nhận dạng.
- `--stt whisper|vosk|google|auto`, `--whisper-model small` (mặc định), `--energy` đặt ngưỡng âm lượng, `--hold-to-talk` giữ để nói, `--no-map` ẩn bản đồ, `--debug` hiện độ trễ.
- `--parse`: hiện lộ trình đã tách từ câu nói. `--ros`: publish `/voice_text`, đọc `/voice_nav/status` và vẽ vị trí robot, đường đi.

Không chạy `speech_node` cùng lúc với `voice_gui.py --ros`: cả hai cùng publish `/voice_text`.
