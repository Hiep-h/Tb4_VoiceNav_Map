# voice_gui.py

Giao diện thử giọng nói: đếm ngược 3-2-1, hiện "HÃY NÓI!", rồi hiện nội dung đã nói.

    python3 tools/voice_gui.py --stt vosk --parse
    python3 tools/voice_gui.py --stt google --parse

SPACE: nói tiếp. ESC: thoát. `--loop`: tự lặp lại sau `--loop-wait` giây (mặc định 3s), không cần nhấn SPACE. `--debug`: hiện thêm độ trễ nhận dạng và các phương án khác.

## Chế độ --ros

    python3 tools/voice_gui.py --stt google --parse --ros

Khi bật `--ros`: mỗi câu nhận dạng được (không rỗng) được publish lên `/voice_text`
để `voice_nav_node` nhận; giao diện đồng thời đọc `/voice_nav/status` và hiện ngay
dưới dòng nội dung đã nói.

KHÔNG chạy `speech_node` cùng lúc với `voice_gui.py --ros` - cả hai đều publish
`/voice_text`, chạy cùng lúc sẽ có hai nguồn ghi đè lên nhau trên cùng một topic.
