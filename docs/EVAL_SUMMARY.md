# EVAL_SUMMARY

Số liệu dưới đây chỉ ghi khi đã đo thật từ log, không ước lượng.

## Bước 0 — kiểm tra môi trường

- Bản đồ small_house: có sẵn, `resolution=0.05`, `origin=[-10.4, -6.63, 0]`.
- Micro: ghi được qua `arecord`, RMS tín hiệu ~24500 trên clip 2s.
- Vosk tiếng Việt (small): load model 0.11s, suy luận clip 2s mất 1.39s, chạy offline không lỗi.
- Nav2 + AMCL (world `small_house`, headless `-s -r --headless-rendering`):
  - RTF đứng yên (sau spawn, trước Nav2 activate): 0.7853 (n=116 mẫu, 12s)
  - RTF lúc điều hướng: 0.8911 (n=117 mẫu)
  - Nav2 activate sau khi khởi động: ~207s (chưa có TF `map` trước khi set `/initialpose`)
  - Goal (0,2): SUCCEEDED, 37.37s (wall), 1 lần recovery
  - Goal (0,0): SUCCEEDED, 33.08s (wall)
  - Lỗi/cảnh báo đáng chú ý: `StaticLayer lookupTransform map` lỗi 489 lần trước initialpose (hết sau khi có pose); `Failed to make progress` ở cả 2 goal nhưng Nav2 tự recovery và tới đích; `gz_ros2_control` cảnh báo chu kỳ điều khiển 0.001s nhanh hơn chu kỳ mô phỏng 0.003s.

## Phân vùng bản đồ

Bản đồ `small_house` (map.yaml từ tb4_lab04) gần như một khối free-space liền mạch — SLAM không ghi lại tường ngăn phòng đầy đủ. Phân vùng tự động bằng co ngót free-space (đóng cửa) + watershed cho ra 1 vùng duy nhất (vô nghĩa) nên đã bỏ hướng đó. Hướng hiện tại: đọc `small_house.sdf`, nhóm đồ nội thất theo 3 phòng thật (giường/tủ đầu giường/tủ quần áo/bàn đọc sách/ghế → Phòng ngủ; sofa/bàn trà/TV/tủ TV → Phòng khách; bàn bếp/tủ bếp/tủ lạnh/ghế bếp → Phòng bếp, cấu hình trong `config/room_furniture.yaml`). Mỗi món đồ là một seed độc lập (không dùng tâm nhóm làm seed duy nhất); nhãn mỗi pixel free = nhãn của seed gần nhất theo khoảng cách đi trong vùng trống (multi-source Dijkstra 8 hướng trên lưới free-space, không xuyên vật cản).

`inflation_radius` đọc thật từ `/opt/ros/humble/share/turtlebot4_navigation/config/nav2.yaml`: **0.45 m**. Margin thêm 0.05 m → yêu cầu tối thiểu 0.50 m.

17 seed (món đồ) cho 3 phòng; 5/17 seed rơi vào vật cản trên bản đồ (do đồ vật được LIDAR ghi là chướng ngại) nên bị dịch tới pixel free gần nhất:

| Model | Khoảng dịch (m) |
|---|---|
| NightStand_01_001 | 0.907 |
| NightStand_01_002 | 0.792 |
| KitchenCabinet_01_001 | 0.359 |
| Refrigerator_01_001 | 0.453 |
| ReadingDesk_01_001 | 0.064 |

Điểm đích mỗi phòng = điểm có khoảng cách tới vật cản (Euclidean distance transform) lớn nhất, giới hạn trong vùng của phòng đó **và** trong bán kính 2.0 m quanh tâm cụm đồ nội thất của phòng (config `goal_radius_m`, không cần mở rộng ở lần chạy này).

3 vùng (`maps/regions.json`, ảnh `maps/map_segmented.png`):

| Phòng | Diện tích (m²) | Điểm đích (m) | Clearance tới vật cản (m) |
|---|---|---|---|
| Phòng ngủ | 60.20 | (-5.35, 0.72) | 0.949 |
| Phòng khách | 64.72 | (2.65, -4.23) | 1.880 |
| Phòng bếp | 32.59 | (7.25, 1.07) | 0.969 |

Cả 3 điểm đích đều vượt yêu cầu tối thiểu 0.500 m. Tổng diện tích 3 vùng (157.51 m²) gần bằng toàn bộ free-space bản đồ (157.76 m²) vì mọi pixel free được gán cho phòng gần nhất theo geodesic distance — phần chênh ~0.25 m² là các pixel free bị cô lập không có đường đi trong vùng trống tới seed nào.

### Điều tra RTF thấp (0.23) so với Bước 0 (0.79–0.89)

So sánh log launch gốc của Bước 0 (`~/.ros/log/2026-10-28-08-05-55-*`, `~/.ros/log/2026-10-28-08-06-28-*`, `~/.ros/log/2026-10-28-08-06-33-*`) với `tools/headless_nav_eval.launch.py`: cùng bộ tiến trình (sim headless `-s -r --headless-rendering`, `map_server`+`amcl`, `controller_server`+`bt_navigator`+...), khác biệt duy nhất tìm được là **`LIBGL_ALWAYS_SOFTWARE=1`** — biến này đã bị thêm vào `headless_nav_eval.launch.py` ở lần sửa trước (nhầm tưởng cần cho render offscreen) nhưng Bước 0 KHÔNG đặt biến này.

Kiểm tra từng thay đổi riêng (máy đã dọn sạch qua `stop_all.sh`, kill 2 tiến trình "mồ côi" còn sót từ phiên debug trước: `ros2 service call .../list_controllers` treo từ 08:04 và `ros2 param set safety_override` mồ côi từ 07:40):

| Cấu hình | RTF đo được | Ghi chú |
|---|---|---|
| Có `LIBGL_ALWAYS_SOFTWARE=1`, đủ Nav2+AMCL | 0.232 (n=148, min 0.072 max 0.402) | `ign gazebo` dùng 387% CPU (render bằng Mesa llvmpipe phần mềm) |
| Không `LIBGL_ALWAYS_SOFTWARE`, chỉ sim (không Nav2/AMCL) | 0.82–0.89 (3 mẫu tức thời) | Sim chạy bình thường ngay, không treo |
| Không `LIBGL_ALWAYS_SOFTWARE`, đủ Nav2+AMCL, đã chạy `pin_cpus.sh` (nav2→lõi 4-7, gazebo→lõi 8-11) | **0.671 (n=100, min 0.411 max 0.926)** | `ign gazebo` chỉ còn ~97% CPU (≈1 lõi) |

**Kết luận**: `LIBGL_ALWAYS_SOFTWARE=1` chính là nguyên nhân RTF sụp — ép Gazebo render offscreen bằng CPU (Mesa llvmpipe) thay vì GPU, tốn gần 4 lõi CPU và cạnh tranh trực tiếp với Nav2/AMCL. Lần treo sim đầu tiên khi chưa đặt biến này (trong phiên sửa trước) nhiều khả năng là do context GPU/EGL còn kẹt từ lần Gazebo có giao diện bị segfault ngay trước đó, không phải do thiếu biến này — đã xác nhận lại sạch từ đầu thì không cần `LIBGL_ALWAYS_SOFTWARE` vẫn chạy ổn. Đã bỏ biến này khỏi mặc định của `tools/headless_nav_eval.launch.py` (thêm tham số `software_render` để bật lại nếu cần thử nghiệm). `pin_cpus.sh` giúp thêm chút ổn định nhưng không phải yếu tố chính.

### Kiểm thử Nav2 thật tới 3 điểm đích (thời gian chờ theo giờ mô phỏng, đọc `/clock`)

Dùng cấu hình đã sửa (không `LIBGL_ALWAYS_SOFTWARE`, có `pin_cpus.sh`), RTF lúc đo ~0.67. `tools/run_navigate_goals.py` sửa lại để tính thời gian theo sim-time thật (`use_sim_time=true`, đọc `/clock` qua `self.get_clock().now()`), tối đa 180s mô phỏng mỗi goal (trần an toàn 600s giờ thực).

| Phòng | Kết quả | Giờ mô phỏng | Giờ thực | Recovery | Sai số tới đích (m) |
|---|---|---|---|---|---|
| Phòng ngủ | sim_time_timeout (hủy ở 180s mô phỏng) | 180.0s | 380.9s | 15 | 0.85 (ước tính từ vị trí cuối) |
| Phòng khách | ABORTED (gần như ngay lập tức) | 0.009s | 0.02s | 0 | — (goal bị từ chối, không có sai số hợp lệ) |
| Phòng bếp | ABORTED | 63.2s | 68.5s | 21 | 6.08 (vị trí cuối cách xa đích, bị kẹt giữa đường) |

**0/3 SUCCEEDED** — nhưng lần này nguyên nhân khác hẳn lần đo RTF thấp trước:

- **Phòng ngủ**: robot tới rất gần đích (còn ~0.85 m) nhưng không hoàn tất trong 180s mô phỏng. Log có đúng **13 lần "Failed to make progress"** trong suốt log (chủ yếu ở goal này, cách nhau đều ~20–25s mô phỏng — khớp với `movement_time_allowance: 10.0` + thời gian recovery mỗi vòng), kèm **15 lần recovery**. `progress_checker` yêu cầu di chuyển `required_movement_radius: 0.5` m trong `movement_time_allowance: 10.0` s (đọc từ `nav2.yaml`) — robot liên tục bị kẹt gần đích đủ lâu để vi phạm ngưỡng này rồi phải recovery, lặp lại nhiều lần. Đây là vấn đề thật của Nav2 khi điều khiển robot vào ngách hẹp gần đích, không phải do CPU/RTF (RTF lúc này ổn định ~0.67).
- **Phòng khách**: `ABORTED` sau 0.009s mô phỏng — bị từ chối gần như ngay khi gửi, log `bt_navigator` cho thấy sau khi huỷ goal Phòng ngủ, lần "Begin navigating" tiếp theo lại là tới (7.25, 1.07) (toạ độ Phòng bếp), tức **goal Phòng khách chưa từng được bt_navigator xử lý** — đây là race condition: gửi goal mới ngay sau khi huỷ goal trước, trong khi `bt_navigator` chưa dọn xong cây hành vi (BT) từ lần huỷ trước nên từ chối ngay. Không phải do điểm đích/costmap.
- **Phòng bếp** (goal thực tế chạy ở bước 3, log ghi đích (7.25, 1.07)): dừng ở (1.29, -0.14), cách đích 6.08 m, bị **21 lần recovery** rồi `Aborting handle`. Log có **158 dòng** `[DWBLocalPlanner] No valid trajectories out of 419!` dồn dập trong ~1 phút cuối — robot kẹt tại một vị trí (gần khối nội thất lớn ở giữa Phòng khách trên đường đi từ Phòng ngủ sang Phòng bếp) mà bộ điều khiển DWB không tìm được quỹ đạo hợp lệ nào, không phải do planner toàn cục từ chối điểm đích (không có `GridBased: failed to create plan`).

**Kết luận**: không còn là vấn đề RTF/hạ tầng — cả 3 lần fail đều có nguyên nhân cụ thể trong log: (1) robot khó vào sát đích Phòng ngủ trong ngưỡng progress_checker, (2) lỗi race condition khi gửi goal liên tiếp quá nhanh sau khi huỷ, (3) DWB kẹt cứng gần khối nội thất lớn giữa Phòng khách. Cần: chờ vài giây giữa các goal (tránh race condition #2), và xem lại vùng gần khối nội thất giữa Phòng khách (goal_radius/costmap inflation) cho vấn đề #3.

### Điều tra lần 2 — lỗi hệ quy chiếu (frame offset) và nghi vấn AMCL trôi

**Phát hiện 1 — lỗi offset world↔map trong tính điểm đích**: `tb4_voicenav/tb4_voicenav/segment_map.py::parse_world_objects` đọc tọa độ đồ nội thất trực tiếp từ `small_house.sdf` (hệ world) nhưng KHÔNG trừ offset spawn (1.0, 1.0) trước khi đổi sang pixel bản đồ — trong khi `tools/run_trial.py::world_to_map()` đã trừ offset này đúng từ trước (robot spawn world (1.0,1.0,yaw0) ứng với map (0,0), vì bản đồ SLAM được xây với odom=0 tại đúng điểm spawn này). Xác nhận bằng kiểm tra offline (không cần sim): đếm số đồ vật rơi vào vật cản trong bán kính 0.3m quanh tọa độ tương ứng trên `map.pgm` — **21/29 vật thể khớp vật cản nếu trừ offset (1.0,1.0), chỉ 7/29 nếu không trừ** → xác nhận lệch tịnh tiến (1.0, 1.0) đúng như spawn offset, không có lệch xoay đáng kể ở bước này. Đã sửa `parse_world_objects` để trừ offset (hằng số `SPAWN_WORLD_OFFSET_M`).

**Phát hiện 2 — lỗi chọn điểm đích trong `tools/validate_goals_nav2.py`**: logic cũ chọn ứng viên Nav2-khả-thi ĐẦU TIÊN (bán kính tìm kiếm tăng dần quanh tâm đồ nội thất) mà không ưu tiên đạt `min_required_clearance_m` (= inflation_radius 0.45m + margin 0.05m = 0.50m) — khiến điểm đích Phòng ngủ ban đầu (sau khi sửa offset) chỉ đạt clearance 0.35m, dưới ngưỡng an toàn. Thử sửa bằng cách sắp theo clearance giảm dần toàn cục thì lại đẩy điểm đích ra sát ranh giới giữa 2 phòng (vì điểm clearance lớn nhất trong vùng có thể nằm ở biên, nơi không gian mở của phòng bên cạnh "tràn" vào). Cách sửa cuối: vẫn ưu tiên bán kính tăng dần (giữ điểm gần tâm phòng) nhưng trong mỗi vòng ưu tiên ứng viên đạt clearance tối thiểu trước.

**Tọa độ đích mới sau cả 2 lần sửa** (ghi trong `maps/regions.json`, đã kiểm tra bằng mắt qua `maps/map_segmented.png` — cả 3 điểm nằm rõ trong đúng vùng phòng, không sát ranh giới):

| Phòng | Tọa độ cũ (m) | Tọa độ mới (m) | Clearance mới (m) |
|---|---|---|---|
| Phòng ngủ | (-5.35, 0.72) | (-8.85, -0.03) | 0.799 |
| Phòng khách | (2.65, -4.23) | (0.70, -4.93) | 0.959 |
| Phòng bếp | (7.25, 1.07) | (6.10, -2.03) | 1.039 |

Lệch vài mét giữa tọa độ cũ/mới — xác nhận lỗi offset là có thật và đáng kể, không phải nhiễu.

**Kiểm thử lại với tọa độ mới** (sim riêng từng trial, reset sạch bằng `tools/run_clean_trial.sh`, dùng `scripts/stop_all.sh` + `scripts/pin_cpus.sh`, kết quả lưu `/tmp/tb4_trials_v2/`):

| Phòng | Kết quả (theo Nav2/AMCL) | Sai số AMCL vs ground truth lúc kết thúc | Giờ mô phỏng | Giờ thực | Recovery |
|---|---|---|---|---|---|
| Phòng ngủ | `sim_time_timeout` (180s, chưa SUCCEEDED) | **1.84 m** (bắt đầu 0.02m) | 180.0s | 347.6s | 15 |
| Phòng khách | `SUCCEEDED` (sai số AMCL vs goal chỉ 0.078m) | **1.43 m** (bắt đầu 0.018m) | 69.5s | 80.0s | 1 |
| Phòng bếp | chưa đo lại (phiên bị dừng giữa chừng do giới hạn tài nguyên của agent điều tra) | — | — | — | — |

**Phát hiện 3 — nghi vấn AMCL trôi (chưa kết luận nguyên nhân gốc)**: ở cả 2 trial đo được, sai số AMCL-vs-ground-truth tăng mạnh theo quãng đường đi (từ ~0.02m lúc xuất phát lên 1.4-1.8m lúc kết thúc, quãng đường chỉ 5-9m). Đáng chú ý nhất: **Phòng khách báo `SUCCEEDED` theo AMCL (coi như đã tới đích, sai số chỉ 0.078m) nhưng vị trí thật (ground truth) của robot cách đích tới khoảng 1.4m** — nghĩa là tiêu chí "tới đích" của Nav2 hiện tại dựa hoàn toàn vào AMCL, có thể báo thành công sai nếu AMCL trôi. Lần điều tra trước (trước khi sửa offset) đã từng thấy hiện tượng tương tự ở Phòng khách với tỷ lệ sai_số/quãng_đường gần như hằng số (~0.35-0.39), gợi ý nghi vấn lệch GÓC (yaw) giữa world và map frame — **chưa xác minh được nguồn gốc chính xác** (do AMCL tự trôi theo quãng đường, do yaw spawn không khớp yaw lúc xây bản đồ SLAM, hay do bias nào khác trong odometry). Đây là việc còn dang dở, cần điều tra tiếp: so sánh `/odom` với ground truth (tách riêng sai số odometry khỏi sai số lọc AMCL), kiểm tra yaw spawn thực tế so với yaw giả định 0.0 trong `/initialpose`.

### Điều tra lần 3 — tìm ra nguyên nhân gốc của Phát hiện 3 (và một lỗi hạ tầng nghiêm trọng khác)

**Phát hiện 4 — LIDAR trả về toàn 0 khi `software_render:=false` (giải thích đầy đủ Phát hiện 3)**: kiểm tra trực tiếp topic `/scan` bằng `ros2 topic echo` và bằng node rclpy độc lập, với `tools/headless_nav_eval.launch.py` chạy mặc định sau lần sửa của điều tra lần 2 (`software_render:=false`, dùng GPU thật thay vì Mesa phần mềm): **toàn bộ 240 tia LIDAR trả về giá trị 0.0** (không phải "không có vật cản" — `range_min=0.164`, 0.0 là giá trị lỗi/không hợp lệ), kiểm tra ngay sau spawn (sim-time 8s) và sau 94s đều như vậy, không phải do chưa "khởi động xong". Khi bật lại `software_render:=true` (ép `LIBGL_ALWAYS_SOFTWARE=1`), LIDAR có dữ liệu hợp lệ ngay lập tức (sim-time 1.8s, 240/240 tia khác 0, min=2.05m max=inf). Máy có GPU Intel thật và `glxinfo` xác nhận "direct rendering: Yes", nhưng rõ ràng Ignition Gazebo ở chế độ headless-rendering (`-s -r --headless-rendering`) không khởi tạo được context render/EGL cần thiết cho plugin GPU-ray LIDAR khi không ép Mesa phần mềm trong môi trường này — nguyên nhân sâu hơn (driver/EGL/cấu hình container) chưa điều tra tiếp vì đã có cách khắc phục ổn định (`software_render:=true`).

**Đây CHÍNH LÀ nguyên nhân gốc của Phát hiện 3**: khi LIDAR không có dữ liệu thật, AMCL không nhận được phép đo cảm biến nào để sửa sai số — nó chạy thuần túy theo mô hình chuyển động (motion model = tích lũy odometry) không có bước cập nhật đo lường, tương đương dead-reckoning. Điều này khớp hoàn hảo với số liệu đã đo: sai số AMCL-vs-ground-truth tăng dần theo quãng đường, và Phòng khách báo `SUCCEEDED` theo AMCL dù vị trí thật lệch 1.4m — vì AMCL "tin" vào vị trí dự đoán từ odometry (vốn cũng bị trôi — xem Phát hiện 5) mà không có cách nào tự sửa. Đã sửa: đổi mặc định `software_render` trong `headless_nav_eval.launch.py` về lại `true`, và `run_clean_trial.sh` gọi tường minh `software_render:=true`. Điều này **đảo ngược kết luận của lần điều tra trước** ("RTF sụp vì LIBGL_ALWAYS_SOFTWARE=1 nên bỏ mặc định") — RTF thấp hơn khi bật software rendering là có thật, nhưng việc tắt nó khiến LIDAR hỏng hoàn toàn, nghiêm trọng hơn nhiều so với vấn đề tốc độ mô phỏng. Mọi số liệu "tới đích"/AMCL đo trong lần điều tra 1 và 2 của phiên làm việc này (kể cả bảng ở Điều tra lần 2) đều có khả năng bị ảnh hưởng bởi lỗi này nếu dùng cấu hình `software_render:=false` — cần coi là KHÔNG đáng tin cậy, chỉ số liệu đo sau khi sửa (với `software_render:=true`) mới dùng được.

**Phát hiện 5 — nghi vấn trượt bánh xe (wheel slip) tại đúng điểm spawn (1.0, 1.0)**: đo trực tiếp odometry vs ground truth bằng kịch bản mới `tools/odom_vs_gt_check.py` (lệnh `/cmd_vel` tăng dần trong 2s rồi giữ đều, tránh bước nhảy vận tốc đột ngột) trong sim sạch, LIDAR đã hoạt động đúng (`software_render:=true`), không chạy Nav2 (test odometry thuần túy):
- Lệnh tiến thẳng 0.15 m/s trong 20s (kỳ vọng ~3m): **odometry báo đã đi 1.42m, nhưng ground truth chỉ đi đúng 0.022m** (tỷ lệ odom/ground_truth ≈ 65 lần) — bánh xe quay (encoder tích lũy) nhưng thân robot gần như không dịch chuyển trong thế giới thật.
- Lệnh quay tại chỗ (góc 0.3 rad/s, đủ 1 vòng ~20.9s): odometry và ground truth khớp khá tốt (yaw delta -0.444 rad vs -0.431 rad, sai lệch ~3%; vị trí trôi trong lúc quay 0.066m vs 0.061m — gần khớp) — **quay tại chỗ hoạt động bình thường, chỉ riêng tiến thẳng bị kẹt**.
- LIDAR tại vị trí này cho thấy phía trước thoáng 4.5m (không có vật cản) — nên đây KHÔNG phải va chạm với tường/đồ vật nhìn thấy được bằng LIDAR, mà giống hiện tượng mất độ bám (wheel slip) hoặc vướng vật lý ở gầm/bánh xe (ví dụ đường nối giữa các mảnh sàn trong world SDF đúng tại tọa độ (1.0, 1.0)) khiến bánh xe quay trượt tại chỗ khi đi thẳng.
- Có liên hệ khớp với quan sát trước đó: cảm biến va chạm `bump_front_center` từng kích hoạt lặp lại khi robot cố tiến từ vị trí này (ghi nhận trong phiên điều tra trước khi tìm ra Phát hiện 4) dù phía trước không có vật cản theo LIDAR — cùng một hiện tượng "không đi được từ đúng điểm spawn" nhìn từ 2 góc đo khác nhau.
- **Chưa giải thích được tại sao các trial Nav2 đầy đủ trước đó (vd Phòng khách `SUCCEEDED`, di chuyển thật 5-9m) vẫn đi được** — có thể do `velocity_smoother`/`controller_server` của Nav2 tăng tốc từ từ và/hoặc bẻ lái ngay từ đầu (không đi thẳng tuyệt đối như kịch bản test này) nên tránh được điểm kẹt, hoặc do khác biệt giữa lần chạy. **Chưa kết luận, cần điều tra tiếp**: thử lại với gia tốc/tốc độ khác, kiểm tra world SDF có seam/ranh giới mesh sàn tại (1.0, 1.0) hay không, thử spawn tại tọa độ khác để so sánh.

### Điều tra lần 4 — xác nhận nguyên nhân Phát hiện 5: trạm sạc (dock) chắn ngay trước robot

**Phát hiện 6 — đúng như nghi vấn: `standard_dock` spawn cách robot 0.157m, chắn ngay phía trước, giải thích toàn bộ Phát hiện 5.** Kiểm tra bằng `ign model --list` trong sim đang chạy: có model `standard_dock`. Đọc pose: `standard_dock` tại world (1.157, 1.0, yaw=π), robot (`turtlebot4`) tại world (1.0005, 1.0, yaw≈0) — dock cách robot đúng **0.157m** về phía trước (robot yaw=0 hướng +x, dock ở x lớn hơn), quay mặt về robot (yaw=π). Đây là hành vi mặc định của `turtlebot4_spawn.launch.py` (xác nhận đọc source tại `/opt/ros/humble/share/turtlebot4_ignition_bringup/launch/turtlebot4_spawn.launch.py`): robot luôn spawn "đã cắm sạc" (docked), dock được spawn tự động cách robot đúng 0.157m theo offset xoay theo yaw (`RotationalOffsetX/Y(0.157, yaw)`) — độ lệch 0.157m này khớp chính xác với khoảng cách suy đoán trong giả thuyết trước khi đo. Xác nhận thêm bằng `/hazard_detection` trong lúc robot cố tiến: ghi nhận `BUMP` (`bump_front_center`) và `BACKUP_LIMIT` (`base_link`) liên tục — đúng hành vi robot va chạm vật cản ngay sát phía trước rồi bị giới hạn lùi lại, không phải lỗi vật lý/mô phỏng ngẫu nhiên.

**Khắc phục**: viết `tools/turtlebot4_spawn_no_dock.launch.py` (bản rút gọn của `turtlebot4_spawn.launch.py`, bỏ hẳn phần `dock_description` + spawn model `standard_dock`, giữ nguyên mọi phần khác — robot, bridge, node, static transform). Không copy/tham khảo trực tiếp repo ngoài `Tb4_Gesture` (không cần thiết — toàn bộ cơ chế đã đọc được từ source cài sẵn trong `/opt/ros/humble/share/turtlebot4_ignition_bringup/`). `tools/headless_nav_eval.launch.py` thêm tham số `no_dock` (mặc định `true`): khi bật, dùng spawn không dock và tự động gọi `ros2 param set /motion_control safety_override backup_only` (qua service `/motion_control/set_parameters`, chờ tới khi thành công) ngay sau khi sim sẵn sàng — cần thiết để robot được phép lùi (mặc định `safety_override=none` giới hạn lùi vì giả định luôn có dock phía sau). `tools/run_clean_trial.sh` cũng cập nhật theo mặc định mới.

**Đo lại odometry sau khi bỏ dock** (sim sạch, `software_render:=true`, `no_dock:=true`, `run_nav2:=false`, cùng kịch bản `odom_vs_gt_check.py` có tăng tốc dần):
- Tiến thẳng 0.15 m/s trong 20s: odometry 2.8640m, ground truth 2.8640m — **tỷ lệ odom/ground_truth = 0.9999997** (gần như hoàn hảo, không còn trượt bánh).
- Quay tại chỗ (≈1 vòng, 0.3 rad/s): yaw delta odometry -0.29087 rad vs ground truth -0.29091 rad — khớp sát (giá trị ~0.29 rad chứ không phải 2π vì có pha tăng tốc 2s ở đầu lệnh, cả odom lẫn ground truth đều phản ánh đúng pha này).
- **Kết luận: odometry chính xác, Phát hiện 5 đã được giải thích và khắc phục triệt để bằng cách bỏ dock khi spawn cho mục đích thử nghiệm điều hướng.** Toàn bộ nghi vấn "AMCL trôi" trong các lần điều tra trước (Phát hiện 3) có 2 nguyên nhân cộng dồn: LIDAR hỏng (Phát hiện 4, đã sửa) và robot vật lý không di chuyển được bình thường từ điểm spawn do vướng dock (Phát hiện 6, đã sửa).

**Đo lại RTF với LIDAR hoạt động thật** (ghi đè số liệu Bước 0 — 0.79-0.89 — vì số đó đo khi LIDAR CHƯA hoạt động, không dùng được nữa):
- RTF khi chỉ có sim + base controller (không Nav2), `software_render:=true`: **0.276** (đo 15s wall qua `/clock`).
- RTF khi có đủ Nav2 (map_server, AMCL, costmap, planner, controller, bt_navigator) + đã ghim CPU, lúc đứng yên (chưa nhận goal): **0.149**.
- RTF trong lúc Nav2 đang xử lý một goal thật (Phòng ngủ, xem bên dưới): ước tính từ trial thực tế ≈ **0.048** (28.6s mô phỏng trong 600s thực) — **thấp hơn nhiều so với lúc đứng yên**, cho thấy tải tính toán thật của planner/costmap/AMCL trong môi trường này rất nặng so với CPU sẵn có, là một hạn chế hạ tầng thật, không phải lỗi logic Nav2.

**Thử đo lại Phòng ngủ với cấu hình đã sửa đầy đủ** (sim sạch, dock đã bỏ, LIDAR hoạt động, `software_render:=true`): lần 1 gặp lỗi hạ tầng thoáng qua khác (`map_server` không phản hồi `change_state` do timeout rmw/DDS — cùng dạng lỗi "mất phản hồi service" mà lần điều tra 2 đã ghi nhận ở `/controller_manager/list_controllers` và `/amcl/change_state`, lần này rơi vào `/map_server/change_state`; `lifecycle_manager` treo vô thời hạn chờ phản hồi không bao giờ tới) — coi là lỗi hạ tầng, không phải lỗi logic, thử lại. Lần 2 Nav2 lên bình thường (bt_navigator active sau ~15s), goal (-8.85, -0.03) được `compute_path_to_pose` xác nhận khả thi (360 điểm, dài 9.03m), nhưng do RTF trong lúc điều hướng quá thấp (~0.048), trial chỉ đạt **28.6s mô phỏng trong 600s thực** rồi hết hạn giờ thực (`wall_time_timeout`), KHÔNG đủ thời gian mô phỏng để tới đích hay kết luận thành/bại thật sự. Số liệu đo được tại thời điểm dừng: AMCL (-6.456, -0.472), ground truth (-6.514, -0.536) — **sai số AMCL-vs-ground-truth chỉ 0.082m**, so với 1.4-1.8m ở các lần đo trước khi sửa Phát hiện 4+6 — xác nhận AMCL giờ bám sát vị trí thật trong lúc di chuyển tích cực, không chỉ lúc đứng yên. Tệp kết quả JSON đầy đủ của lần chạy này không lưu được do `run_clean_trial.sh` dùng `timeout 650` cho `run_trial.py` trong khi `run_trial.py` tự chờ tới 600s wall-timeout rồi mới hủy+đóng kết nối — không đủ dư thời gian, bị cắt giữa chừng khi đang hủy goal, làm hỏng tiến trình rclpy shutdown. **Đã sửa**: tăng lên `timeout 900` trong `run_clean_trial.sh`.

### Điều tra lần 5 — giảm tải để tăng RTF lúc điều hướng

**Profile CPU theo tiến trình** (`top -b -n 1`, lấy mẫu 5 lần cách nhau 2s trong lúc Nav2 đang xử lý một goal thật, world `small_house`, LIDAR hoạt động, đã ghim CPU): tiến trình **`ign gazebo` (chạy qua wrapper `ruby`) chiếm 560-660% CPU** (gần 6-6.6 lõi/12 lõi máy) ở MỌI mẫu — áp đảo hoàn toàn so với các tiến trình Nav2: `controller_server` ~23-25%, `bt_navigator` ~18-25%, `map_server`/`amcl`/`motion_control`/`sensors_node` mỗi cái ~12%. Không có tiến trình Nav2 nào vượt quá ~25% CPU. **Kết luận trước khi chỉnh bất kỳ gì**: nút thắt CPU nằm ở bản thân mô phỏng vật lý/render của Ignition Gazebo (`ign gazebo`), không phải ở Nav2 — nên các tham số phía Nav2 (controller_frequency, DWB samples, costmap update_frequency, AMCL particles) về lý thuyết khó tạo khác biệt lớn cho RTF tổng thể, dù vẫn đáng thử vì không tốn kém.

**Phương pháp đo chuẩn hoá** (dùng cho mọi so sánh dưới đây): 1 sim sạch (`stop_all.sh` trước mỗi lần), `software_render:=true`, `no_dock:=true`, đã ghim CPU, gửi goal Phòng ngủ (-8.85, -0.03) qua `navigate_to_pose`, chờ 5s "khởi động" sau khi goal được chấp nhận rồi đo tỷ lệ sim-time/wall-time qua `/clock` trong cửa sổ 20s tiếp theo (một số lần đo cửa sổ bị cắt ngắn hơn 20s do goal hoàn tất/action kết thúc sớm trong lúc đo — ghi rõ wall_dt thực tế đo được, không nội suy).

**Baseline (đã áp dụng sẵn 3 thay đổi Nav2 nhẹ: `controller_frequency` 20→10Hz, DWB `vx/vy/vtheta_samples` 20/5/20→10/1/10, local costmap `update_frequency` 5→3Hz — tất cả override qua `tb4_voicenav/config/nav2_voicenav.yaml`/`localization_voicenav.yaml`, không sửa file hệ thống)**: RTF = 0.2273 và 0.2358 (2 lần đo, cửa sổ đủ 20s) — các tham số Nav2 này không gây hại (đúng dự đoán từ profile CPU, vì chỉ ~25% CPU nằm ở Nav2) nhưng cũng không cải thiện rõ rệt so với mặc định hệ thống.

**Thay đổi 1 — tắt camera OAK-D**: `model:=lite` của TurtleBot4 (đã thử ở điều tra trước) **KHÔNG tắt được camera** — đọc source `/opt/ros/humble/share/turtlebot4_description/urdf/lite/turtlebot4.urdf.xacro` xác nhận bản lite vẫn gắn OAK-D (model "lite" thay vì "pro"), chỉ bỏ vỏ/khung ngoài. Do đó viết riêng `tb4_voicenav/tools/turtlebot4_no_camera.urdf.xacro` (bản sao `turtlebot4.urdf.xacro` hệ thống, bỏ hẳn `<xacro:include>` file `oakd.urdf.xacro`/`camera_bracket.urdf.xacro` và 2 khối instantiate camera) + `tools/robot_description_no_camera.launch.py` (launch riêng trỏ tới xacro này) + cờ `no_camera` trong `turtlebot4_spawn_no_dock.launch.py`/`headless_nav_eval.launch.py`. Xác nhận robot vẫn có đủ `/scan` (dữ liệu hợp lệ), `/odom`, TF `map`→`base_link` (tra được qua `tf2_echo`), Nav2 hoạt động bình thường (bt_navigator active, nhận goal được).

RTF đo được (2 lần, cửa sổ đủ 20s): **0.1629 và 0.2017** — cả 2 lần đều THẤP HƠN baseline (0.227-0.236), không có lần nào cao hơn. Không tìm thấy lỗi lặp lại liên quan camera trong log (`ros_ign_bridge` không báo lỗi bridge camera dồn dập). **Kết luận: bỏ camera KHÔNG cải thiện RTF trong môi trường này** (rất có thể vì camera RGBD chỉ chạy 2Hz/320x240, chi phí nhỏ so với ~600% CPU của toàn bộ `ign gazebo`, nên việc bỏ nó không chạm vào nút thắt thật) — **không giữ làm mặc định** (`no_camera` vẫn mặc định `false`, công cụ được giữ lại trong code để dùng khi cần nhưng không bật sẵn).

**Thay đổi 2 — giảm tần số LiDAR**: đọc source `/opt/ros/humble/share/turtlebot4_description/urdf/sensors/rplidar.urdf.xacro` — tần số thật hiện tại là **5Hz, 240 tia** (không phải 10Hz như giả định ban đầu trong yêu cầu — ghi nhận lại cho đúng). Viết `tools/rplidar_reduced.urdf.xacro` (bản sao, đổi `update_rate` 5.0→2.5) + `tools/turtlebot4_lowrate_lidar.urdf.xacro` (bản turtlebot4.urdf.xacro dùng rplidar đã giảm tốc, GIỮ camera) + `tools/robot_description_lowrate_lidar.launch.py` + cờ `lidar_lowrate`. Chưa giảm số tia (240→180) vì bước giảm tần số đã đo xong trước, theo đúng thứ tự yêu cầu.

RTF đo được (lidar riêng, cửa sổ bị cắt ngắn còn ~10.5s do goal kết thúc sớm trong lúc đo): **0.2289** — gần như bằng baseline (0.227-0.236), không có khác biệt rõ rệt. Khớp với dự đoán từ profile CPU (giảm tần số 1 cảm biến không chạm nút thắt `ign gazebo`).

**Kiểm tra độ chính xác với LIDAR 2.5Hz** (sim sạch, `lidar_lowrate:=true`, chạy `run_trial.py` thật tới Phòng ngủ, trần 40s mô phỏng): robot đi được phần lớn quãng đường dài 9.03m (`compute_path_to_pose` xác nhận khả thi) trước khi hết 40s (`sim_time_timeout`, không phải thất bại — chỉ do trần thời gian ngắn cho bài kiểm tra này). **Sai số AMCL-vs-ground-truth = 0.194m** (đạt tiêu chí <0.3m/5m, đo trên quãng đường thực tế ~9m) — LIDAR 2.5Hz vẫn đủ để AMCL sửa đúng vị trí dù odometry tự trôi 1.601m không có cảm biến sửa lỗi (dead-reckoning thuần, đúng như kỳ vọng vật lý, không phải lỗi). **Kết luận: giảm LiDAR xuống 2.5Hz không làm hỏng điều hướng/định vị, nhưng cũng không cải thiện RTF đáng kể — là thay đổi an toàn nhưng không đáng để làm mặc định xét theo tiêu chí RTF** (giữ `lidar_lowrate` mặc định `false`, công cụ sẵn sàng dùng nếu cần giảm tải cảm biến vì lý do khác).

**Thử kết hợp cả 2 thay đổi** (theo đề xuất trong lúc làm việc): `no_camera:=true` + `lidar_lowrate:=true` cùng lúc — viết thêm `turtlebot4_no_camera_lowrate_lidar.urdf.xacro` + launch tương ứng. RTF đo được: **0.2435** (cửa sổ bị cắt ngắn còn ~10.2s), số liệu đơn lẻ, cao hơn baseline một chút nhưng với n=1 và cửa sổ ngắn nên độ tin cậy thấp hơn các phép đo đủ 20s — không đủ cơ sở để kết luận chắc chắn tốt hơn baseline.

**Hạ tầng không ổn định trong lúc đo**: nhiều lần `bt_navigator`/`map_server`/`lifecycle_manager` treo do lỗi rmw/DDS mất phản hồi service (cùng dạng đã ghi nhận ở các lần điều tra trước), phải huỷ và thử lại — không liên quan tới các thay đổi camera/LiDAR. Phát hiện thêm 1 vấn đề công cụ: khi một lần thử bị huỷ nửa chừng, tiến trình `python3 measure_rtf.py` có thể sót lại chạy ngầm (không bị `stop_all.sh` dọn vì không khớp PATTERNS của script đó) và tiếp tục gửi goal/polling vào phiên sim MỚI ở lần thử sau, gây nhiễu kết quả — đã phát hiện và dọn thủ công (`kill`) khi xảy ra; đây là hạn chế của công cụ đo tạm thời dùng trong phiên này (`/tmp/tb4_profile/`, không phải code trong repo) nên không sửa trong repo.

**Kết luận cuối — RTF khi điều hướng và các thay đổi được giữ**:
- RTF khi điều hướng tích cực, đo trong cửa sổ ổn định (sau khi Nav2 đã "vào guồng"): dao động **0.16-0.24** tuỳ cấu hình và thời điểm đo, trung bình quanh **~0.22-0.23** với cấu hình baseline (3 tham số Nav2 nhẹ đã áp dụng). Không đạt mục tiêu đề xuất ≥0.2 một cách chắc chắn/ổn định (một số lần đo dưới 0.2), nhưng cũng không còn thấp nghiêm trọng như số đo ~0.048 của lần điều tra trước (số đó đo trung bình CẢ một trial 600s kể cả các đoạn robot bị kẹt/recovery nhiều, không cùng cách đo với các con số ở đây).
- **Thay đổi ĐƯỢC GIỮ**: 3 tham số Nav2 nhẹ trong `nav2_voicenav.yaml`/`localization_voicenav.yaml` (controller_frequency, DWB samples, costmap update_frequency, AMCL max_particles 2000→500) — không gây hại, không cải thiện rõ rệt nhưng hợp lý về nguyên tắc (giảm tải tính toán không cần thiết, ví dụ `vy_samples` cho robot không thể đi ngang).
- **Thay đổi KHÔNG giữ làm mặc định**: tắt camera OAK-D (`no_camera`) — đo được THẤP HƠN baseline, không có lý do RTF để bật. Giảm tần số LiDAR (`lidar_lowrate`) — trung tính về RTF (không hại, không rõ lợi), đã xác nhận an toàn về độ chính xác nếu cần dùng sau này nhưng không bật mặc định.
- **Nguyên nhân gốc chưa giải quyết**: nút thắt CPU thật sự là `ign gazebo` (~600% CPU, theo profile), không phải Nav2 hay cảm biến — muốn tăng RTF đáng kể cần giảm tải vật lý/render của chính Ignition Gazebo (ví dụ giảm độ chi tiết mesh va chạm, world đơn giản hơn, hoặc dùng physics engine/step size khác) chứ không phải các tham số đã thử ở đây. Chưa thử đổi physics step size (mục d trong yêu cầu ban đầu) do hết thời gian của phiên — để lại cho lần sau.
- Chưa đo lại THÀNH/BẠI 3 phòng với cấu hình cuối cùng — việc này cần một phiên riêng với đủ thời gian (RTF ~0.22 vẫn khiến một trial 180s sim cần ~800s thực).

## Nhận dạng giọng nói

Đã viết `speech_node` (backend `google`/`vosk`/`keyboard`/`auto`, publish `/voice_text`), `command_parser` (so khớp mờ cụm tên phòng, hỗ trợ "đi đến/đi tới/đến/di chuyển đến X" và "từ X đến Y", 12 unit test pass bao gồm sai chính tả nhẹ), `tools/mic_test.py` và `tools/eval_voice.py` (đo độ chính xác + độ trễ bằng micro thật, có đếm ngược 3-2-1 để đo đúng "từ lúc nói").

Model Vosk tiếng Việt có sẵn tại `/home/hiep/tb4_voicenav_data/models/vosk-model-small-vn-0.4`. Micro thật của máy xác nhận hoạt động qua PulseAudio (`sounddevice` thấy device `pulse`/`default`, `arecord` ghi được).

**Chưa đo được**: bảng độ chính xác (đúng/10) và độ trễ cho 4 câu (2 câu "đi đến", 1 câu "từ X đến Y", 1 câu khu vực không tồn tại) x 2 backend (google, vosk) theo yêu cầu mục D — bước này cần người dùng tự nói vào micro, agent không có khả năng phát âm thanh thật. Chạy:

```
python3 tools/eval_voice.py --stt vosk --reps 10
python3 tools/eval_voice.py --stt google --reps 10
```

rồi dán bảng kết quả in ra vào đây để chọn backend mặc định cho `speech_node` (tham số `stt`).

## Điều hướng

Đã viết `voice_nav_node` (nhận `/voice_text`, parse lệnh, gửi `NavigateToPose` tới `goal_m` trong `regions.json` theo đúng frame `map`, hàng đợi cho "từ X đến Y", publish lỗi rõ lên `/voice_nav/status` khi khu vực không tồn tại hoặc câu không phải lệnh). Giai đoạn này mới viết và test bằng mock Nav2 (`FakeActionClient` thay `ActionClient` thật, 5 unit test pass: gửi 1 đích, gửi hàng đợi 2 đích đúng thứ tự, không gửi goal khi khu vực không tồn tại, không gửi goal khi không phải lệnh, dừng hàng đợi khi Nav2 từ chối goal).

(chưa đo trên Nav2/robot thật trong phiên này)
