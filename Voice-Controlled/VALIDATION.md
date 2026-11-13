# Hiện tại: đích gần tâm A/B và lệnh tiếng Anh

- A: (-1.855, -1.145) m, B: (1.895, -0.895) m. Chọn ô an toàn gần tâm
  đa giác nhất, đạt clearance 0,30 m. Tâm B gần vật cản nên điểm được chiếu
  tới ô an toàn gần nhất. Policy center được lưu và dùng cả trong giao diện.
- C/D và mặt nạ/đường biên các vùng được giữ nguyên. Bản sao trước khi đổi
  đích nằm trong `../Map Partitioning/backups/before_center_goals/`.
- Whisper mặc định language=en, small, beam search 5. Parser nhận go/move/
  navigate, area/region/zone/room, via/through/then, stop/cancel; vẫn giữ
  tùy chọn language=vi để dùng các lệnh tiếng Việt cũ.
- **230 tests đạt**; build thành công.
- Thử âm thanh tiếng Anh tổng hợp → Whisper → ROS → Nav2/Gazebo thật:
  `Go from Area A, via Area B, via Area C, then to Area D.`
  Parser/robot đi A → B → C → D và trả **SUCCEEDED**.
- Bằng chứng: `checks/english_center_goals/report.json` và logs cùng thư mục.
  Đây là kiểm tra bằng file âm thanh, không phải bản thu microphone người dùng.

Các mục sau ghi lại các phiên trước với đích/ngôn ngữ khác.

# Kiểm chứng hiện tại — 31/10/2026

- Bộ vùng hiện tại: A, B, C, D; giữ nguyên catalog và raster người dùng đã lưu.
- Build robot_node thành công; toàn bộ **200 tests đạt**.
- Lượt đầy đủ dùng file tiếng Việt tổng hợp, **Whisper small local** (beam search 5),
  node speech_recognition thật, ROS, Nav2 và Gazebo Fortress thật.
- Transcript: `Di chuyển từ khu vực A, qua khu vực B, qua khu vực C, rồi đến khu vực Đê.`
- Parser và chuỗi mục tiêu đều A → B → C → D; Nav2 trả **SUCCEEDED**.
- Tổng lượt kiểm tra khoảng 66,9 giây; script dừng các tiến trình do nó khởi động.
- Báo cáo: `checks/latest/report.json`; log: `checks/latest/logs/`.
- Chưa kiểm tra microphone với giọng người dùng trong lượt này. Dùng model small
  và đọc rõ, ngắt nhẹ giữa các vùng. Các lượt thử base hoặc âm thanh không có
  khoảng ngắt có lỗi nhận dạng, được lưu riêng trong các file preflight để đối chiếu.

Các mục bên dưới là lịch sử kiểm tra với bộ vùng/tên phòng cũ, không phải
cấu hình điều khiển hiện tại.

# Kết quả kiểm tra — 31/10/2026

- `colcon build --symlink-install --packages-select robot_node`: thành công.
- Toàn bộ `robot_node/test`: **172 passed** (gồm 34 kiểm tra mới cho semantic
  map và voice navigation).
- Catalog cài đặt qua ament đọc được ba vùng; launch arguments, executable
  phân vùng và file RViz cài đặt đều đọc được.
- Gazebo Fortress chạy headless, world `small_room`, camera tắt; localization
  AMCL tại (0,0,0), Nav2 thật và node semantic cùng domain riêng 88.
- Gửi text `Đi đến khu vực Z`: nhận `INVALID_COMMAND`, không tạo hành trình.
- Gửi text `Đi từ phòng A qua hành lang C rồi đến phòng B` từ pose spawn:
  `PLANNING A → C → B`, lần lượt `NAVIGATING 1/3 → A`, `2/3 → C`, `3/3 → B`,
  kết thúc `SUCCEEDED`. Controller Nav2 báo `Reached the goal!` đủ ba lần.
- Các tiến trình thử nghiệm đã được dừng sau kiểm tra.

Lượt kiểm tra tích hợp gửi văn bản vào `/voice_command`. Chưa kiểm tra
microphone, chất lượng Whisper với ba câu nói thật, hoặc vật cản động đưa
thêm trong lúc di chuyển. Các kiểm tra này không thay thế nghiệm thu trên
robot thật.

## Bổ sung: khắc phục cài đặt Whisper

Đã cài thành công vào `.venv-voice`: Whisper 20250625, PyTorch 2.5.1+cpu,
pip 26.2.2, setuptools 79.0.1 và coverage 7.16.2. Kiểm tra import Whisper,
sounddevice, ROS và speech node đạt; tokenizer tiếng Việt trả `vi`.
Hướng dẫn đã bổ sung nâng cấp pip/build tools để tránh metadata UNKNOWN;
requirements bổ sung coverage mới để tránh xung đột Numba với coverage 6.2
được kế thừa từ Ubuntu.

## Bổ sung: video luồng giọng nói hoàn chỉnh

Đã chạy suy luận Whisper base thực tế với bốn file WAV tiếng Việt tổng hợp,
gợi ý từ vựng ba tên vùng. Ba lệnh hợp lệ nhận đúng tên vùng và hoàn thành
Nav2/Gazebo: A, B, A → C → B. Lệnh phòng thư viện bị từ chối (Whisper trả
`phòng thư viễn`, parser bỏ dấu và xác định tên không có trong catalog).
Video `demo/turtlebot4_voice_demo.mp4` dài 121,4 giây; có 349 mẫu pose robot,
bốn transcript và bốn sự kiện âm thanh trong `demo/events.jsonl`. Đã kiểm
tra giải mã H.264/AAC và âm thanh ở cả bốn mốc. Chưa kiểm tra microphone
với giọng người dùng thực tế.
