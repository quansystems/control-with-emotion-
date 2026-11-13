> Video trong thư mục này ghi bộ vùng cũ A/B/C. Mã ghi demo hiện tại đọc
> catalog đang lưu (A/B/C/D) và mặc định ghi vào `../demo_current/`, dùng
> Whisper small. Các mốc và transcript bên dưới mô tả video lịch sử này.

# Video demo điều hướng giọng nói

Bản đã ghi: **2 phút 01,4 giây**, 1600×900, 15 fps, H.264 + AAC,
khoảng 1,4 MB. Đã kiểm tra giải mã toàn bộ video, bốn đoạn tiếng nói, ba
hành trình thành công và một lệnh khu vực lạ bị từ chối.

| Mốc video | Nội dung |
| --- | --- |
| 00:03 | Âm thanh: đến phòng học → A |
| 00:30 | Âm thanh: đến phòng thí nghiệm → B |
| 00:54 | Âm thanh: phòng học → hành lang → phòng thí nghiệm |
| 01:45 | Âm thanh: đến phòng thư viện (không tồn tại) |

Mở `turtlebot4_voice_demo.mp4` để xem toàn bộ luồng:

1. Nghe file âm thanh tiếng Việt (giọng tổng hợp).
2. Whisper `base` trên CPU nhận dạng âm thanh thành văn bản.
3. Parser xác định đích theo catalog thật của `my_map`.
4. Nav2 điều khiển robot trong Gazebo Fortress `small_room`.
5. Giao diện bản đồ hiển thị pose từ TF `map → base_link`, quỹ đạo đã đi,
   đường dự kiến và kế hoạch Nav2 cập nhật trong lúc di chuyển.

Video gồm ba lệnh hợp lệ: đến phòng học (A), đến phòng thí nghiệm (B),
đi từ phòng học qua hành lang đến phòng thí nghiệm (A → C → B); sau đó thử
phòng thư viện không tồn tại. Văn bản nhận dạng được hiển thị nguyên kết quả
Whisper, kể cả tên vùng lạ nghe sai. Không thay văn bản nhận dạng bằng câu mẫu.

Nguồn âm thanh là giọng đọc tổng hợp tiếng Việt tạo bằng gTTS; video không
phải bản thu giọng người dùng qua microphone. Whisper, parser, Nav2 và
chuyển động Gazebo chạy thật. Gợi ý từ vựng cho Whisper chỉ gồm tên ba vùng:
`Tên khu vực: phòng học, phòng thí nghiệm, hành lang.`

Các file kèm theo:

- `manifest.json`: thời lượng, thông tin chạy, transcript, đích và kết quả.
- `events.jsonl`: timestamp từng sự kiện ROS và các pose robot đã nhận.
- `assets/*.wav`: chính xác các file âm thanh đầu vào.
- `logs/`: log các node thật trong lượt ghi.
- `poster.png`: ảnh xem trước video.

Chạy lại để ghi video mới, từ workspace đã build/source:

```bash
cd ~/turtle
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=89
export IGN_PARTITION=turtle_voice_video_demo
export LIBGL_ALWAYS_SOFTWARE=1
export OMP_NUM_THREADS=4
export MKL_NUM_THREADS=4
.venv-voice/bin/python src/Turtlebot4/Voice-Controlled/record_demo.py
```

Lệnh ghi đè các file demo đã tạo và tự dừng các tiến trình mô phỏng do nó
khởi động. Cần ffmpeg, Pillow, tf2_ros và môi trường voice đã cài. File WAV
có sẵn để chạy lại, không cần gTTS khi ghi. Source của bộ ghi không publish
`/voice_command`: câu lệnh này được node `speech_recognition` publish từ
kết quả Whisper.
