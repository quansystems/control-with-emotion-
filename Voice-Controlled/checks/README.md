# Kiểm chứng hiện tại: tiếng Anh và đích gần tâm A/B

Kết quả mới nhất là `english_center_goals/report.json`: Whisper small
(language=en) nhận `Go from Area A, via Area B, via Area C, then to Area D.`
Nav2/Gazebo thật đi đủ A → B → C → D và trả SUCCEEDED với các đích A/B mới.
Mặt nạ vùng giữ nguyên; chỉ đích A/B đổi sang ô gần tâm an toàn.

Chạy lại:

```bash
export OMP_NUM_THREADS=4
export MKL_NUM_THREADS=4
ROS_DOMAIN_ID=93 IGN_PARTITION=turtle_english_center_check LIBGL_ALWAYS_SOFTWARE=1 \
  .venv-voice/bin/python src/Turtlebot4/Voice-Controlled/smoke_test_voice.py \
  --audio-file src/Turtlebot4/Voice-Controlled/checks/english_center_goals/navigation_command.mp3 \
  --audio-source synthetic --language en --model small --expected-route A B C D \
  --output src/Turtlebot4/Voice-Controlled/checks/english_center_goals
```

Thư mục `latest/` dưới đây là phiên tiếng Việt cũ, trước khi đổi tâm A/B.
Dùng `--language vi` nếu chạy lại với file âm thanh tiếng Việt.

# Kiểm chứng luồng điều hướng giọng nói

Kết quả cuối của lượt kiểm tra nằm trong **latest/report.json**:
âm thanh tiếng Việt tổng hợp → node Whisper small → ROS → Nav2/Gazebo →
A → B → C → D → SUCCEEDED. `latest/logs/` là log của chính các tiến trình đó.

File âm thanh đã kiểm chứng là `latest/navigation_paused.mp3`; câu đọc có
khoảng ngắt giữa tên các vùng. Transcript và danh sách đích thực tế có trong
report. Các file `preflight*.json`, `navigation_command.mp3` và kết quả
preflight khác giữ lại các thử nghiệm chẩn đoán ASR trước lượt chạy cuối,
có thể chứa nhận dạng không đạt; chúng không phải kết quả cuối của robot.

Chạy lại bằng môi trường .venv-voice, source ROS/workspace, rồi:

```bash
export OMP_NUM_THREADS=4
export MKL_NUM_THREADS=4
ROS_DOMAIN_ID=92 IGN_PARTITION=turtle_voice_full_check LIBGL_ALWAYS_SOFTWARE=1 \
  .venv-voice/bin/python src/Turtlebot4/Voice-Controlled/smoke_test_voice.py \
  --audio-file src/Turtlebot4/Voice-Controlled/checks/latest/navigation_paused.mp3 \
  --audio-source synthetic --language vi --model small --expected-route A B C D
```

Script chạy simulation riêng, không thu microphone và tự dừng các tiến trình
của nó. Kết quả kiểm tra file âm thanh không bảo đảm nhận đúng mọi giọng nói
hoặc chất lượng microphone thực tế.
