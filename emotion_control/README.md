# Điều khiển TurtleBot4 bằng biểu cảm khuôn mặt

Package ROS 2 Humble mới, độc lập với các node giọng nói/cử chỉ có sẵn.
Camera USB/laptop → OpenCV → DeepFace → lọc kết quả → `geometry_msgs/Twist`
trên `/cmd_vel` (20 Hz). Camera và suy luận chạy ở hai luồng riêng để mô hình
chậm không chặn bộ kiểm tra dừng hoặc cửa sổ camera.

| Biểu cảm | Hành động mặc định |
| --- | --- |
| Happy | Cong trái: tiến 0,10 m/s, quay +0,50 rad/s trong 3 s |
| Sad | Cong phải: tiến 0,10 m/s, quay −0,50 rad/s trong 3 s |
| Neutral | Dừng ngay |
| Angry | Lùi 0,10 m/s trong 1 s |
| Surprise | Xoay trái tại chỗ, dừng khi `/odom` tích lũy gần 2π rad |
| Fear | Tiến 0,10 m/s trong 1 s |

## Cài đặt và build

Chạy từ workspace `~/turtle`. Tạo môi trường riêng để tránh ảnh hưởng các
mô hình khác. Dùng Python hệ thống 3.10 để tương thích ROS Humble:

```bash
cd ~/turtle
source /opt/ros/humble/setup.bash
/usr/bin/python3 -m venv --system-site-packages .venv-emotion
touch .venv-emotion/COLCON_IGNORE
source .venv-emotion/bin/activate
python -m pip install --upgrade pip --timeout 300
python -m pip install --timeout 300 --retries 10 -r src/Turtlebot4/emotion_control/requirements.txt
python -c "from deepface import DeepFace; DeepFace.build_model(model_name='Emotion', task='facial_attribute')"
colcon build --symlink-install --packages-select emotion_control
source install/setup.bash
```

DeepFace tải trọng số Emotion lần đầu (cần Internet), lưu trong `~/.deepface`.
Thư viện dùng API `analyze(actions=['emotion'], enforce_detection=True)`;
tham khảo [DeepFace chính thức](https://github.com/serengil/deepface).
Nếu thiếu `venv`, cài `python3-venv` của Ubuntu trước. Không cài bản OpenCV
`headless` vì bài yêu cầu cửa sổ hiển thị.

## Chạy

Nếu dùng mô phỏng, mở terminal riêng, source ROS/workspace rồi chạy:

```bash
export ROS_DOMAIN_ID=69
export IGN_PARTITION=turtle_small_room
ros2 launch robot_sim sim.launch.py world:=small_room
```

Node dùng webcam của máy tính; không cần bật camera trên robot mô phỏng.
Dừng teleop, Nav2 hoặc các node khác đang phát `/cmd_vel` trước khi chạy.
Với robot thật, dùng cùng `ROS_DOMAIN_ID` và topic odometry của robot.

Trong terminal chạy nhận diện, dùng trực tiếp Python trong venv để bảo đảm
DeepFace được tìm thấy (entry point `ros2 run` có thể dùng Python hệ thống):

```bash
cd ~/turtle
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=69
export IGN_PARTITION=turtle_small_room
.venv-emotion/bin/python -m emotion_control.node --ros-args \
  --params-file src/Turtlebot4/emotion_control/config/emotion.yaml
```

Nếu DeepFace đã được cài vào Python dùng để build, có thể chạy:

```bash
ros2 launch emotion_control emotion_control.launch.py camera_index:=0 show_image:=true
```

Đổi camera bằng `-p camera_index:=2`. Máy không có màn hình dùng
`-p show_image:=false`. Cấu hình tốc độ, thời gian, độ tin cậy và tên topic trong
`config/emotion.yaml`; ví dụ `-p odom_topic:=/robot/odom`.

Cửa sổ hiển thị camera trực tiếp, khung khuôn mặt, biểu cảm, điểm số và vận tốc.
Nhấn **Q**, **Esc**, đóng cửa sổ hoặc **Ctrl+C** để dừng và thoát.
Topic `/detected_emotion` (`std_msgs/String`) phát nhãn thô hoặc `unknown`.

```bash
ros2 topic echo /detected_emotion
ros2 topic echo /cmd_vel
ros2 topic hz /odom
```

## Quy tắc xử lý

- Chọn khuôn mặt lớn nhất khi có nhiều người; ảnh hiển thị lật gương mặc định.
- Cần 3 kết quả liên tiếp khác frame, điểm cao nhất ≥ 0,60 mới kích hoạt.
- Một biểu cảm giữ nguyên chỉ chạy một hành động. Đổi sang neutral đủ 3 kết quả
  rồi biểu cảm lại để thực hiện lần nữa. Thay đổi biểu cảm sẽ dừng hành động cũ
  trong thời gian xác nhận biểu cảm mới.
- Không thấy mặt, điểm thấp hoặc `disgust` (ngoài đề bài) sẽ dừng.
  Không chọn nhãn hạng hai để thay thế `disgust`.
- Dữ liệu nhận diện được tính tuổi từ lúc chụp; quá 2 s sẽ dừng. Nếu máy suy luận
  chậm hơn mức này, tăng `detection_timeout` một cách phù hợp. Mô hình đang tải,
  lỗi thư viện, lỗi camera hoặc suy luận bị treo đều không cho phép tiếp tục chạy
  quá thời hạn dữ liệu.
- Surprise cần `/odom` cập nhật trong 0,5 s gần nhất, cộng góc quay có dấu qua
  mốc ±π; dừng ở khoảng 2π (sai số góc 0,03 rad cộng độ trễ điều khiển), hoặc sau
  tối đa 20 s. Mất odometry sẽ hủy hành động, không tự tiếp tục khi có lại.
  Cần đổi biểu cảm rồi thử lại. Độ chính xác vòng xoay phụ thuộc odometry.
- Giới hạn chuyển động không có chức năng tránh vật cản. Bộ điều khiển đế robot
  cần có timeout `/cmd_vel` để dừng nếu tiến trình bị kill hoặc máy mất kết nối.

## Kiểm tra

```bash
cd ~/turtle
source /opt/ros/humble/setup.bash
colcon test --packages-select emotion_control --event-handlers console_direct+
colcon test-result --verbose
```

Kiểm thử logic không cần camera, tải mô hình hoặc robot: ánh xạ đủ 6 biểu cảm,
lọc frame trùng/điểm thấp, timeout, kích hoạt lại, vòng xoay và mất odometry.
Để nghiệm thu camera thực, chạy node, lần lượt thể hiện 6 biểu cảm, kiểm tra nhãn
và hướng di chuyển; che camera để kiểm tra dừng. Không tự động mở camera hoặc
phát lệnh lên robot thật trong quá trình kiểm thử mã.
