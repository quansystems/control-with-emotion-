# Điều khiển TurtleBot4 bằng giọng nói tiếng Anh

Bộ vùng hiện tại do người dùng vẽ là **A, B, C, D**. Tất cả node đọc tên, ID,
aliases và đích từ `Map Partitioning/generated/regions.yaml`; `labels.npy`
chứa mặt nạ vùng dùng để hiển thị. Các ví dụ Phòng học/Phòng thí nghiệm trong
video cũ thuộc bản phân vùng trước, không phải tên vùng hiện tại.

| ID | Vị trí trên bản đồ hiện tại | Đích x, y (m) | Cách đọc ID |
| --- | --- | --- | --- |
| A | Dưới bên trái, gần tâm vùng | -1.855, -1.145 | ay |
| B | Dưới bên phải, gần tâm an toàn | 1.895, -0.895 | bee |
| C | Phía trên | -0.955, 1.655 | see |
| D | Ở giữa | 0.045, -0.495 | dee |

Đích A/B được chọn gần tâm đa giác nhất trong các ô trống đạt clearance
0,30 m. B có vật cản gần tâm nên đích được dịch tới ô an toàn gần nhất.
Đích C/D được giữ nguyên. Thuật toán và giao diện dùng cùng goal_policy
để việc lưu lại tên/vùng không đưa đích A/B trở về sát biên.

Luồng thực thi: microphone → Whisper tiếng Anh local → `/voice_command` → parser
catalog → Nav2 planner → lần lượt `NavigateToPose`. Robot luôn ghé cả vùng
xuất phát được nói trong câu. Chỉ chuyển đích sau khi đích trước thành công.

## 1. Chuẩn bị một lần

```bash
cd ~/turtle
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select robot_node
source install/setup.bash
```

Môi trường `.venv-voice` và model Whisper `small` đã được chuẩn bị trên máy.
Nếu cần cài mới:

```bash
sudo apt install python3-venv libportaudio2 ffmpeg python3-pyqt5
python3 -m venv --system-site-packages .venv-voice
touch .venv-voice/COLCON_IGNORE
.venv-voice/bin/python -m pip install --upgrade pip wheel 'setuptools>=65,<80'
.venv-voice/bin/python -m pip install 'torch==2.5.1' --index-url https://download.pytorch.org/whl/cpu
.venv-voice/bin/python -m pip install -r src/Turtlebot4/robot_node/requirements-voice.txt
```

pip cũ có thể báo metadata Whisper `UNKNOWN`; dùng lệnh nâng cấp ở trên.
`coverage` mới trong requirements xử lý xung đột Numba với bản Ubuntu cũ.
Whisper có dependency Triton trên Linux x86_64 dù đang suy luận bằng CPU.

## 2. Mở năm terminal

Trong **mỗi terminal**, chạy:

```bash
cd ~/turtle
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=69
export IGN_PARTITION=turtle_small_room
export TURTLE_REGIONS_FILE="$PWD/src/Turtlebot4/Map Partitioning/generated/regions.yaml"
```

Terminal 1 — Gazebo và robot tại (0,0,0):

```bash
ros2 launch robot_sim sim.launch.py world:=small_room enable_camera:=false rviz:=false
```

Terminal 2 — bản đồ, AMCL và RViz có các display semantic:

```bash
ros2 launch robot_navigation localization.launch.py \
  map:="$PWD/src/Turtlebot4/robot_navigation/maps/my_map.yaml" \
  rviz:=true \
  rviz_config:="$PWD/src/Turtlebot4/Voice-Controlled/voice_navigation.rviz" \
  set_initial_pose:=true initial_pose_x:=0.0 initial_pose_y:=0.0 initial_pose_yaw:=0.0
```

Initial pose trên dành cho spawn mặc định. Nếu spawn khác, dùng 2D Pose
Estimate trong RViz để LiDAR khớp tường. Fixed Frame phải là `map`.

Terminal 3 — Nav2:

```bash
ros2 launch robot_navigation navigation.launch.py rviz:=false
```

Terminal 4 — node điều hướng giọng nói:

```bash
ros2 launch robot_node voice_navigation.launch.py \
  regions_file:="$TURTLE_REGIONS_FILE" rviz:=false
```

Đợi Nav2 active và node báo READY. READY xác nhận map khớp; planner vẫn cần
Nav2 và TF hoạt động. Dừng teleop, gesture behavior và waypoint navigator
trước khi chạy điều hướng giọng nói để chỉ có một nguồn điều khiển.

Terminal 5 — microphone và Whisper:

```bash
export OMP_NUM_THREADS=4
export MKL_NUM_THREADS=4
.venv-voice/bin/python -m robot_node.speech_recognition --ros-args \
  -p model:=small -p language:=en -p record_seconds:=10.0 -p input_device:=-1 \
  -p regions_file:="$TURTLE_REGIONS_FILE"
```

Nhấn Enter ở lời nhắc rồi đọc trong 10 giây. Đọc tiếng Anh, rõ ID và ngắt nhẹ giữa các
vùng khi nói câu dài. Xem RECOGNIZED và trạng thái điều hướng trước lệnh mới.
Whisper small là mặc định; base vẫn chọn được nhưng các lượt thử có lỗi
nhận dạng chữ cái/câu dài. Model small có sẵn trong cache; máy mới cần tải
model lần đầu. Whisper mặc định dùng tiếng Anh, CPU, beam search 5. Có thể chọn lại tiếng
Việt bằng language:=vi; parser vẫn hỗ trợ các câu cũ.

Nếu microphone mặc định chưa đúng, liệt kê thiết bị rồi thay input_device:

```bash
.venv-voice/bin/python -c 'import sounddevice as sd; print(sd.query_devices())'
```

Âm lượng RMS quá nhỏ hoặc đoạn nhận dạng không có tiếng nói sẽ không gửi lệnh.

## 3. English commands

| Câu nói tiếng Anh | Kết quả |
| --- | --- |
| Go to area A. | A |
| Move to area B. | B |
| Navigate to area C. | C |
| Go to area D. | D |
| Go from area A to area B. | A → B |
| Go from area A via area D then to area B. | A → D → B |
| Go from area A to area B via area D. | A → D → B |
| Go from area A, via area B, via area C, then to area D. | A → B → C → D |
| Go to an unknown area. | INVALID_COMMAND; không điều hướng |
| Stop. / Cancel navigation. | Yêu cầu hủy hành trình hiện tại |

Có thể dùng area, region, zone hoặc room trước ID. Parser hiểu cả ID A/B/C/D
và cách viết âm đọc ay/bee/see/dee. Vẫn hỗ trợ tên/aliases do người dùng lưu.
Lệnh chứa vùng lạ hoặc thiếu đích bị từ chối toàn bộ, không đoán đích.
Từ chỉ thứ tự via/through/then/to phải đi cùng các tên vùng hợp lệ.
Lệnh mới khi đang chạy báo BUSY; chờ hoàn tất hoặc hủy trước. Hủy bằng giọng
nói cũng cần thời gian thu/nhận dạng.

## 4. Chỉnh vùng, xem trạng thái và kiểm tra

Mở giao diện [Map Partitioning](<../Map Partitioning/README.md>), vẽ/đặt tên
rồi nhấn Lưu vùng. Node điều hướng đọc lại file trước hành trình mới, xóa
marker cũ và hiển thị vùng mới; không thay các đích giữa một hành trình đang
chạy. Whisper đọc lại file trước mỗi lượt, tự tạo gợi ý từ vựng. Tham số
initial_prompt mặc định rỗng để dùng gợi ý tự động; chỉ đặt khi cần tự tùy biến.

Trong terminal phụ cùng môi trường:

```bash
ros2 topic echo /voice_navigation/status
# Hoặc kiểm tra Nav2:
ros2 lifecycle get /bt_navigator
```

Kiểm tra điều hướng độc lập microphone:

```bash
ros2 topic pub --once /voice_command std_msgs/msg/String \
  "data: 'Go from area A via area B via area C then to area D'"
```

Các trạng thái gồm NOT_READY, INVALID_COMMAND, PLANNING, NAVIGATING, SUCCEEDED,
FAILED, REJECTED, STOPPING, STOPPED, BUSY, CONFIG_ERROR và REGIONS_UPDATED.
Timeout planner mặc định 30 giây thực, mỗi đích 180 giây thực. Khi lỗi/hủy,
node dừng chuỗi và đợi Nav2 xác nhận kết thúc. Ctrl+C yêu cầu hủy trước khi thoát.

RViz có các topic /semantic_regions (nhãn, biên, đích),
/voice_navigation/route (thứ tự), /voice_navigation/planned_path (kế hoạch
đầu toàn tuyến) và /plan (đường hiện tại Nav2 lập lại trong khi di chuyển).
Đường toàn tuyến là snapshot; Nav2 tiếp tục xử lý vật cản bằng costmap/controller.

## 5. Chạy kiểm tra và công cụ video

```bash
QT_QPA_PLATFORM=offscreen ROS_DOMAIN_ID=87 /usr/bin/python3 -m pytest src/Turtlebot4/robot_node/test -q
```

Kiểm tra một lượt âm thanh → node Whisper thật → Nav2/Gazebo thật trong domain
riêng (script tự khởi động/dừng các tiến trình của nó):

```bash
export OMP_NUM_THREADS=4
export MKL_NUM_THREADS=4
ROS_DOMAIN_ID=92 IGN_PARTITION=turtle_voice_full_check LIBGL_ALWAYS_SOFTWARE=1 \
  .venv-voice/bin/python src/Turtlebot4/Voice-Controlled/smoke_test_voice.py \
  --language en --model small --expected-route A B C D \
  --output src/Turtlebot4/Voice-Controlled/checks/english_center_goals
```

Báo cáo và log: `checks/english_center_goals/report.json`, `checks/english_center_goals/logs/`. Lượt này dùng
file âm thanh tiếng Anh tổng hợp, không phải bản thu microphone của người dùng.

`record_demo.py` và `record_microphone_demo.py` nhận `--regions-file`, `--model`, `--language en|vi`,
tự sinh tình huống theo catalog hiện tại. Mặc định lưu trong demo_current và
demo_microphone_current; các video/log cũ trong demo và demo_microphone được
lưu nguyên như bằng chứng của phiên trước. Giọng mẫu được lưu theo hash của
câu nói để không dùng nhầm âm thanh cũ sau khi đổi vùng. Công cụ giọng mẫu
cần gTTS để tạo file mới; phần microphone không cần gửi âm thanh lên mạng.
