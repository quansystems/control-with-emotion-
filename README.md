# NHẬN DIỆN BIỂU CẢM KHUÔN MẶT VÀ ĐIỀU KHIỂN TURTLEBOT

Chương trình ROS 2 Humble dùng DeepFace và OpenCV để nhận diện 6 biểu cảm, điều khiển TurtleBot4 qua `/cmd_vel`.

- [Hướng dẫn cài đặt và chạy](emotion_control/README.md)
- [Video và kết quả trên Google Drive](https://drive.google.com/drive/folders/1jWZfR07deVEz_qWt0VAmqhq7rrx8jRdR?usp=sharing)

## Mã nguồn

Package chính: `emotion_control/`. Các package `robot_description`, `robot_sim`, `robot_mapping`, `robot_navigation` và `robot_node` kế thừa workspace TurtleBot4 hiện có.

Kiểm tra mã: build thành công, 23 kiểm thử đạt. Nhận diện webcam chưa được xác nhận trong lần kiểm tra tự động; cần cài đủ dependency trước khi chạy.

---

# TurtleBot4 — ROS 2 Humble / Gazebo Fortress

Nhận diện 6 biểu cảm khuôn mặt và điều khiển quỹ đạo qua `/cmd_vel`:
[emotion_control](emotion_control/README.md).

Phân vùng bản đồ và điều hướng giọng nói: [Map Partitioning](<Map Partitioning/README.md>)
và [Voice-Controlled](Voice-Controlled/README.md). Bộ vùng hiện tại gồm A/B/C/D,
Whisper tiếng Anh (mặc định language=en), lệnh nhiều đích qua Nav2 và cấu hình hiển thị RViz.

Tài liệu báo cáo: [Cấu trúc, khởi tạo, node/topic và ánh xạ cử chỉ](BAO_CAO.md).
Bản text để dựng sơ đồ: [Đặc tả flowchart chi tiết](FLOWCHART.txt).

## 1. Cấu trúc chương trình

```text
Turtlebot4/
├── README.md
├── robot_description/
│   ├── urdf/          # Xacro robot, cảm biến và plugin Gazebo
│   ├── meshes/        # Mesh TurtleBot4 và iRobot Create 3 chính thức
│   ├── config/        # simulation.yaml
│   ├── launch/        # display.launch.py
│   └── rviz/
├── robot_sim/
│   ├── worlds/        # World small_room
│   ├── config/        # bridge_config.yaml, demo_layout.yaml
│   ├── launch/        # sim, gazebo, spawn, bridge
│   └── scripts/
├── robot_mapping/
│   ├── config/        # slam.yaml
│   ├── launch/        # mapping.launch.py, save_map.launch.py
│   └── rviz/
├── robot_navigation/
│   ├── maps/          # Bản đồ SLAM đã lưu: YAML và PGM
│   ├── config/        # nav2.yaml
│   ├── launch/        # localization.launch.py, navigation.launch.py
│   └── rviz/
└── robot_node/
    ├── robot_node/    # Node teleop và xử lý vận tốc
    ├── resource/
    └── test/
```

| Package | Chức năng |
| --- | --- |
| `robot_description` | Mô tả robot, khớp, frame, cảm biến và cấu hình mô phỏng. |
| `robot_sim` | Mở Gazebo, spawn robot và bridge các topic ROS–Gazebo. |
| `robot_mapping` | Chạy SLAM Toolbox và lưu bản đồ. |
| `robot_navigation` | Chạy localization và navigation bằng hai launch riêng. |
| `robot_node` | Điều khiển robot bằng bàn phím, gửi lệnh tới `/cmd_vel`. |

## 2. Clone và cài đặt

Môi trường: **Ubuntu 22.04, ROS 2 Humble, Gazebo Fortress**.
Các lệnh dưới đây giả định ROS 2 Humble đã được cài đặt.

Cài công cụ build và dependency:

```bash
sudo apt update
sudo apt install git python3-colcon-common-extensions python3-yaml \
  ros-humble-ros-gz ros-humble-xacro \
  ros-humble-robot-state-publisher ros-humble-joint-state-publisher-gui \
  ros-humble-rviz2 ros-humble-slam-toolbox \
  ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-teleop-twist-keyboard
```

```bash
mkdir -p ~/turtle
cd ~/turtle
git clone https://github.com/quansystems/fundamental_turtlebot4.git src/Turtlebot4
```

Build workspace (chạy lại sau khi sửa mã nguồn hoặc world):

```bash
cd ~/turtle
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 3. Khảo sát small_room với bản đồ đã quét my_map

Map server nhận **my_map.yaml**, file này tham chiếu ảnh **my_map.pgm** cùng thư
mục. Dùng map đã lưu thì không cần chạy mapping. World và map nằm tại
`src/Turtlebot4/robot_sim/worlds/small_room.sdf` và
`src/Turtlebot4/robot_navigation/maps/my_map.yaml`. Nếu chưa có map, làm theo mục 4 trước.

Mở 3 terminal; trong **mỗi terminal**, chạy:

```bash
cd ~/turtle
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=69
export IGN_PARTITION=turtle_small_room
```

Terminal 1 — Gazebo và robot, spawn tại (0, 0), yaw = 0:

```bash
ros2 launch robot_sim sim.launch.py world:=small_room enable_camera:=true
```

Terminal 2 — Map, AMCL và RViz; chỉ dùng pose dưới đây nếu spawn khớp (0, 0, 0) trong hệ map đã lưu. Nếu không, bỏ các tham số initial pose và dùng **2D Pose Estimate**:

```bash
ros2 launch robot_navigation localization.launch.py \
  map:=$PWD/src/Turtlebot4/robot_navigation/maps/my_map.yaml \
  set_initial_pose:=true initial_pose_x:=0.0 initial_pose_y:=0.0 initial_pose_yaw:=0.0
```

Terminal 3 — Navigation:

```bash
ros2 launch robot_navigation navigation.launch.py
```

Đợi Nav2 active, sau đó chọn **Nav2 Goal** trong RViz và đặt đích ở vùng
trống. RViz dùng Fixed Frame `map`. Nếu đổi vị trí spawn, đổi initial pose
cho tương ứng hoặc dùng **2D Pose Estimate**.

Kiểm tra mô phỏng ở terminal khác đã source môi trường như trên:

```bash
ros2 run robot_sim smoke_test.py --camera
ros2 run tf2_ros tf2_echo map base_link
```

Để điều khiển thủ công, dừng navigation rồi chạy
`ros2 run robot_node teleop_keyboard`. Dừng các launch bằng `Ctrl+C`.

## 4. Mapping lại và dùng map tự tạo

### Chuẩn bị terminal

Mở 4 terminal. Trong **mỗi terminal**, chạy:

```bash
cd ~/turtle
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=69
export IGN_PARTITION=turtle_small_room
```

Giữ terminal ở thư mục `~/turtle` để các đường dẫn `$PWD/src/Turtlebot4/...` đúng.

### Mở simulation — terminal 1

```bash
ros2 launch robot_sim sim.launch.py world:=small_room enable_camera:=true
```

### Bước 1 — Mapping

Terminal 2 mở SLAM Toolbox và RViz:

```bash
ros2 launch robot_mapping mapping.launch.py
```

Terminal 3 mở teleop:

```bash
ros2 run robot_node teleop_keyboard
```

Focus terminal teleop, dùng các phím chữ thường (tắt gõ tiếng việt):

| Phím | Chức năng |
| --- | --- |
| `w` / `s` | Tiến / lùi |
| `a` / `d` | Quay trái / phải |
| `Space` | Dừng |
| `Ctrl+C` | Dừng và thoát |

Terminal 4 lưu map **khi mapping vẫn đang chạy**:

```bash
ros2 launch robot_mapping save_map.launch.py \
  map_name:=$PWD/src/Turtlebot4/robot_navigation/maps/my_map
```

Sau khi lưu xong, `Ctrl+C` tắt **teleop và mapping**, giữ simulation.

### Bước 2 — Localization

Terminal 2:

```bash
ros2 launch robot_navigation localization.launch.py \
  map:=$PWD/src/Turtlebot4/robot_navigation/maps/my_map.yaml
```

Launch mở map server, AMCL và RViz. Trong RViz:

1. Đặt **Global Options → Fixed Frame** thành `map`; có thể gõ trực tiếp
   nếu danh sách chưa có frame này.
2. Chọn **2D Pose Estimate**, nhấp vị trí hiện tại của robot trên bản đồ
   và kéo mũi tên theo hướng robot.
3. Đợi scan khớp với vật cản trên map.

Terminal 4 kiểm tra TF:

```bash
ros2 run tf2_ros tf2_echo map base_link
```

Khi thấy pose được xuất, `Ctrl+C` dừng riêng lệnh kiểm tra.
**Giữ localization và RViz chạy khi sang bước 3.**

### Bước 3 — Navigation

Terminal 3, sau khi teleop đã tắt:

```bash
ros2 launch robot_navigation navigation.launch.py
```

Đợi Nav2 chuyển sang `active`. Trong cửa sổ RViz của localization, chọn
**Nav2 Goal**, nhấp đích ở vùng trống đã lập bản đồ và kéo mũi tên đặt hướng
cuối. Robot sẽ lập kế hoạch và di chuyển đến đích.

## 5. Tự động di chuyển qua các vùng đã lưu

Giữ simulation và localization chạy, đặt **2D Pose Estimate** để LiDAR khớp map.
Không chạy mapping hoặc teleop cùng lúc với điều hướng tự động.
Trong mỗi terminal mới, source ROS/workspace và đặt `ROS_DOMAIN_ID`,
`IGN_PARTITION` như mục 3.

Terminal navigation:

```bash
ros2 launch robot_navigation navigation.launch.py
```

Terminal điều khiển tự động:

```bash
ros2 run robot_node waypoint_navigator --ros-args -p use_sim_time:=true
```

Mặc định node đọc catalog `Map Partitioning/generated/regions.yaml`, lấy đích
các vùng theo thứ tự ID (hiện tại A → B → C → D), chờ pose AMCL và Nav2 active,
rồi gửi từng goal qua action `navigate_to_pose`. Chỉ sau khi goal thành công
mới gửi goal tiếp theo. Nếu bị từ chối, thất bại, hủy hoặc quá thời gian,
node dừng chuỗi và trả mã thoát khác 0. `Ctrl+C` yêu cầu hủy goal hiện tại.
Node không đặt lại initial pose. Không dùng chuột đặt Nav2 Goal sau khi node chạy.

Tọa độ `x`, `y` theo hệ `map`, đơn vị mét; `yaw` dùng radian.
Các đích được lấy từ bộ vùng người dùng đã lưu, với `yaw` trong catalog và
`z = 0` cho điều hướng phẳng. File `config/waypoints.yaml` là bản tọa độ tham
khảo hiện tại; truyền `goals_file` chỉ khi muốn dùng một tuyến cố định riêng.
Có thể dùng file khác hoặc thay thời gian tối đa mỗi goal (giây thực):

```bash
ros2 run robot_node waypoint_navigator --ros-args \
  -p use_sim_time:=true \
  -p goals_file:=/home/quan2905/turtle/src/Turtlebot4/robot_node/config/waypoints.yaml \
  -p goal_timeout_sec:=180.0
```

Chạy lại lệnh node sẽ bắt đầu lại từ Goal 1. Nếu môi trường thiếu thư viện,
cài `sudo apt install ros-humble-nav2-simple-commander python3-yaml`.

## 6. Nhận diện camera bằng YOLO

Node `object_detector` nhận `/camera/image`, chuyển sang OpenCV BGR bằng
`cv_bridge`, chạy YOLOv8n, vẽ bounding box/tên lớp/độ tin cậy, hiển thị cửa sổ
và publish ảnh lên `/detections/image`. Log terminal gồm tên lớp, độ tin cậy
và tọa độ bounding box theo pixel, mặc định cập nhật mỗi giây.
Node độc lập với điều hướng, có thể chạy song song `waypoint_navigator`.

Cài thư viện vào môi trường riêng (chỉ cần làm một lần; máy hiện tại đã cài):

```bash
cd /home/quan2905/turtle
python3 -m venv --system-site-packages .venv-yolo
touch .venv-yolo/COLCON_IGNORE
.venv-yolo/bin/python -m pip install \
  'torch==2.5.1+cpu' 'torchvision==0.20.1+cpu' \
  --index-url https://download.pytorch.org/whl/cpu
.venv-yolo/bin/python -m pip install -r src/Turtlebot4/robot_node/requirements-yolo.txt
```

Nếu máy khác thiếu venv/cv_bridge/OpenCV, cài bằng apt:
`sudo apt install python3-venv ros-humble-cv-bridge python3-opencv`.
Giữ NumPy dưới phiên bản 2 để dùng với `cv_bridge` của ROS 2 Humble.

Giữ Gazebo chạy với `enable_camera:=true`. Trong terminal mới:

```bash
cd /home/quan2905/turtle
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=69
export IGN_PARTITION=turtle_small_room

.venv-yolo/bin/python -m robot_node.object_detector
```

Dùng Python của venv trực tiếp để bảo đảm nạp đúng YOLO/PyTorch;
`ros2 run` có thể dùng Python hệ thống theo shebang lúc build.
Trọng số mặc định nằm ở `~/.cache/turtlebot4/yolo/yolov8n.pt`, tự tải lần đầu.
Nhấn `q`, `Esc` trong cửa sổ ảnh hoặc `Ctrl+C` ở terminal để dừng node.

Mặc định dùng CPU, tối đa 3 lần suy luận/giây, ngưỡng confidence 0.35,
kích thước xử lý 640. Nếu mô phỏng chậm, giảm tần suất và kích thước:

```bash
.venv-yolo/bin/python -m robot_node.object_detector --ros-args \
  -p max_fps:=2.0 -p image_size:=320 -p confidence:=0.35
```

Chạy không có cửa sổ OpenCV:

```bash
.venv-yolo/bin/python -m robot_node.object_detector --ros-args -p show_image:=false
```

Trong RViz chọn **Add → Image**, đặt topic `/detections/image`.
Các tham số khác: `image_topic`, `output_topic`, `model_file`, `device`,
`log_interval_sec`. Node không thay đổi ảnh camera gốc.
Vật thể dựng hình đơn giản có thể chưa được mô hình pretrained nhận diện;
cần kiểm tra góc nhìn, khoảng cách, ánh sáng hoặc dùng trọng số đã fine-tune.

API suy luận và vẽ kết quả: https://docs.ultralytics.com/modes/predict/.

Chai `yolo_bottle` dùng mesh/texture Water Bottle từ Gazebo Fuel (iche033,
CC BY 4.0), đã chuyển sang Collada cho Fortress; xem nguồn và các thay đổi
trong `robot_sim/worlds/assets/water_bottle/SOURCE.txt`.
Chai giữ tọa độ `(2.5, -0.5)` và collision ban đầu nên việc thay phần hiển thị
không yêu cầu quét lại map. Khởi động lại Gazebo để nạp mesh mới và định vị
lại robot trong RViz nếu đã khởi động lại simulation.
Thử nghiệm ảnh camera Gazebo ở khoảng cách 1.3 m, độ cao camera 0.48 m:
YOLOv8n nhận chai mới với confidence khoảng 0.86; chai hình học cũ không có
detection lớp bottle ngay cả tại ngưỡng 0.05. Đây là kết quả ở góc nhìn thử,
không bảo đảm nhận diện ở mọi vị trí của robot.

## 7. Nhận diện cử chỉ tay từ webcam USB

Node `gesture_recognition_node` đọc trực tiếp webcam bằng OpenCV, chuyển BGR
sang RGB và chạy MediaPipe Tasks Gesture Recognizer ở chế độ VIDEO với timestamp
đơn điệu. Không cần chạy Gazebo hoặc camera topic. Xử lý tối đa một bàn tay;
đưa một tay vào khung hình để tránh đổi tay trong lúc vẫy.
Danh sách yêu cầu gồm **7 hành động** (dù mô tả ban đầu ghi 6):

| Hành động | Giá trị `data` trên `/detected_gesture` |
| --- | --- |
| Vẫy tay | `Waving` |
| Giơ ngón trỏ chỉ lên | `Pointing_Up` |
| Giơ ngón trỏ chỉ xuống | `Pointing_Down` |
| Giơ 2 ngón trỏ và giữa hình chữ V | `Victory` |
| Giơ ngón cái lên | `thumb_up` |
| Xòe bàn tay | `Open_Palm` |
| Nắm tay | `Closed_Fist` |

Topic dùng `std_msgs/msg/String`, publish một nhãn mỗi khung hình đã xử lý.
`None` biểu thị không thấy tay, cử chỉ không hỗ trợ, điểm dưới ngưỡng hoặc lỗi
đọc ảnh; tránh giữ nhãn cũ khi tay đã ra khỏi khung hình. Nhãn được giữ ở dạng
ASCII cố định để các node khác dễ xử lý. Confidence và nguồn điểm xuất hiện
trên cửa sổ và trong log khi nhãn thay đổi. Tên tiếng Việt trên cửa sổ dùng
chữ không dấu vì font OpenCV Hershey không hỗ trợ tiếng Việt có dấu.

Model có sẵn không có lớp vẫy tay hoặc ngón trỏ chỉ xuống. Node dùng điểm khớp
để kiểm tra ngón trỏ duỗi, ba ngón còn lại gập và hướng lên/xuống theo ảnh.
Điểm của hai hướng chỉ là điểm hình học (`geometry rule`), không phải xác suất
đã hiệu chỉnh từ model. Vẫy tay được nhận diện theo hai cách: dịch tâm lòng bàn
tay ngang hoặc lắc cổ tay làm các đầu ngón nghiêng trái/phải. Cần đổi hướng
ít nhất hai lần trong 2 giây. Ngưỡng dịch ngang mặc định là 3.5% chiều rộng
ảnh; ngưỡng nghiêng được chuẩn hóa theo chiều dài lòng bàn tay để dùng được
khi tay ở xa camera. Luật kiểm tra ít nhất ba ngón duỗi hỗ trợ các khung hình
model mất nhãn Open_Palm hoặc giảm điểm trong lúc vẫy. Nhãn phân loại gián đoạn
ngắn được giữ lịch sử tối đa 0.45 giây, nhưng không phát nhãn Waving cũ khi
khung hình hiện tại chưa đủ điều kiện. Mất toàn bộ điểm khớp hoặc chuyển sang
một tư thế tĩnh khác có confidence cao sẽ xóa lịch sử.
Điểm cho vẫy tay lấy từ điểm tư thế bàn tay (model hoặc hình học), không phải
xác suất của một model vẫy tay. Bàn tay xòe đứng yên trả về `Open_Palm`.
Các tư thế còn lại dùng điểm phân loại của MediaPipe. Đây là luật bổ sung cần
kiểm tra thực tế theo góc tay, ánh sáng và tốc độ; chưa phải model động đã huấn luyện.

Cài một lần vào môi trường riêng, giữ NumPy dưới 2 để tương thích ROS Humble:

```bash
cd /home/quan2905/turtle
python3 -m venv --system-site-packages .venv-gesture
touch .venv-gesture/COLCON_IGNORE
.venv-gesture/bin/python -m pip install -r src/Turtlebot4/robot_node/requirements-gesture.txt
mkdir -p ~/.cache/turtlebot4/gesture
curl -fL --retry 2 -o ~/.cache/turtlebot4/gesture/gesture_recognizer.task \
  https://storage.googleapis.com/mediapipe-models/gesture_recognizer/gesture_recognizer/float16/1/gesture_recognizer.task
```

Build và chạy bằng Python của venv để nạp được MediaPipe:

```bash
cd /home/quan2905/turtle
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select robot_node
source install/setup.bash
.venv-gesture/bin/python -m robot_node.gesture_recognition_node
```

Executable ROS đã đăng ký: `ros2 run robot_node gesture_recognition_node`.
Lệnh này dùng Python theo shebang lúc build; nếu MediaPipe chỉ được cài trong
venv, dùng lệnh `.venv-gesture/bin/python -m ...` ở trên.
Trong terminal khác, source ROS/workspace và dùng cùng `ROS_DOMAIN_ID`:

```bash
source /opt/ros/humble/setup.bash
source /home/quan2905/turtle/install/setup.bash
ros2 topic echo /detected_gesture std_msgs/msg/String
```

Mặc định dùng `/dev/video0`. Đổi sang `/dev/video2`, điều chỉnh ngưỡng, hoặc chạy
không có cửa sổ:

Trên máy hiện tại, webcam USB có tên `USB Composite Device: USB Camer`
ở `/dev/video2`; dùng `-p camera_index:=2` khi chạy. Môi trường `.venv-gesture`
và model trong cache đã được chuẩn bị.

```bash
.venv-gesture/bin/python -m robot_node.gesture_recognition_node --ros-args \
  -p camera_index:=2 -p confidence_threshold:=0.7 -p max_fps:=10.0
.venv-gesture/bin/python -m robot_node.gesture_recognition_node --ros-args -p show_image:=false
```

Tham số khác: `model_path`, `frame_width` (640), `frame_height` (480),
`mirror_image` (true), `wave_window_sec` (2.0), `wave_min_motion` (0.035).
`wave_min_motion` điều chỉnh nhánh dịch ngang; nhánh lắc cổ tay dùng ngưỡng
riêng bằng 0.35 lần chiều dài lòng bàn tay. Khi thử, xòe tay hướng về camera
và lắc trái–phải–trái–phải trong khoảng 1–2 giây.
Tham số được đọc lúc khởi tạo; khởi động lại node sau khi thay đổi.
Nhấn `q`, `Esc`, đóng cửa sổ hoặc `Ctrl+C` để giải phóng camera/model và thoát.
Nếu không mở được webcam, kiểm tra `/dev/video*`, quyền thiết bị, chọn đúng
`camera_index` và tắt ứng dụng khác đang sử dụng webcam. Sau 10 lần đọc ảnh lỗi
liên tiếp node thoát với mã lỗi. Khi không có desktop display, node tắt cửa sổ
và vẫn publish kết quả.

Tài liệu API/model chính thức:
[MediaPipe Gesture Recognizer Python](https://ai.google.dev/edge/mediapipe/solutions/vision/gesture_recognizer/python),
[các lớp cử chỉ có sẵn](https://ai.google.dev/edge/mediapipe/solutions/vision/gesture_recognizer).

## 8. Điều khiển hành vi TurtleBot 4 bằng cử chỉ

`robot_behavior_node` subscribe `/detected_gesture` (`std_msgs/msg/String`) và
publish `/cmd_vel` (`geometry_msgs/msg/Twist`) ở 20 Hz. Node đo tiến độ bằng
`/odom` và kiểm tra khoảng trống bằng topic `scan_topic` (cấu hình demo dùng
`/behavior_scan`). Hàng đợi dùng `deque(maxlen=1)`, chỉ chứa lệnh mức 1 mới
nhất; lệnh mức 1 mới thay thế lệnh đang chờ, không tự chạy.

| Mức | Nhãn nhận diện | Tác dụng |
| --- | --- | --- |
| 1 | `Pointing_Up` | Xếp lệnh tiến 4 chặng, mỗi chặng 0.35 m; nghỉ 1 giây giữa các chặng |
| 1 | `Pointing_Down` | Xếp lệnh lùi 4 chặng, mỗi chặng 0.35 m; nghỉ 1 giây giữa các chặng |
| 1 | `Victory` | Xếp lệnh chạy một vòng tròn bán kính 0.5 m |
| 1 | `Closed_Fist` | Xếp lệnh hủy hành vi, cần xác nhận |
| Xác nhận | `thumb_up` / `Thumb_Up` / `Thumbs_Up` | Lấy lệnh khỏi hàng đợi và thực hiện; hàng đợi trở thành rỗng |
| 2 | `Waving` / `Wave` | Tự quay trái/phải chào lại khi hàng đợi rỗng, không cần xác nhận |
| 0 | `Open_Palm` | Dừng ngay và xóa hàng đợi, kể cả khi robot chỉ đang chờ xác nhận |
| Không nhận diện | `None`, chuỗi rỗng, nhãn lạ | Không thay đổi hành vi hoặc hàng đợi |

Giữ cử chỉ mức 1, vẫy tay hoặc xác nhận ổn định ít nhất 0.25 giây và 3 bản tin.
Một tư thế giữ nguyên chỉ tạo một sự kiện để tránh kích hoạt liên tục; nhãn
không nhận diện không mở lại sự kiện đó. Xòe tay mức 0 có hiệu lực ngay từ bản
tin đầu, không qua bộ lọc này. Hàng đợi không tự hết hạn theo thời gian.

**Số mức càng nhỏ thì càng cấp bách: ưu tiên mức 0, rồi mức 1, rồi mức 2.** Khi robot đang
chào mức 2, một lệnh mức 1 được bộ lọc chấp nhận sẽ ngắt động tác chào ngay,
đưa lệnh vào hàng đợi và chờ `thumb_up`. Việc ngắt động tác chào không tự thực
hiện lệnh mức 1. Xòe tay mức 0 ngắt mọi hành vi và xóa hàng đợi ngay từ bản tin
đầu tiên.

Khi robot đang thực hiện mức 1, lệnh mức 1 khác thay lệnh chờ; hành vi hiện tại
tiếp tục đến khi nhận `thumb_up` để chuyển sang lệnh vừa chọn. Vẫy tay mức 2
không được ngắt hành vi mức 1, kể cả khi hàng đợi đã rỗng sau xác nhận. Vẫy tay
chỉ tự đáp trả khi robot không có hành vi đang chạy và hàng đợi rỗng; giữ nguyên
cử chỉ không khởi động lại động tác chào đang chạy. `thumb_up` khi hàng đợi
rỗng không làm gì. Ngón trỏ chỉ xuống chọn lùi và cần xác nhận để bắt đầu lùi;
nắm tay chọn hủy và cũng cần xác nhận. Hành vi đã bị ngắt không tự tiếp tục.

Khi một hành vi hoàn tất, lệnh đang chờ vẫn được giữ và không tự chạy. Điểm
cuối của 4 chặng tiến/lùi cách điểm đầu khoảng 1.4 m theo hướng đã chọn. Nếu lệnh
đã được lấy ra nhưng cảm biến/khoảng trống không cho phép chạy, robot dừng và
lệnh đó vẫn được coi là đã tiêu thụ, cần chọn lại.

Không cần giữ ngón cái trong lúc robot chạy: có thể hạ tay để recognizer gửi
`None`; robot tiếp tục hành vi hiện tại. Phân biệt với **mất luồng topic**:
không nhận được bất kỳ bản tin `/detected_gesture` nào trong 0.8 giây sẽ dừng
chuyển động, nhưng giữ lệnh chờ. Topic hoạt động lại không tự tiếp tục hành vi
bị ngắt. Các kiểm tra vật cản/cảm biến vẫn có thể dừng robot dù nhãn không đổi.
Trạng thái, lệnh chờ (`queued`), hành vi đang chạy (`active`), mức (`level`) và lý do xuất hiện
trong log cùng topic `/robot_behavior/status` (`std_msgs/msg/String`).

World `gesture_demo.sdf` có vùng trống 10 × 10 m, robot spawn tại tâm. Quỹ đạo
mặc định nằm trong bán kính 2 m quanh vị trí khởi tạo. Trước khi chạy, node
yêu cầu khoảng trống 360 độ tối thiểu 0.55 m cho động tác chào, 1.95 m cho
4 chặng tiến/lùi và 1.55 m cho vòng tròn. Trong khi chạy, vật cản cách lidar <= 0.55 m,
scan không hợp lệ, thiếu odometry/scan mới trong 0.6 giây, odometry nhảy bất
thường hoặc ra khỏi vùng demo đều làm robot dừng. Scan phải phủ đủ 360 độ để
kiểm tra cả khu vực quanh robot; `NaN`/giá trị ngoài miền đo bị chặn. Timestamp
cảm biến trùng lặp không làm mới watchdog; timestamp lùi sẽ hủy hành vi.
Watchdog dùng đồng hồ thực đơn điệu, vẫn chạy khi `/clock` mô phỏng bị dừng.

Cơ chế lidar này dừng trước vật cản trong mặt phẳng quét, không lập kế hoạch
đường vòng hay bảo đảm nhận ra vật thấp ngoài mặt phẳng quét. Dùng world demo
trống đã cung cấp cho các chuyển động này. Cấu hình demo dùng lidar phụ ảo ở
độ cao 0.55 m, frame `behavior_lidar_link`, bật bằng `enable_behavior_lidar:=true`.
Lidar gốc `/scan` có các vùng quét trúng trụ của robot và trả `-inf`, vì vậy
không phù hợp cho phép kiểm tra nghiêm ngặt này. Lidar phụ tránh các trụ và có
TF đúng theo vị trí gắn; nó chỉ dùng cho mô phỏng và vẫn không thấy vật thấp
ngoài mặt phẳng quét. Nếu chạy node không có file cấu hình, `scan_topic` mặc
định là `/scan` và node chặn chuyển động nếu có dữ liệu không hợp lệ.
Chỉ chạy một nguồn điều khiển
`/cmd_vel` trong cùng ROS domain; tắt teleop, waypoint navigator và Nav2 trong
domain đó để tránh các nguồn lệnh ghi đè nhau.

Build (không cần MediaPipe để chạy riêng node hành vi):

```bash
cd /home/quan2905/turtle
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select robot_description robot_node robot_sim
```

Mở 3 terminal và chạy phần chuẩn bị sau trong **mỗi terminal**. Domain/partition
riêng tách demo khỏi các phiên small_room/Nav2 đang chạy ở domain khác:

```bash
cd /home/quan2905/turtle
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=70
export IGN_PARTITION=turtle_gesture_demo
```

Terminal 1 — world trống và robot tại tâm; camera mô phỏng không cần dùng vì
nhận diện lấy hình từ webcam USB:

```bash
ros2 launch robot_sim sim.launch.py world:=gesture_demo enable_camera:=false \
  enable_behavior_lidar:=true
```

Terminal 2 — nhận diện webcam:

```bash
.venv-gesture/bin/python -m robot_node.gesture_recognition_node \
  --ros-args -p camera_index:=2
```

Terminal 3 — hành vi robot (đợi mô phỏng có `/odom` và `/behavior_scan`):

```bash
ros2 run robot_node robot_behavior_node --ros-args \
  --params-file src/Turtlebot4/robot_node/config/robot_behavior.yaml
```

Thử lần lượt: giơ ngón trỏ lên → thấy `SELECTED; queued=forward` → giơ ngón
cái → robot tiến 4 chặng và nghỉ 1 giây giữa các chặng; giơ hai ngón rồi xác
nhận → một vòng tròn; vẫy tay khi hàng đợi rỗng → tự quay chào. Thử giơ hai
ngón rồi giơ ngón trỏ lên trước khi xác nhận để kiểm tra lệnh mới thay lệnh cũ.
Giơ ngón trỏ xuống rồi xác nhận để lùi 4 chặng. Nắm tay rồi xác nhận để hủy.
Trong khi robot chào, chọn tiến/lùi/vòng tròn sẽ ngắt chào và chờ xác nhận.
Trong khi robot tiến/lùi/chạy tròn, vẫy tay không ngắt hành vi đó. Xòe tay dừng ngay và
xóa cả lệnh chờ. Hạ tay để trả `None` sẽ giữ hành vi; dừng node nhận diện sẽ
kiểm tra cơ chế dừng do mất topic. Có thể xem trạng thái từ terminal phụ
(cùng phần chuẩn bị ở trên):

```bash
ros2 topic echo /robot_behavior/status
```

Tham số tốc độ, khoảng cách, bán kính và các timeout nằm trong
`robot_node/config/robot_behavior.yaml`; khởi động lại node khi thay đổi.
Các mức tốc độ được giới hạn tối đa 0.2 m/s và 0.8 rad/s cho demo. Vận tốc
giảm khi gần cuối chặng; có khoảng dừng 1 giây giữa các chặng. Quãng đường/góc
được lấy từ odometry, không dùng lệnh `sleep` để chạy chuyển động theo thời gian.

Kiểm tra logic bao gồm thay thế/tiêu thụ hàng đợi, ưu tiên mức 0, chặn vẫy
tay khi có lệnh chờ hoặc mức 1 đang chạy, mức 1 ngắt mức 2 trước xác nhận,
hủy cần xác nhận, giữ nguyên khi nhãn không nhận diện và 4 chặng tiến/lùi có
nghỉ giữa chặng. Kiểm tra ROS dùng topic vận tốc riêng để
kiểm chứng bản tin điều khiển và watchdog khi `/clock` không hoạt động.
Toàn bộ 138 kiểm tra đạt. Thử Gazebo với cử chỉ phát giả lập đã xác nhận mức 1
ngắt chào mức 2 trước xác nhận, vẫy tay không ngắt chuyển động mức 1 dù hàng đợi
rỗng, lùi đủ 4 chặng (thực đo khoảng 1.37 m), 3 khoảng nghỉ 1 giây và mức 0
dừng ngay. Lượt thử kiểm tra điều khiển, không đánh giá nhận diện webcam.
