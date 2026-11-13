# Quay video bằng giọng người dùng qua microphone

Chạy `record_microphone_demo.py` để mở giao diện thu chủ động. Chương trình
tự khởi động robot trong Gazebo, localization và Nav2 ở ROS domain riêng.
Microphone chỉ mở sau khi bạn nhấn **Thu lệnh** và đếm ngược ba giây; mỗi
lượt thu mười giây. Không thu âm khi chờ nút hoặc khi robot đang di chuyển.

Giao diện hiện tại đọc catalog và sinh các lượt theo vùng đang lưu. Với bộ
vùng A/B/C/D, lần lượt đọc sáu câu được hiển thị:

1. Go to area A.
2. Go to area B.
3. Go to area C.
4. Go to area D.
5. Go from area A, via area B, via area C, then to area D.
6. Go to unknown area.


Mỗi lượt tiếp theo cần bạn nhấn nút. Nếu không có tiếng nói hoặc câu lệnh
bị nhận dạng không hợp lệ, giao diện cho thu lại. Văn bản nhận dạng do
Whisper (mặc định language=en) tạo từ file ghi âm thật; không thay bằng câu gợi ý.

Sau khi hoàn thành, video được lưu thành
`turtlebot4_voice_microphone_demo.mp4` trong `../demo_microphone_current/`, cùng WAV đầu vào,
manifest, sự kiện ROS và log. File chỉ xuất sau khi quay xong; video tổng hợp
trước đó vẫn nằm riêng trong `../demo/`.

Chạy lại từ workspace:

```bash
cd ~/turtle
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=90
export IGN_PARTITION=turtle_user_microphone_video
export LIBGL_ALWAYS_SOFTWARE=1
export OMP_NUM_THREADS=4
export MKL_NUM_THREADS=4
.venv-voice/bin/python src/Turtlebot4/Voice-Controlled/record_microphone_demo.py
```

Đóng cửa sổ để kết thúc phiên đang chờ hoặc hủy phiên thu. Nếu không có
âm thanh, chọn thiết bị đầu vào khác trong danh sách ở dưới cửa sổ và thử
lại. Chưa hoàn thành đủ các lượt thì chưa có video microphone hoàn chỉnh.
