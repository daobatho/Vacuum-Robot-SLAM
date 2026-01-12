# Vacuum-Robot-SLAM
Autonomous vacuum robot using ROS2 and slam_toolbox with rotating TF-Luna LiDAR on Raspberry Pi &amp; ESP32 for real-time 2D mapping.

## Tổng quan đồ án

**Vacuum-Robot-SLAM** là một đồ án robot tự hành tập trung vào bài toán **định vị và xây dựng bản đồ môi trường (2D SLAM)** trong không gian trong nhà, phục vụ cho ứng dụng robot hút bụi thông minh.

Thay vì sử dụng LiDAR thương mại, đồ án xây dựng một **hệ thống LiDAR tùy chỉnh** dựa trên cảm biến đo khoảng cách **TF-Luna** kết hợp với **động cơ bước quay 180° qua lại**, từ đó tạo ra dữ liệu quét không gian và chuyển đổi sang chuẩn `sensor_msgs/LaserScan` của ROS2.

Hệ thống sử dụng:
- **Raspberry Pi** làm bộ xử lý trung tâm, chạy ROS2, SLAM và trực quan hóa dữ liệu
- **ESP32** để điều khiển động cơ bánh xe và tính toán odometry
- **TF-Luna LiDAR + Stepper Motor (ULN2003)** để tạo dữ liệu quét môi trường
- **slam_toolbox** để thực hiện SLAM thời gian thực

Robot có khả năng:
- Thu thập dữ liệu khoảng cách từ môi trường xung quanh
- Xây dựng bản đồ 2D theo thời gian thực
- Theo dõi và cập nhật vị trí robot trong bản đồ
- Hiển thị kết quả trên RViz2

Đồ án hướng tới mục tiêu:
- Hiểu rõ kiến trúc ROS2 và hệ thống TF
- Làm chủ pipeline SLAM từ phần cứng đến phần mềm
- Ứng dụng kiến thức Embedded + Robotics vào một hệ thống thực tế
- Xây dựng nền tảng cho các chức năng nâng cao như **Navigation2** và tự hành hoàn toàn
## Cài đặt môi trường và gói phần mềm
- cài ros2 
```bash
sudo apt install ros-humble-ros-base
```
- cài SLAM
```bash
sudo apt install ros-humble-slam-toolbox
```
- Cài hệ thống navigation2
```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
```
- Cài Rviz2
```bash
sudo apt install ros-humble-rviz2
```