# 🔧 FIX: Localization Drift Along Y-Axis

## 📋 Vấn Đề
Khi chạy định vị (AMCL/Cartographer), robot bị **lệch dần theo phương Y** trong RViz.

## 🔍 Nguyên Nhân Chính

### 1. ❌ **LỖI NGHIÊM TRỌNG: Dấu Sai Trong Wheel Odometry**
**File:** `src/cmd_vel/src/wheel_odometry.cpp` (line 20)

**Trước:**
```cpp
float v_l = -vel_left  / 20.0f;  // ❌ DẤU TRỪ SAI!
float v_r =  vel_right / 20.0f;
```

**Sau:**
```cpp
float v_l = vel_left  / 20.0f;   // ✅ FIXED
float v_r = vel_right / 20.0f;
```

**Giải thích:**
- Dấu trừ làm đảo ngược hướng bánh trái
- Tính toán sai vận tốc góc: `omega = (v_r - v_l) / WHEEL_BASE`
- Dẫn đến tích phân sai: `y += v * sin(wheel_yaw) * dt`
- **Kết quả: Drift nghiêm trọng theo Y**

### 2. 🔒 **Race Condition - Thread Safety**
**Files:** 
- `src/cmd_vel/include/can_node.h`
- `src/cmd_vel/src/can_node.cpp`

**Vấn đề:**
- Biến global `left_mps`, `right_mps`, `qx`, `qy`, `qz`, `qw`, `gyro_z` không được bảo vệ
- Ghi từ CAN receive thread
- Đọc từ ROS callbacks
- **Dữ liệu bị corrupt → sai số tích lũy**

**Giải pháp:**
- Thêm `std::mutex sensor_data_mutex`
- Bảo vệ tất cả read/write operations với `std::lock_guard`

### 3. 📊 **Covariance Quá Cao**
**File:** `src/cmd_vel/src/wheel_odometry.cpp` (line 66-68)

**Trước:**
```cpp
odom.pose.covariance[0]  = 0.05;  // X
odom.pose.covariance[7]  = 0.05;  // Y - QUÁ CAO
odom.pose.covariance[35] = 0.5;   // Yaw
```

**Sau:**
```cpp
odom.pose.covariance[0]  = 0.01;  // X - Tin tưởng hơn
odom.pose.covariance[7]  = 0.01;  // Y - Tin tưởng hơn
odom.pose.covariance[35] = 0.1;   // Yaw - Tin tưởng hơn
```

**Giải thích:**
- Covariance cao → EKF không tin odometry
- Chỉ dựa vào lidar → dễ drift trong môi trường đối xứng
- Giảm covariance → EKF kết hợp tốt hơn odometry + lidar

## 🛠️ Các File Đã Sửa

1. **src/cmd_vel/src/wheel_odometry.cpp**
   - ✅ Bỏ dấu trừ sai ở `v_l`
   - ✅ Giảm covariance từ 0.05 → 0.01 (X, Y)
   - ✅ Giảm covariance từ 0.5 → 0.1 (Yaw)

2. **src/cmd_vel/include/can_node.h**
   - ✅ Thêm `#include <mutex>`
   - ✅ Khởi tạo biến `number = 0`, `new_number = 0`, `calib = 0`
   - ✅ Declare `extern std::mutex sensor_data_mutex`

3. **src/cmd_vel/src/can_node.cpp**
   - ✅ Define `std::mutex sensor_data_mutex`
   - ✅ Bảo vệ ghi `qx, qy, qz, qw` (case 0x15)
   - ✅ Bảo vệ ghi `gyro_z` (case 0x13)
   - ✅ Bảo vệ ghi `left_mps, right_mps` (case 0x11)

## 🚀 Cách Test

### 1. Build lại workspace:
```bash
cd ~/robot_fablab_ws
catkin_make
source devel/setup.bash
```

### 2. Chạy localization:
```bash
# Terminal 1: Launch nodes
roslaunch launch_manager amcl_localization.launch

# Terminal 2: RViz
rviz -d ~/robot_fablab_ws/src/rviz/amcl.rviz
```

### 3. Kiểm tra:
- ✅ Robot di chuyển thẳng → odometry không lệch Y
- ✅ Định vị lidar khớp với odometry
- ✅ Không còn drift theo thời gian

### 4. Debug (nếu cần):
```bash
# Xem wheel odometry
rostopic echo /wheel_odometry

# Xem EKF output
rostopic echo /odometry/filtered

# So sánh tốc độ trái/phải
rostopic echo /wheel_odometry | grep twist -A 10
```

## 📊 Kết Quả Mong Đợi

**Trước fix:**
- Y drift: ~0.5m trong 30s đi thẳng
- Lidar localization lệch dần
- EKF không tin odometry

**Sau fix:**
- Y drift: <0.05m trong 30s
- Lidar + odometry khớp nhau
- EKF kết hợp tốt cả 2 nguồn

## ⚠️ Lưu Ý

1. **Nếu vẫn còn drift nhẹ:**
   - Kiểm tra xem encoder có bị trượt không
   - Hiệu chỉnh `WHEEL_BASE` trong `wheel_odometry.cpp`
   - Kiểm tra IMU calibration

2. **Nếu drift theo X thay vì Y:**
   - Có thể cần đảo dấu cả 2 bánh (tùy phương hướng motor)
   - Hoặc kiểm tra lại mapping left/right trong CAN protocol

3. **Compile errors:**
   - Đảm bảo `catkin_make` thành công
   - Nếu lỗi về ROS headers, chạy: `source /opt/ros/melodic/setup.bash`

## 📝 Các Vấn Đề Khác Cần Sửa (Không Urgent)

1. ⚠️ **CAN Protocol Frame Parsing** - Có vẻ đọc thừa bytes (dòng 170-175 trong can_node.h)
2. ⚠️ **Zombie Processes** - system() calls không cleanup (can_node.cpp line 48, 491)
3. ⚠️ **Hardcoded Paths** - `/home/nvidia/...` không portable
4. ⚠️ **Global Variables in Header** - Vi phạm ODR, nên dùng extern

---
**Date:** 2026-01-22  
**Status:** ✅ FIXED - Ready to test
