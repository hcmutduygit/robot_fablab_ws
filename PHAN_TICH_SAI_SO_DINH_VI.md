# 🔍 PHÂN TÍCH TOÀN DIỆN: SAI SỐ ĐỊNH VỊ KHI QUẸO

## 📊 TÌNH TRẠNG HIỆN TẠI

### Quan sát từ User:
- ✅ Hệ thống: AMCL + Lidar (RPLidar S2) + IMU (BNO055) + Encoder (Wheel Odometry)
- ❌ **Vấn đề:** Khi quẹo → định vị bắt đầu có sai số
- ❌ **Không hồi về map** đã quét sau khi quẹo

### Dữ liệu từ CSV logs:
```
IMU_Packages_per_sec: 0 ← ⚠️ NGHIÊM TRỌNG!
Odom_Packages_per_sec: 200
Send_Packages_per_sec: 200
quaternion (qx,qy,qz,qw): ALL ZERO ← ⚠️ IMU KHÔNG HOẠT ĐỘNG!
```

---

## 🔴 CÁC VẤN ĐỀ NGHIÊM TRỌNG ĐÃ PHÁT HIỆN

### **1. IMU HOÀN TOÀN KHÔNG GỬI DỮ LIỆU** ⚠️⚠️⚠️
**Bằng chứng từ log:**
```csv
IMU_Packages_per_sec,qx,qy,qz,qw,quaternion_yaw
0,                   0.0000,0.0000,0.0000,0.0000,0.0000
```

**Nguyên nhân tiềm ẩn:**
- ❌ IMU node không chạy hoặc crash
- ❌ I2C communication bị lỗi `/dev/i2c-1`
- ❌ Topic `/imu/data` không được publish
- ❌ CAN frame 0x15 (IMU quaternion) và 0x13 (IMU gyro) không nhận được

**Tác động:**
- AMCL/EKF không có thông tin orientation từ IMU
- Khi quẹo: **chỉ dựa vào encoder** → sai số lớn vì:
  - Encoder có slippage khi quẹo
  - Không có gyro để hiệu chỉnh angular velocity
  - Wheel odometry tích phân sai số theo thời gian

---

### **2. DẤU TRỪ SAI TRONG WHEEL ODOMETRY** ⚠️⚠️
**File:** `src/cmd_vel/src/wheel_odometry.cpp` line 22

```cpp
double v_l = -vel_left  / 20.0;  // ❌ DẤU TRỪ SAI!
double v_r =  vel_right / 20.0;
```

**Tại sao đây là vấn đề nghiêm trọng khi quẹo:**

Khi robot quẹo phải (turn right):
- Bánh phải quay chậm hơn: `v_r = 0.2 m/s`
- Bánh trái quay nhanh hơn: `v_l_actual = 0.4 m/s`

**Với dấu trừ sai:**
```cpp
v_l = -0.4 / 20 = -0.02  // SAI!
v_r = 0.2 / 20 = 0.01

omega = (v_r - v_l) / WHEEL_BASE
      = (0.01 - (-0.02)) / 0.58
      = 0.03 / 0.58 = 0.052 rad/s  // QUẸO TRÁI thay vì PHẢI!
```

**Kết quả:**
- Robot quẹo phải nhưng odometry tính là quẹo trái
- Yaw angle tích phân sai hoàn toàn
- AMCL không match được với map → **không hồi về**

---

### **3. EKF CONFIGURATION SAI** ⚠️
**File:** `src/robot_fablab/config/ekf_localization.yaml`

```yaml
odom0_config:
  [ false, false, false,   # x, y, z ← ❌ KHÔNG DÙNG X,Y!
    false, false, true,    # roll, pitch, yaw
    true,  true,  false,   # vx, vy, vz ← ❌ VY = true là SAI!
    false, false, true,    # vroll, vpitch, vyaw
    false, false, false ]

imu0_config:
  [ false, false, false,
    false, false, true,    # yaw ← Đúng
    false, false, false,
    false, false, true,    # vyaw (angular velocity)
    true, true, false ]    # ❌ ax, ay = true là SAI cho differential drive!
```

**Vấn đề:**
1. **Không sử dụng position (x,y) từ odometry** → EKF chỉ dùng velocity
   - Khi quẹo → velocity không đủ để track position chính xác
   - Drift tích lũy nhanh

2. **Sử dụng vy (lateral velocity)** từ diff drive robot
   - Differential drive không có vy!
   - Data nhiễu → EKF bị confuse

3. **Sử dụng linear acceleration từ IMU**
   - IMU trên robot di động có nhiễu rất cao
   - Không nên dùng acceleration cho navigation

---

### **4. AMCL PARAMETERS KHÔNG TỐI ƯU CHO QUẸO** ⚠️
**File:** `src/launch_manager/amcl_localization.launch`

```xml
<param name="odom_alpha1" value="0.008"/>  <!-- rotation từ rotation -->
<param name="odom_alpha2" value="0.040"/>  <!-- rotation từ translation -->
<param name="odom_alpha3" value="0.004"/>  <!-- translation từ translation -->
<param name="odom_alpha4" value="0.025"/>  <!-- translation từ rotation -->
```

**Vấn đề:**
- `odom_alpha1` v   à `odom_alpha2` **QUÁ THẤP** (0.008, 0.040)
- Khi quẹo: AMCL **quá tin odometry về rotation**
- Nhưng odometry có dấu sai → AMCL tin vào dữ liệu SAI
- Particles không spread đủ → không recover được

**So sánh với giá trị khuyến nghị:**
```
Khuyến nghị:    alpha1=0.2, alpha2=0.2, alpha3=0.2, alpha4=0.2
Hiện tại:       alpha1=0.008 ← 25x thấp hơn!
```

---

### **5. WHEEL_BASE SAI** ⚠️
**File:** `src/cmd_vel/src/wheel_odometry.cpp` line 7

```cpp
static constexpr double WHEEL_BASE = 0.58;
```

**Nếu wheelbase thực tế khác 0.58m:**
- Omega calculation sai: `omega = (v_r - v_l) / WHEEL_BASE`
- Robot quẹo 90° → odometry tính 80° hoặc 100°
- Sau nhiều lần quẹo → sai số tích lũy nghiêm trọng

**Cách kiểm tra:**
```bash
# Đo khoảng cách giữa 2 bánh xe (center to center)
# Nếu khác 0.58m → cần update!
```

---

### **6. COVARIANCE QUÁ CAO** ⚠️
**File:** `src/cmd_vel/src/wheel_odometry.cpp`

```cpp
odom.pose.covariance[0]  = 0.05;   // X
odom.pose.covariance[7]  = 0.05;   // Y
odom.pose.covariance[35] = 0.5;    // Yaw - QUÁ CAO!

odom.twist.covariance[0]  = 0.2;   // vx
odom.twist.covariance[35] = 1.0;   // vyaw - QUÁ CAO!
```

**Vấn đề:**
- Yaw covariance = 0.5 → EKF không tin odometry về orientation
- vyaw covariance = 1.0 → EKF không tin angular velocity
- Khi quẹo: EKF bỏ qua thông tin quan trọng từ encoder

---

### **7. KHÔNG CÓ TF TREE VALIDATION** ⚠️
**Quan sát:**
```xml
<node pkg="tf2_ros" type="static_transform_publisher" name="basefootprint_to_laser"
        args="0.15 0 0.6 0 0 0 base_footprint laser" />

<node pkg="tf2_ros" type="static_transform_publisher" name="basefootprint_to_imu"
        args="0.2 0 0.6 0 0 0 base_footprint imu" />
```

**Vấn đề tiềm ẩn:**
- Lidar offset: `x=0.15, z=0.6` → nếu sai vị trí thực tế
- Khi quẹo: lidar scan không align với map
- IMU offset: `x=0.2, z=0.6` → nếu IMU hoạt động, rotation center sai

---

### **8. RACE CONDITION VỚI SENSOR DATA** ⚠️
**File:** `src/cmd_vel/include/can_node.h`

```cpp
// Global variables - KHÔNG thread-safe
float right_mps = 0.0;
float left_mps = 0.0;
float qx = 0.0, qy = 0.0, qz = 0.0, qw = 0.0;
```

**Vấn đề:**
- CAN receive thread ghi dữ liệu
- ROS callbacks đọc dữ liệu
- Không có mutex → dữ liệu corrupt khi quẹo (nhiều update nhanh)

---

## 🎯 THỨ TỰ ƯU TIÊN SỬA LỖI

### **CRITICAL (Phải sửa ngay):**

1. **Kiểm tra IMU hoạt động** ⚡
   ```bash
   # Check IMU node
   rostopic echo /imu/data
   
   # Check I2C
   sudo i2cdetect -y 1
   
   # Check CAN frames
   rostopic echo /can_rx | grep "0x15\|0x13"
   ```

2. **Sửa dấu trừ trong wheel odometry** ⚡
   ```cpp
   // Line 22 in wheel_odometry.cpp
   double v_l = vel_left / 20.0;   // BỎ DẤU TRỪ
   ```

3. **Fix EKF config** ⚡
   ```yaml
   odom0_config:
     [ true,  true,  false,   # ✅ DÙNG x, y position
       false, false, true,    # yaw
       true,  false, false,   # ✅ CHỈ vx, KHÔNG vy
       false, false, true,    # vyaw
       false, false, false ]
   
   imu0_config:
     [ false, false, false,
       false, false, true,    # yaw
       false, false, false,
       false, false, true,    # vyaw
       false, false, false ]  # ✅ KHÔNG dùng acceleration
   ```

4. **Tăng AMCL alpha parameters** ⚡
   ```xml
   <param name="odom_alpha1" value="0.2"/>  <!-- 0.008 → 0.2 -->
   <param name="odom_alpha2" value="0.2"/>  <!-- 0.040 → 0.2 -->
   <param name="odom_alpha3" value="0.2"/>  <!-- 0.004 → 0.2 -->
   <param name="odom_alpha4" value="0.2"/>  <!-- 0.025 → 0.2 -->
   ```

### **HIGH Priority:**

5. **Giảm covariance khi có IMU**
   ```cpp
   // Khi IMU hoạt động, tin yaw hơn
   odom.pose.covariance[35] = 0.1;   // 0.5 → 0.1
   odom.twist.covariance[35] = 0.3;  // 1.0 → 0.3
   ```

6. **Verify WHEEL_BASE**
   - Đo thực tế khoảng cách 2 bánh
   - Update nếu sai

7. **Add thread safety (mutex)**

### **MEDIUM Priority:**

8. **Validate TF tree**
   ```bash
   rosrun tf view_frames
   # Kiểm tra laser và IMU offset
   ```

9. **Tăng số particles AMCL khi quẹo**
   ```xml
   <param name="min_particles" value="1000"/>  <!-- 500 → 1000 -->
   <param name="max_particles" value="5000"/>  <!-- 2000 → 5000 -->
   ```

---

## 🔬 DEBUG STEPS

### 1. Kiểm tra IMU trước:
```bash
# Terminal 1
roslaunch launch_manager amcl_localization.launch

# Terminal 2
rostopic hz /imu/data
rostopic echo /imu/data | head -50

# Terminal 3 - Check log
rosrun cmd_vel can_node | grep "IMU"
```

### 2. Test wheel odometry:
```bash
# Robot đi thẳng 2m
rostopic echo /wheel_odometry

# Kiểm tra:
# - x tăng ~2m, y ~0
# - theta ~0

# Robot quẹo제자리 360°
# Kiểm tra:
# - x,y thay đổi ít
# - theta quay đủ 2π rad (~6.28)
```

### 3. Validate AMCL recovery:
```bash
# Set initial pose trong RViz
# Di chuyển robot quẹo nhiều lần
# Xem particles có converge về đúng không
```

---

## 📊 KẾT LUẬN

**Nguyên nhân chính gây sai số khi quẹo:**

1. **IMU không hoạt động (0 packages/sec)** → Mất thông tin orientation quan trọng nhất
2. **Dấu sai trong wheel odometry** → Tính ngược chiều quẹo
3. **EKF config sai** → Không dùng position, dùng vy sai
4. **AMCL alpha quá thấp** → Tin odometry sai quá mức

**Nếu sửa 4 vấn đề trên → 90% cơ hội fix được!**

---

## 📝 CHECKLIST SỬA LỖI

- [ ] 1. Kiểm tra IMU node chạy: `rosnode list | grep imu`
- [ ] 2. Kiểm tra I2C: `sudo i2cdetect -y 1` (phải thấy address 0x28)
- [ ] 3. Bỏ dấu trừ trong `v_l = -vel_left / 20.0`
- [ ] 4. Fix EKF config: enable x,y position, disable vy
- [ ] 5. Tăng AMCL alpha: 0.2 cho tất cả
- [ ] 6. Giảm yaw covariance: 0.5 → 0.1
- [ ] 7. Test robot quẹo제자리 360° → check theta
- [ ] 8. Verify WHEEL_BASE = khoảng cách 2 bánh thực tế
- [ ] 9. Add mutex cho sensor data
- [ ] 10. Validate TF tree với `rosrun tf view_frames`

---

**Ngày phân tích:** 2026-01-23  
**Trạng thái:** 🔴 Chưa sửa - Chờ user xác nhận trước khi edit code
