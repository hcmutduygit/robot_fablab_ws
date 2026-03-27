#include <cmath>
#include "can_node.h"
#include "wheel_odometry.h"
#include <cstdlib>
#include <string>
#include <sstream>
#include <map>
#include <vector>
#include <chrono> // For time measurement
#include <ctime>
#include <iomanip>
#include <fstream>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h> // add this line
#define PI 3.14159265358979323846

// Global variable to store the updated value
float yaw_angle = 0;
int right_wheel_velocity = 0;
int left_wheel_velocity = 0;
float x = 0;
float y = 0;
float wheel_yaw = 0;
float yaw = 0;
std::mutex wheel_odom_mutex; 
float yaw_offset = 0;
float yaw_prev = 0.0;
bool initialized = false;
int odom_count = 0;
float qx_prev, qy_prev, qz_prev, qw_prev;
// ros::Time lasttime;



static volatile int g_should_exit = 0;

WaveshareCAN can("/dev/ttyUSB0", 2000000, 2.0);

int ConvertPulse(float &velocity)
{
    // Convert m/s to rounds per second (assuming wheel radius is 0.1 m)
    const float wheel_radius = 100;                                                                 // in millimeters
    const int pulse_per_revolution = 10000;                                                         // Assuming 360 pulses per revolution
    int pulse = static_cast<int>(pulse_per_revolution * velocity * 1000 / (2 * PI * wheel_radius)); // Convert m/s to pulses
    return pulse;                                                                                   // Pulse per second
}

// Inverse: convert pulses per second back to linear velocity (m/s)
float ConvertVelocityFromPulse(int pulse)
{
    // v (m/s) = (pulse / PPR) * circumference(mm) / 1000
    const float wheel_radius = 100.0f;      // in millimeters
    const int pulse_per_revolution = 10000; // pulses per wheel revolution
    const float circumference_mm = 2.0f * PI * wheel_radius;

    float velocity_mps = (static_cast<float>(pulse) * circumference_mm) /
                         (static_cast<float>(pulse_per_revolution) * 1000);
    return velocity_mps;
}

void send_vel(WaveshareCAN &can) //0x030
{
    try
    {
        // Get integer velocities
        int right_vel = right_wheel_velocity;
        int left_vel = left_wheel_velocity;
        // int right_vel = 3000;
        // int left_vel = -3000;
     
        // Create 8-byte data array: first 4 bytes for left wheel, last 4 bytes for right wheel
        uint8_t data[8];

        // Convert left velocity to bytes (first 4 bytes)
        std::memcpy(data, &left_vel, sizeof(int));

        // Convert right velocity to bytes (last 4 bytes)
        std::memcpy(data + 4, &right_vel, sizeof(int));

        // Create data vector
        std::vector<uint8_t> velocity_data(data, data + 8);

        // Send both velocities to single ID 0x013
        can.send(0x030, velocity_data);
        // cnt_send++;
        // std::cout << "Sent left velocity " << left_vel << " and right velocity " << right_vel << " to ID 0x013" << std::endl;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Invalid velocity input: " << e.what() << std::endl;
    }
}


// Convert two bytes to a signed 16-bit integer
int16_t hex_to_signed(const std::vector<uint8_t> &data, size_t start_idx, size_t bits = 16)
{
    uint16_t value = (data[start_idx] << 8) | data[start_idx + 1];
    // Convert unsigned to signed using proper casting
    int16_t signed_value = static_cast<int16_t>(value);
    return signed_value;
}

// Convert two bytes to an unsigned 16-bit integer for angles
uint16_t hex_to_unsigned(const std::vector<uint8_t> &data, size_t start_idx)
{
    // Combine two bytes into a 16-bit unsigned integer (big-endian)
    return static_cast<uint16_t>((data[start_idx] << 8) | data[start_idx + 1]);
}

float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Process CAN frame (equivalent to Python's process_frame)
void process_frame(uint16_t can_id, const std::vector<uint8_t> &data, ros::Publisher& odom_pub, ros::Time& lasttime)
{
    switch (can_id)
    {
    case 0x11://encoder 
    {
        // Ensure the data has exactly 8 bytes
        if (data.size() != 8)
        {
            std::cerr << "Error: Expected 8 bytes for ID 0x011, but received " << data.size() << " bytes.\n";
            return;
        }

        // Extract left velocity from first 4 bytes
        int received_left_vel;
        std::memcpy(&received_left_vel, &data[0], sizeof(int));

        // Extract right velocity from last 4 bytes
        int received_right_vel;
        std::memcpy(&received_right_vel, &data[4], sizeof(int));

        // Convert pulses to linear velocity (m/s)
        left_mps = ConvertVelocityFromPulse(received_left_vel);
        right_mps = ConvertVelocityFromPulse(received_right_vel);
        // std::cout << std::fixed << std::setprecision(3);
        // std::cout << "Converted Left Velocity (m/s): " << left_mps << "\n";
        // std::cout << "Converted Right Velocity (m/s): " << right_mps << "\n";
        updateWheelOdometry(left_mps, right_mps, quaternion_yaw, odom_pub, lasttime);
        cnt_receive_odom++;
        break;
    }
    default:
        break;
    }
}

void callbackImu(const sensor_msgs::Imu::ConstPtr& msg)
{
    // Process IMU data
    qx = msg->orientation.x;
    qy = msg->orientation.y;
    qz = msg->orientation.z;
    qw = msg->orientation.w;
    quaternion_yaw = tf::getYaw(msg->orientation);
    // std::cout << "imu yaw: " << quaternion_yaw * 180/PI << "\n";
}

std::string generateCSVFileName() 
{
    // Tạo tên file dựa trên timestamp khi bắt đầu chương trình
    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    
    std::stringstream filename;
    filename << "/home/nvidia/robot_fablab_ws/src/cmd_vel/csv_data/can_data_";
    filename << std::put_time(std::localtime(&now_time), "%Y%m%d_%H%M%S");
    filename << ".csv";
    
    return filename.str();
}

void saveDataToCSV(int imu_packages, int odom_packages, int send_packages)
{
    // Tạo timestamp cho dữ liệu
    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()) % 1000;
    
    std::stringstream timestamp;
    timestamp << std::put_time(std::localtime(&now_time), "%Y-%m-%d %H:%M:%S");
    timestamp << "." << std::setfill('0') << std::setw(3) << ms.count();
    
    // Biến static để lưu tên file - chỉ tạo 1 lần khi chương trình bắt đầu
    static std::string csv_file_path = generateCSVFileName();
    static bool is_first_write = true;
    
    // Thông báo tên file khi lần đầu ghi
    if (is_first_write) {
        ROS_INFO("CSV file created: %s", csv_file_path.c_str());
        is_first_write = false;
    }
    
    // Kiểm tra xem file có tồn tại không để thêm header
    bool file_exists = std::ifstream(csv_file_path).good();
    
    std::ofstream csv_file(csv_file_path, std::ios::app);
    if (csv_file.is_open()) {
        // Thêm header nếu file mới tạo
        if (!file_exists) {
            csv_file << "Timestamp,IMU_Packages_per_sec,Odom_Packages_per_sec,Send_Packages_per_sec,"
                     << "Yaw_Angle,Left_Velocity_mps,Right_Velocity_mps,"
                     << "qx,qy,qz,qw,x,y,quaternion_yaw\n";
        }
        
        // Ghi dữ liệu
        csv_file << timestamp.str() << "," 
                 << imu_packages << "," 
                 << odom_packages << "," 
                 << send_packages << ","
                 << std::fixed << std::setprecision(2) << yaw_angle << ","
                 << std::setprecision(4) << left_mps << ","
                 << std::setprecision(4) << right_mps << ","
                 << std::setprecision(4) << qx << ","
                 << std::setprecision(4) << qy << ","
                 << std::setprecision(4) << qz << ","
                 << std::setprecision(4) << qw << ","
                 << std::setprecision(4) << x << ","
                 << std::setprecision(4) << y << ","
                 << std::setprecision(4) << quaternion_yaw << "\n";
        csv_file.close();
        
        ROS_INFO("Data saved: IMU=%d, Odom=%d, Send=%d, Yaw=%.2f, Left=%.4f m/s, Right=%.4f m/s, qx=%.4f, qy=%.4f, qz=%.4f, qw=%.4f, x=%.4f, y=%.4f, quat_yaw=%lf", 
                 imu_packages, odom_packages, send_packages, yaw_angle, 
                 left_mps, right_mps, qx, qy, qz, qw, x, y, quaternion_yaw);
    } else {
        ROS_ERROR("Cannot open CSV file: %s", csv_file_path.c_str());
    }
}

void CntBytes(const ros::TimerEvent &event)
{
    ROS_WARN("Receive IMU Packages = %d Pkg/s", cnt_receive_imu);
    ROS_WARN("Receive Odom Packages = %d Pkg/s", cnt_receive_odom);
    ROS_WARN("Send Packages = %d Pkg/s", cnt_send);
    
    // Lưu dữ liệu vào CSV trước khi reset
    saveDataToCSV(cnt_receive_imu, cnt_receive_odom, cnt_send);
    
    // Publish telemetry data to MQTT (non-blocking, low overhead)
    if (!g_should_exit) {
        std::string python_script = "/home/nvidia/robot_fablab_ws/src/MQTT/telemetry_publisher.py";
        std::stringstream command;
        // Sử dụng nohup để tránh zombie processes
        command << "nohup python2 " << python_script << " " 
                << cnt_receive_imu << " " 
                << cnt_receive_odom << " " 
                << cnt_send << " >/dev/null 2>&1 &";
        
        system(command.str().c_str());
        // Không cần kiểm tra result vì đã chạy background
    }
    
    // Reset counters
    cnt_receive_imu = 0;
    cnt_receive_odom = 0;
    cnt_send = 0;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Can_node");
    ros::NodeHandle nh;
    odom_pub = nh.advertise<nav_msgs::Odometry>("wheel_odometry", 10);
    imu_sub = nh.subscribe<sensor_msgs::Imu>("imu/data", 10, callbackImu);
    ros::Time lasttime = ros::Time::now();

    can.open();
    can.start_receive_loop([&](uint16_t can_id, const std::vector<uint8_t>& data) {
        process_frame(can_id, data, odom_pub, lasttime);
    });
    ros::NodeHandle arg_nh("~");
    nh.getParam("mode", number);
    arg_nh.getParam("calib", calib);
    arg_nh.getParam("cycle_transmit", cycle_transmit);
    ROS_INFO("mode = %d", number);

    uint8_t data[8];
    std::memcpy(data, &number, sizeof(int));
    std::vector<uint8_t> velocity_data(data, data + 8);
    can.send(0x020, velocity_data); // chuyen mode /cmd_vel/mode
    if (number == 1) {
        can.send(0x020, {1, 0, 0, 0, 0, 0, 0, 0});
        std::cout << "send 1" << "\n";
    }
    else if (number == 2) {
        can.send(0x020, {2, 0, 0, 0, 0, 0, 0, 0});
        std::cout << "send 2" << "\n";
    } 

    cnt_byte = nh.createTimer(ros::Duration(10), CntBytes);
    
    // Master request 
    loopControl = nh.createTimer(
        ros::Duration(cycle_transmit),
        [&](const ros::TimerEvent&) {
            can.send(0x050, {1, 0, 0, 0, 0, 0, 0, 0}); // slave master - gửi trước
            cnt_send++;
        }
    );

     // Thêm biến mới để kiểm tra runtime thay đổi mode
    new_number = number;

    // Giữ nguyên ros::spin() nhưng thêm vòng kiểm tra trong luồng song song
    std::thread param_monitor([&]() {
        ros::Rate rate(5.0); // kiểm tra 2Hz
        while (ros::ok()) {
            arg_nh.getParam("mode", new_number);
            if (new_number != number) {
                number = new_number;
                ROS_WARN("Mode changed at runtime to: %d", number);
                uint8_t data2[8];
                std::memcpy(data2, &number, sizeof(int));
                std::vector<uint8_t> velocity_data2(data2, data2 + 8);
                can.send(0x020, velocity_data2);
            }
            rate.sleep();
        }
    });

    ros::spin();
    param_monitor.join(); // đảm bảo thread kết thúc gọn gàng khi node tắt
    return 0;
}
