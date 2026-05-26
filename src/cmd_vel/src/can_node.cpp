#include <cmath>
#include "can_node.h"
#include "wheel_odometry.hpp"
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
#include <geometry_msgs/PoseWithCovarianceStamped.h>
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
float theta_yaw = 0; // Biến mới để lưu góc yaw từ AMCL
std::mutex wheel_odom_mutex; 
float yaw_offset = 0;
float yaw_prev = 0.0;
bool initialized = false;
int odom_count = 0;
float qx_prev, qy_prev, qz_prev, qw_prev;
// ros::Time lasttime;



static volatile int g_should_exit = 0;

WaveshareCAN can("/dev/usbcan", 2000000, 2.0);

// RFID database - mapping RFID data to user info
static const std::map<std::vector<uint8_t>, std::pair<std::string, std::string>> rfid_database = {
    {{0xd2, 0x0f, 0x49, 0x2e, 0xba, 0x55, 0xaa, 0xc8}, {"HOAI PHU", "Phu"}},
    {{0xd2, 0xb1, 0x3d, 0x05, 0x5b, 0x55, 0xaa, 0xc8}, {"MINH KY", "Ky"}},
    {{0xfa, 0xdc, 0x02, 0xcd, 0xe9, 0x55, 0xaa, 0xc8}, {"QUANG DUY", "Duy"}},
    {{0xef, 0xa8, 0x98, 0x1e, 0xc1, 0x55, 0xaa, 0xc8}, {"CHI THIEN", "Thien"}},
    {{0xb6, 0x87, 0x13, 0x2b, 0x09, 0x55, 0xaa, 0xc8}, {"VAN LOI", "Loi"}},
    {{0xc2, 0xbf, 0xb0, 0x2e, 0xe3, 0x55, 0xaa, 0xc8}, {"BACH THU", "Thu"}},
    {{0xd2, 0xb8, 0x3d, 0x04, 0x5b, 0x55, 0xaa, 0xc8}, {"DINH HUY", "Huy"}}};

// Simple function to publish MQTT message via Python script
void publishMQTTMessage(const std::string &user_name, const std::string &mqtt_msg, const std::string &timestamp)
{
    // Don't publish if we're shutting down
    if (g_should_exit) {
        return;
    }
    
    std::string python_script = "/home/nvidia/robot_fablab_ws/src/MQTT/name_publisher.py";
    std::string command = "setsid timeout 2 python2 " + python_script + " \"" + mqtt_msg + "\" \"" + user_name + "\" \"" + timestamp + "\" &";

    // std::cout << "Publishing MQTT message for " << user_name << " at " << timestamp << ": " << mqtt_msg << std::endl;
    int result = system(command.c_str());

    if (result == 0)
    {
        // std::cout << "MQTT message sent successfully!" << std::endl;
    }
    else if (!g_should_exit)
    {
        // std::cout << "Failed to send MQTT message!" << std::endl;
    }
}

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

void CallBackVel(const utils::cmd_vel::ConstPtr &cmd_vel)
{
    v_left = cmd_vel->v_left;
    v_right = cmd_vel->v_right;

    // ROS_INFO("vel_left guidance = %f, vel_right guidance = %f", v_left, v_right);

    v_left *= 20;
    v_right *= 20;

    left_wheel_velocity = ConvertPulse(v_left);
    right_wheel_velocity = ConvertPulse(v_right);

    if (number==2) {
        send_vel(can);
    }
    // send_vel(can);
}

void CallBackAmclPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    double orientation_x = msg->pose.pose.orientation.x;
    double orientation_y = msg->pose.pose.orientation.y;
    double orientation_z = msg->pose.pose.orientation.z;
    double orientation_w = msg->pose.pose.orientation.w;

    tf::Quaternion q(orientation_x, orientation_y, orientation_z, orientation_w);
    double roll, pitch, amcl_yaw;
    tf::Matrix3x3(q).getRPY(roll, pitch, amcl_yaw);
    
    // Lưu giá trị yaw (radian) vào biến toàn cục
    theta_yaw = amcl_yaw * 180 / PI; 
    // ROS_INFO("Received theta_yaw: %f", theta_yaw);
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

void publish_yaw(float yaw_angle) 
{
    utils::pose_robot pose;
    pose.yaw = yaw_angle;
    pub.publish(pose);
    // ROS_INFO("yaw_angle = %f", yaw_angle);
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
    // RFID
    case 0x19:
    {
        std::cout << "ID 0x" << std::hex << can_id << std::dec << " receive RFID hex: ";
        for (uint8_t b : data)
        {
            std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)b << " ";
        }
        std::cout << std::dec << std::endl;

        // Lookup RFID in database
        auto it = rfid_database.find(data);
        if (it != rfid_database.end())
        {
            const std::string &full_name = it->second.first;
            const std::string &short_name = it->second.second;

            auto now = std::chrono::system_clock::now();
            std::time_t now_time = std::chrono::system_clock::to_time_t(now);

            std::stringstream ss;
            ss << std::put_time(std::localtime(&now_time), "%H:%M:%S");
            std::string timestamp = ss.str();

            std::cout << "RFID detected: " << full_name << std::endl;
            publishMQTTMessage(full_name, short_name, timestamp);
        }
        else
        {
            std::cout << "Unknown RFID data" << std::endl;
        }

        break;
    }
    // // CO2 Sensor
    // case 0x41:
    // {
    //     cnt_receive++;
    //     break;
    // }
    // IMU Angle
    case 0x12:
    {
        // // Ensure data has at least 6 bytes for roll, pitch, yaw (2 bytes each)
        // if (data.size() < 6)
        // {
        //     std::cerr << "Error: Insufficient data bytes for ID 0x012\n";
        //     return;
        // }
        // // std::cout << "ID 0x" << std::hex << can_id << std::dec << " receive IMU hex: ";
        // // for (uint8_t b : data)
        // // {
        // //     std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)b << " ";
        // // }
        // // std::cout << std::dec << std::endl;
        // // Extract roll, pitch, yaw as signed 16-bit integers and scale by 100.0
        // // double roll = hex_to_signed(data, 0) / 100.0;  // Bytes 0-1
        // // double pitch = hex_to_signed(data, 2) / 100.0; // Bytes 2-3
        // float raw_yaw = hex_to_unsigned(data, 4) / 100.0; // Bytes 4-5
        // // float raw = 0;
        // // if (raw_yaw>341 && raw_yaw<360) raw=mapf(raw_yaw,341,360,333,360);
        // // else if (raw_yaw>280 && raw_yaw<341) raw=mapf(raw_yaw,280,341,243,333);
        // // else if (raw_yaw>147 && raw_yaw<280) raw=mapf(raw_yaw,147,280,153,243);
        // // else if (raw_yaw>44.5 && raw_yaw<147) raw=mapf(raw_yaw,44.5,147,63.18,153);
        // // else if (raw_yaw>0 && raw_yaw<44.5) raw=mapf(raw_yaw,0,44.5,0,63.18); 
        // while (raw_yaw > 180.0)
        // {
        //     raw_yaw-= 360.0;
        // }
        // while (raw_yaw <= -180.0)
        // {
        //     raw_yaw += 360.0;
        // }
        
        // yaw_angle = raw_yaw; // Update global yaw angle
        // // publish_yaw(yaw);
        // // std::cout << "roll_degree: " << roll << "\n";
        // // std::cout << "pitch_degree: " << pitch << "\n";
        // // std::cout << "Yaw_degree: " << raw_yaw << "\n";
        // // updateOdometry(left_mps, right_mps, odom_pub, lasttime);
        // cnt_receive_imu++;
        break;
    }
    case 0x15:  // IMU quaternion
    {
        // Ensure data has at least 8 bytes for quaternion (4 bytes each)
        if (data.size() < 8)
        {
            std::cerr << "Error: Insufficient data bytes for ID 0x015\n";
            return;
        }
        // std::cout << "ID 0x" << std::hex << can_id << std::dec << " receive IMU hex: ";
        // for (uint8_t b : data)
        // {
        //     std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)b << " ";
        // }
        // std::cout << std::dec << std::endl;
        // Extract roll, pitch, yaw as signed 16-bit integers and scale by 100.0
        qw = hex_to_signed(data, 0) / 10000.0;  // Bytes 0-1
        qx = hex_to_signed(data, 2) / 10000.0; // Bytes 2-3
        qy = hex_to_signed(data, 4) / 10000.0; // Bytes 4-5
        qz = hex_to_signed(data, 6) / 10000.0; // Bytes 6-7

        // std::cout << "qx=" << qx << ", qy=" << qy << ", qz=" << qz << ", qw=" << qw << "\n";

        tf::Quaternion q(qx, qy, qz, qw);
        tf::Matrix3x3(q).getRPY(roll, pitch, quaternion_yaw);
        // quaternion_yaw = quaternion_yaw * 180.0 / PI;
        // std::cout << "Quaternion Yaw (deg): " << quaternion_yaw * 180.0 / PI << "\n";
        cnt_receive_imu++;
        break;
    }
    // IMU Gyro
    case 0x13:
    {
        if (data.size() < 6) {
            ROS_WARN("IMU Gyro CAN frame invalid size!");
            break;
        }

        // Giải mã 3 trục Gyro từ 6 byte đầu
        int16_t gx_raw = (int16_t)((data[0] << 8) | data[1]);
        int16_t gy_raw = (int16_t)((data[2] << 8) | data[3]);
        int16_t gz_raw = (int16_t)((data[4] << 8) | data[5]);

        // Vì STM32 gửi nguyên giá trị float cast sang int16_t ⇒ đơn vị °/s
        float gx_deg = gx_raw/16.0f;
        float gy_deg = (float)gy_raw/16.0f;
        float gz_deg = (float)gz_raw/16.0f;

        float gx_rad = gx_deg * PI / 180.0f;
        float gy_rad = gy_deg * PI / 180.0f;
        float gz_rad = gz_deg * PI / 180.0f;

        // Lưu global biến Z
        gyro_z = gz_rad;

        // Debug
        // std::cout << "[IMU Gyro] Z=" << gz_rad << " rad/s" << std::endl;
        cnt_receive_imu++;
        break;
    }
    // // IMU Accel
    // case 0x43:
    // {
    //     cnt_receive++;
    //     break;
    // }
    // case 0x16:
    // {
    //     // Ensure the data has exactly 8 bytes
    //     if (data.size() != 8)
    //     {
    //         std::cerr << "Error: Expected 8 bytes for ID 0x016, but received " << data.size() << " bytes.\n";
    //         return;
    //     }
    //     // Split the 8 bytes into 4 groups of 2 bytes and convert to integers
    //     int group1 = (data[0] << 8) | data[1]; // Combine bytes 0 and 1
    //     int group2 = (data[2] << 8) | data[3]; // Combine bytes 2 and 3
    //     int group3 = (data[4] << 8) | data[5]; // Combine bytes 4 and 5
    //     int group4 = (data[6] << 8) | data[7]; // Combine bytes 6 and 7

    //     // // Print the results
    //     // std::cout << "ID 0x016 received: ";
    //     // for (uint8_t b : data)
    //     // {
    //     //     std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)b << " ";
    //     // }
    //     // std::cout << std::dec << "\n";

    //     // std::cout << "Truoc: " << group1 << "\n";
    //     // std::cout << "Phai: " << group2 << "\n";
    //     // std::cout << "Trai: " << group3 << "\n";
    //     // std::cout << "Sau: " << group4 << "\n";
    //     cnt_receive++;
    //     break;
    // }
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

        // // Print the received data in hex format
        // std::cout << "ID 0x017 received: ";
        // for (uint8_t b : data)
        // {
        //     std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)b << " ";
        // // }
        // std::cout << std::dec << "\n";

        // Print the received velocities (pulses per second)
        // std::cout << "Received Left Velocity (pulses/s): " << received_left_vel << "\n";
        // std::cout << "Received Right Velocity (pulses/s): " << received_right_vel << "\n";

        // Convert pulses to linear velocity (m/s)
        left_mps = ConvertVelocityFromPulse(received_left_vel);
        right_mps = ConvertVelocityFromPulse(received_right_vel);
        // std::cout << std::fixed << std::setprecision(3);
        // std::cout << "Converted Left Velocity (m/s): " << left_mps << "\n";
        // std::cout << "Converted Right Velocity (m/s): " << right_mps << "\n";
        updateWheelOdometry(left_mps, right_mps, odom_pub, lasttime);
        cnt_receive_odom++;
        break;
    }
    default:
    //     // Handle unknown CAN IDs
    //     std::cout << "Unknown CAN ID: 0x" << std::hex << can_id << std::dec << std::endl;
    //     cnt_receive++;
        break;
    }
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
                     << "x,y,quaternion_yaw,Left_Velocity_LOS,Right_Velocity_LOS,theta_yaw\n";
        }
        
        // Ghi dữ liệu
        csv_file << timestamp.str() << "," 
                 << imu_packages << "," 
                 << odom_packages << "," 
                 << send_packages << ","
                 << std::fixed << std::setprecision(2) << yaw_angle << ","
                 << std::setprecision(4) << left_mps/20 << ","
                 << std::setprecision(4) << right_mps/20 << ","
                 << std::setprecision(4) << x << ","
                 << std::setprecision(4) << y << ","
                 << std::setprecision(4) << quaternion_yaw << ","
                 << std::setprecision(4) << v_left/20 << ","
                 << std::setprecision(4) << v_right/20 << ","
                 << std::setprecision(4) << theta_yaw << "\n";
        csv_file.close();
        
        // ROS_INFO("Data saved: IMU=%d, Odom=%d, Send=%d, Yaw=%.2f, Left=%.4f m/s, Right=%.4f m/s, x=%.4f, y=%.4f, quat_yaw=%lf, theta_yaw=%.4f", 
        //          imu_packages, odom_packages, send_packages, yaw_angle, 
        //          left_mps, right_mps, x, y, quaternion_yaw, theta_yaw);
    } else {
        ROS_ERROR("Cannot open CSV file: %s", csv_file_path.c_str());
    }
}

void CntBytes(const ros::TimerEvent &event)
{
    // ROS_WARN("Receive IMU Packages = %d Pkg/s", cnt_receive_imu);
    // ROS_WARN("Receive Odom Packages = %d Pkg/s", cnt_receive_odom);
    // ROS_WARN("Send Packages = %d Pkg/s", cnt_send);
    
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

void TransmitSTM(ros::Publisher& odom_pub, ros::Time& lasttime)
{
    publish_yaw(yaw_angle);
    // ROS_INFO("yaw_angle = %f", yaw_angle);
    // publish_yaw(yaw_angle);
    // updateOdometry(left_mps, right_mps, x, y, odom_pub, lasttime, yaw_offset, initialized, yaw_prev, yaw_angle);

    utils::cmd_vel vel;
    vel.v_left_stm = left_mps;
    vel.v_right_stm = right_mps;
    // ROS_INFO("lef = %f", left_mps);
    // ROS_INFO("rig = %f", right_mps);
    pub_vel_stm.publish(vel);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Can_node");
    ros::NodeHandle nh;
    odom_pub = nh.advertise<nav_msgs::Odometry>("wheel_odometry", 10);
    ros::Time lasttime = ros::Time::now();
    pub = nh.advertise<utils::pose_robot>("pose_robot", 10);
    pub_vel_stm = nh.advertise<utils::cmd_vel>("Guidance", 10);
    ros::Subscriber amcl_sub = nh.subscribe("amcl_pose", 10, CallBackAmclPose); // Thêm subscriber cho amcl_pose

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

    sub = nh.subscribe("Cmd_vel", 10, CallBackVel);
    // amcl_sub = nh.subscribe("amcl_pose", 10, CallBackAMCL);
    cnt_byte = nh.createTimer(ros::Duration(10), CntBytes);
    
    // Master request 
    loopControl = nh.createTimer(
        ros::Duration(cycle_transmit),
        [&](const ros::TimerEvent&) {
            can.send(0x050, {1, 0, 0, 0, 0, 0, 0, 0}); // slave master - gửi trước
            cnt_send++;
            TransmitSTM(odom_pub, lasttime); // publish sau
        }
    );

    utils::cmd_vel cmd_vel;

    cmd_vel.v_left = 0;
    cmd_vel.v_right = 0;
    
    loopControl2 = nh.createTimer(
        ros::Duration(0.1),
        [&](const ros::TimerEvent&) {
            // auto ptr = boost::make_shared<utils::cmd_vel>(cmd_vel);
            // CallBackVel(ptr);

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
