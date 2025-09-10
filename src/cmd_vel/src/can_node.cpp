#include <cmath>
#include "can_node.h"
#include <cstdlib>
#include <string>
#include <map>
#include <vector>
#include <chrono> // For time measurement
#include <ctime>
#include <iomanip>
#define PI 3.14159265358979323846

// Global variable to store the updated value
float yaw_angle = 0;
int right_wheel_velocity = 0;
int left_wheel_velocity = 0;


static volatile int g_should_exit = 0;

WaveshareCAN can("/dev/ttyUSB1", 2000000, 2.0);

// RFID database - mapping RFID data to user info
static const std::map<std::vector<uint8_t>, std::pair<std::string, std::string>> rfid_database = {
    {{0xd2, 0x0f, 0x49, 0x2e, 0xba, 0x55, 0xaa, 0xc8}, {"HOAI PHU", "Phu"}},
    {{0xd2, 0xb1, 0x3d, 0x05, 0x5b, 0x55, 0xaa, 0xc8}, {"MINH KY", "Ky"}},
    {{0xfa, 0xdc, 0x02, 0xcd, 0xe9, 0x55, 0xaa, 0xc8}, {"QUANG DUY", "Duy"}},
    {{0xef, 0xa8, 0x98, 0x1e, 0xc1, 0x55, 0xaa, 0xc8}, {"CHI THIEN", "Thien"}},
    {{0xb6, 0x87, 0x13, 0x2b, 0x09, 0x55, 0xaa, 0xc8}, {"VAN LOI", "Loi"}},
    {{0xc2, 0xbf, 0xb0, 0x2e, 0xe3, 0x55, 0xaa, 0xc8}, {"BACH THU", "Thu"}}};

// Simple function to publish MQTT message via Python script
void publishMQTTMessage(const std::string &user_name, const std::string &mqtt_msg, const std::string &timestamp)
{
    // Don't publish if we're shutting down
    if (g_should_exit) {
        return;
    }
    
    std::string python_script = "/home/bach/robot_fablab_ws/src/MQTT/name_publisher.py";
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

// New: publish velocity (v_left, v_right) via MQTT using the Python publisher




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

void CallBackVel(const utils::cmd_vel::ConstPtr &cmd_vel)
{
    float v_left = cmd_vel->v_left;
    float v_right = cmd_vel->v_right;

    left_wheel_velocity = ConvertPulse(v_left);
    right_wheel_velocity = ConvertPulse(v_right);
}

void ControlStm(const ros::TimerEvent &event)
{
    utils::pose_robot pose;
    pose.yaw = yaw_angle; // cập nhật yaw
    pub.publish(pose);
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

// Process CAN frame (equivalent to Python's process_frame)
void process_frame(uint16_t can_id, const std::vector<uint8_t> &data)
{
    switch (can_id)
    {
    // RFID
    case 0x010:
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
    // CO2 Sensor
    case 0x011:
    {
        cnt_receive++;
        break;
    }
    // IMU Angle
    case 0x12:
    {
        // Ensure data has at least 6 bytes for roll, pitch, yaw (2 bytes each)
        if (data.size() < 6)
        {
            std::cerr << "Error: Insufficient data bytes for ID 0x012\n";
            return;
        }
        // std::cout << "ID 0x" << std::hex << can_id << std::dec << " receive IMU hex: ";
        // for (uint8_t b : data)
        // {
        //     std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)b << " ";
        // }
        // std::cout << std::dec << std::endl;
        // Extract roll, pitch, yaw as signed 16-bit integers and scale by 100.0
        // double roll = hex_to_signed(data, 0) / 100.0;  // Bytes 0-1
        // double pitch = hex_to_signed(data, 2) / 100.0; // Bytes 2-3
        float yaw = hex_to_unsigned(data, 4) / 100.0; // Bytes 4-5
        while (yaw > 180.0)
        {
            yaw -= 360.0;
        }
        while (yaw <= -180.0)
        {
            yaw += 360.0;
        }
        yaw_angle = yaw; // Update global yaw angle

        // std::cout << "Yaw: " << yaw << "\n";
        cnt_receive++;
        break;
    }
    // IMU Gyro
    case 0x013:
    {
        cnt_receive++;
        break;
    }
    // IMU Accel
    case 0x014:
    {
        cnt_receive++;
        break;
    }
    case 0x016:
    {
        // Ensure the data has exactly 8 bytes
        if (data.size() != 8)
        {
            std::cerr << "Error: Expected 8 bytes for ID 0x016, but received " << data.size() << " bytes.\n";
            return;
        }
        // Split the 8 bytes into 4 groups of 2 bytes and convert to integers
        int group1 = (data[0] << 8) | data[1]; // Combine bytes 0 and 1
        int group2 = (data[2] << 8) | data[3]; // Combine bytes 2 and 3
        int group3 = (data[4] << 8) | data[5]; // Combine bytes 4 and 5
        int group4 = (data[6] << 8) | data[7]; // Combine bytes 6 and 7

        // // Print the results
        // std::cout << "ID 0x016 received: ";
        // for (uint8_t b : data)
        // {
        //     std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)b << " ";
        // }
        // std::cout << std::dec << "\n";

        // std::cout << "Truoc: " << group1 << "\n";
        // std::cout << "Phai: " << group2 << "\n";
        // std::cout << "Trai: " << group3 << "\n";
        // std::cout << "Sau: " << group4 << "\n";
        cnt_receive++;
        break;
    }
    case 0x017:
    {
        // Ensure the data has exactly 8 bytes
        if (data.size() != 8)
        {
            std::cerr << "Error: Expected 8 bytes for ID 0x017, but received " << data.size() << " bytes.\n";
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
        cnt_receive++;
        break;
    }
    default:
        // Handle unknown CAN IDs
        // std::cout << "Unknown CAN ID: 0x" << std::hex << can_id << std::dec << std::endl;
        cnt_receive++;
        break;
    }
}

void send_vel(WaveshareCAN &can)
{
    try
    {
        // Get integer velocities
        int right_vel = right_wheel_velocity;
        int left_vel = left_wheel_velocity;
        // ROS_INFO("vel_right = %d, vel_left =%d" ,right_wheel_velocity,left_wheel_velocity);
        // Create 8-byte data array: first 4 bytes for left wheel, last 4 bytes for right wheel
        uint8_t data[8];

        // Convert left velocity to bytes (first 4 bytes)
        std::memcpy(data, &left_vel, sizeof(int));

        // Convert right velocity to bytes (last 4 bytes)
        std::memcpy(data + 4, &right_vel, sizeof(int));

        // Create data vector
        std::vector<uint8_t> velocity_data(data, data + 8);

        // Send both velocities to single ID 0x013
        can.send(0x013, velocity_data);
        cnt_send++;
        // std::cout << "Sent left velocity " << left_vel << " and right velocity " << right_vel << " to ID 0x013" << std::endl;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Invalid velocity input: " << e.what() << std::endl;
    }
}



// void CntBytes(const ros::TimerEvent &event)
// {

//     // ROS_INFO("Receive Packages = %d Pkg/s", cnt_receive);
//     cnt_receive = 0;
//     // ROS_INFO("Send Packages = %d Pkg/s", cnt_send);
//     cnt_send = 0;
// }

void TransmitSTM(const ros::TimerEvent &event)
{
    utils::pose_robot pose;
    utils::cmd_vel vel;
    // send_vel(can);
    pose.yaw = yaw_angle;
    pub.publish(pose);
    vel.v_left_stm = left_mps;
    vel.v_right_stm = right_mps;
    pub_vel_stm.publish(vel);
}

int main(int argc, char **argv)
{
    can.open();
    can.start_receive_loop(process_frame);
    ros::init(argc, argv, "Cmd_vel");
    ros::NodeHandle arg_nh("~");
    arg_nh.getParam("mode", number);
    arg_nh.getParam("calib", calib);
    arg_nh.getParam("cycle_transmit", cycle_transmit);
    ROS_INFO("mode = %d", number);

    uint8_t data[8];
    std::memcpy(data, &number, sizeof(int));
    std::vector<uint8_t> velocity_data(data, data + 8);
    can.send(0x020, velocity_data);

    ros::NodeHandle nh;
    pub = nh.advertise<utils::pose_robot>("pose_robot", 10);
    pub_vel_stm = nh.advertise<utils::cmd_vel>("Guidance", 10);
    sub = nh.subscribe("Cmd_vel", 10, CallBackVel);
    // cnt_byte = nh.createTimer(ros::Duration(1), CntBytes);
    loopControl = nh.createTimer(ros::Duration(cycle_transmit), TransmitSTM);

    ros::spin();
    return 0;
}