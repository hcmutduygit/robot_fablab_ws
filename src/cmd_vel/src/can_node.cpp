#include <can_node.h>
#include <cmath>

#define PI 3.14159265358979323846

int ConvertPulse(float& velocity) {
    // Convert m/s to rounds per second (assuming wheel radius is 0.1 m)
    const float wheel_radius = 100; // in millimeters
    const int pulse_per_revolution = 10000; // Assuming 360 pulses per revolution
    int pulse = static_cast<int>(pulse_per_revolution*velocity * 1000 / (2 * PI * wheel_radius)); // Convert m/s to pulses
    return pulse; // Pulse per second 
}
// Global variable to store the updated value
float yaw_angle = 0;
int right_wheel_velocity = 0;
int left_wheel_velocity = 0;

void CallBackVel(const utils::cmd_vel::ConstPtr& cmd_vel){
    float v_left = cmd_vel->v_left;
    float v_right = cmd_vel->v_right;
    // ROS_INFO("v_left= %.2f", v_left);
    // ROS_INFO("v_right= %.2f", v_right);
    left_wheel_velocity = ConvertPulse(v_left);
    right_wheel_velocity = ConvertPulse(v_right);
}

void ControlStm(const ros::TimerEvent& event){                                     
    utils::pose_robot pose;
    pose.yaw = yaw_angle; // cập nhật yaw
    pub.publish(pose);
}

// Convert two bytes to a signed 16-bit integer
int16_t hex_to_signed(const std::vector<uint8_t>& data, size_t start_idx, size_t bits = 16) {
    uint16_t value = (data[start_idx] << 8) | data[start_idx + 1];
    if (value >= (1U << (bits - 1))) {
        value -= (1U << bits);
    }
    return static_cast<int16_t>(value);
}

// Process CAN frame (equivalent to Python's process_frame)
void process_frame(uint16_t can_id, const std::vector<uint8_t>& data) {
    if (can_id == 0x012) {
        // Ensure data has at least 6 bytes for roll, pitch, yaw (2 bytes each)
        if (data.size() < 6) {
            std::cerr << "❌ Error: Insufficient data bytes for ID 0x012\n";
            return;
        }

        // Extract roll, pitch, yaw as signed 16-bit integers and scale by 100.0
        // double roll = hex_to_signed(data, 0) / 100.0;  // Bytes 0-1
        // double pitch = hex_to_signed(data, 2) / 100.0; // Bytes 2-3
        float yaw = hex_to_signed(data, 4) / 100.0;   // Bytes 4-5
        if (yaw > 180.0) {
            yaw -= 360.0;
        }
        else if (yaw <= -180.0) {
            yaw += 360.0;
        }
        yaw_angle = yaw; // Update global yaw angle
        std::cout << "Updated yaw_angle: " << yaw_angle << " degrees\n";
        // Print data in hex format
        std::cout << "ID 0x012 received: ";
        for (uint8_t b : data) {
            std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)b << " ";
        }
        std::cout << std::dec << "\n";

        // Print roll, pitch, yaw with 2 decimal places
        std::cout << std::fixed << std::setprecision(2);
        // std::cout << "Roll: " << roll << "\n";
        // std::cout << "Pitch: " << pitch << "\n";
        std::cout << "Yaw: " << yaw << "\n";
    }
    if (can_id == 0x016) {
        // Ensure the data has exactly 8 bytes
        if (data.size() != 8) {
            std::cerr << "❌ Error: Expected 8 bytes for ID 0x016, but received " << data.size() << " bytes.\n";
            return;
        }

        // Split the 8 bytes into 4 groups of 2 bytes and convert to integers
        int group1 = (data[0] << 8) | data[1]; // Combine bytes 0 and 1
        int group2 = (data[2] << 8) | data[3]; // Combine bytes 2 and 3
        int group3 = (data[4] << 8) | data[5]; // Combine bytes 4 and 5
        int group4 = (data[6] << 8) | data[7]; // Combine bytes 6 and 7

        // Print the results
        std::cout << "ID 0x016 received: ";
        for (uint8_t b : data) {
            std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)b << " ";
        }
        std::cout << std::dec << "\n";

        std::cout << "Truoc: " << group1 << "\n";
        std::cout << "Phai: " << group2 << "\n";
        std::cout << "Trai: " << group3 << "\n";
        std::cout << "Sau: " << group4 << "\n";
    }   
}

void send_vel(WaveshareCAN& can) {
    try {
        // Get integer velocities
        int right_vel = right_wheel_velocity;
        int left_vel = left_wheel_velocity;

        // Convert integers to bytes for CAN transmission
        uint8_t left_bytes[sizeof(int)];
        uint8_t right_bytes[sizeof(int)];
        std::memcpy(left_bytes, &left_vel, sizeof(int));
        std::memcpy(right_bytes, &right_vel, sizeof(int));

        // Create data vectors
        std::vector<uint8_t> left_data(left_bytes, left_bytes + sizeof(int));
        std::vector<uint8_t> right_data(right_bytes, right_bytes + sizeof(int));

        // Send left velocity to ID 0x013
        can.send(0x013, left_data);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        // Send right velocity to ID 0x014
        can.send(0x014, right_data);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        // std::cout << "Sent left velocity " << left_vel << " to ID 0x013" << std::endl;
        // std::cout << "Sent right velocity " << right_vel << " to ID 0x014" << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Invalid velocity input: " << e.what() << std::endl;
    }
}
int main(int argc, char **argv){

    WaveshareCAN can("/dev/ttyUSB0", 2000000, 2.0);
    can.open();
    can.start_receive_loop(process_frame);
    send_vel(can);
    ros::init(argc,argv,"Cmd_vel");
    ros::NodeHandle nh;
    pub = nh.advertise<utils::pose_robot>("pose_robot",10);
    sub = nh.subscribe("Cmd_vel",10,CallBackVel);
    loopControl = nh.createTimer(ros::Duration(0.02), 
        [&](const ros::TimerEvent& event) {
            utils::pose_robot pose;
            pose.yaw = yaw_angle;
            pub.publish(pose);
            send_vel(can);
        });
    ros::spin();
    return 0;
}
