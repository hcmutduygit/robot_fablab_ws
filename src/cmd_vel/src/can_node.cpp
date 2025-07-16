#include <can_node.h>

// Global variable to store the updated value
float yaw_angle = 0;
float right_wheel_velocity = 0;
float left_wheel_velocity = 0;

void CallBackVel(const utils::cmd_vel::ConstPtr& cmd_vel){
    float v_left = cmd_vel->v_left;
    float v_right = cmd_vel->v_right;
    //  ROS_INFO("v_left= %.2f", v_left);
    //  ROS_INFO("v_right= %.2f", v_right);
    left_wheel_velocity = v_left;
    right_wheel_velocity = v_right;
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
}

void send_vel(WaveshareCAN& can) {
    // std::cout << "Enter right wheel velocity: ";
    // std::string right_input;
    // std::getline(std::cin, right_input);
    
    // std::cout << "Enter left wheel velocity: ";
    // std::string left_input;
    // std::getline(std::cin, left_input);
    
    try {
        float right_vel = right_wheel_velocity;
        float left_vel = left_wheel_velocity;
        
        // std::cout << "Right velocity: " << right_vel << ", Left velocity: " << left_vel << std::endl;
        
        // Convert float to bytes for CAN transmission
        uint8_t left_bytes[4];
        uint8_t right_bytes[4];
        std::memcpy(left_bytes, &left_vel, sizeof(float));
        std::memcpy(right_bytes, &right_vel, sizeof(float));
        
        // Create data vectors
        std::vector<uint8_t> left_data(left_bytes, left_bytes + 4);
        std::vector<uint8_t> right_data(right_bytes, right_bytes + 4);
        
        // Send left velocity to ID 0x013
        can.send(0x013, left_data);
        std::cout << "Sent left velocity " << left_vel << " to ID 0x013" << std::endl;
        
        // Send right velocity to ID 0x014
        can.send(0x014, right_data);
        std::cout << "Sent right velocity " << right_vel << " to ID 0x014" << std::endl;
        
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
