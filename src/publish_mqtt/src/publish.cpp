#include <mqtt.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

static volatile int g_should_exit = 0;

void publishMQTTVelocity(double v_left_mps, double v_right_mps)
{
    // Don't publish if we're shutting down
    if (g_should_exit) {
        return;
    }
    
    // Path to the Python velocity publisher (kept as-is)
    const std::string python_script = "/home/nvidia/robot_fablab_ws/src/MQTT/velocity_publisher.py";

    // Build command with fixed precision and timeout, run in background with process group
    std::ostringstream cmd;
    cmd.setf(std::ios::fixed);
    cmd << std::setprecision(6)
        << "setsid timeout 2 python2 \"" << python_script << "\" "
        << v_left_mps << ' ' << v_right_mps << " &";

    // std::cout << "Publishing MQTT velocity (m/s): v_left=" << std::fixed << std::setprecision(3)
    //           << v_left_mps << ", v_right=" << v_right_mps << std::endl;

    int result = std::system(cmd.str().c_str());
    if (result == 0)
    {
        // std::cout << "MQTT velocity sent successfully!" << std::endl;
    }
    else if (!g_should_exit) // Only show error if not shutting down
    {
        // std::cout << "Failed to send MQTT velocity!" << std::endl;
    }
}

void publishMQTTLocation(double x, double y, double theta) {
    // Don't publish if we're shutting down
    if (g_should_exit) {
        return;
    }
    
    const std::string python_script = "/home/nvidia/robot_fablab_ws/src/MQTT/location_publisher.py";
    std::string command = std::string("setsid timeout 2 python2 \"") + python_script + "\" " +
                         std::to_string(x) + " " + std::to_string(y) + " " + std::to_string(theta) + " &";
    
    // std::cout << "Publishing MQTT location: x=" << x << ", y=" << y << ", theta=" << theta << std::endl;
    
    int result = std::system(command.c_str());
    
}

void CallBackYaw (const utils::pose_robot::ConstPtr& msg){
        theta = msg->yaw;
}

void CallBackPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
        pos_x = msg->pose.pose.position.x;
        pos_y = msg->pose.pose.position.y;
}

void CallBackVel_stm (const utils::cmd_vel::ConstPtr& vel){
    vel_left = vel->v_left_stm;
    vel_right = vel->v_right_stm;
}

void publishMqtt(const ros::TimerEvent &event){
    publishMQTTLocation( pos_x, pos_y, theta);
    publishMQTTVelocity(static_cast<double>(vel_left), static_cast<double>(vel_right));
}

int main(int argc, char **argv){
    ros::init(argc,argv,"Mqtt");
    ros::NodeHandle nh;
    sub_yaw = nh.subscribe("pose_robot",10, CallBackYaw);
    sub_pose = nh.subscribe("amcl_pose",10, CallBackPose);
    sub_vel = nh.subscribe("Guidance",10, CallBackVel_stm);
    loopMqtt = nh.createTimer(ros::Duration(0.1), publishMqtt);
    ros::spin();
    return 0;
}