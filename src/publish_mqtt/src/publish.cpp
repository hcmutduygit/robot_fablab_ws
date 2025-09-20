#include <mqtt.h>
#include <signal.h>
#include <sys/wait.h>

static volatile int g_should_exit = 0;
static std::vector<pid_t> child_processes;

// Cleanup function
void cleanup_processes() {
    for (pid_t pid : child_processes) {
        kill(pid, SIGTERM);
        waitpid(pid, nullptr, WNOHANG);
    }
    child_processes.clear();
}

void signal_handler(int sig) {
    g_should_exit = 1;
    cleanup_processes();
}

void publishMQTTVelocity(double v_left_mps, double v_right_mps)
{
    if (g_should_exit) return;
    
    // Sử dụng popen thay vì system để kiểm soát tốt hơn
    const std::string python_script = "/home/nvidia/robot_fablab_ws/src/MQTT/velocity_publisher.py";
    
    std::ostringstream cmd;
    cmd.setf(std::ios::fixed);
    cmd << std::setprecision(6)
        << "timeout 1 python2 \"" << python_script << "\" "
        << v_left_mps << ' ' << v_right_mps << " 2>/dev/null";

    FILE* pipe = popen(cmd.str().c_str(), "r");
    if (pipe) {
        pclose(pipe);
    }
}

void publishMQTTLocation(double x, double y, double theta) {
    if (g_should_exit) return;
    
    const std::string python_script = "/home/nvidia/robot_fablab_ws/src/MQTT/location_publisher.py";
    
    std::ostringstream cmd;
    cmd.setf(std::ios::fixed);
    cmd << std::setprecision(6)
        << "timeout 1 python2 \"" << python_script << "\" "
        << x << " " << y << " " << theta << " 2>/dev/null";
    
    FILE* pipe = popen(cmd.str().c_str(), "r");
    if (pipe) {
        pclose(pipe);
    }
}

void CallBackYaw (const utils::pose_robot::ConstPtr& msg){
        theta = msg->yaw;
}

void CallBackPose (const utils::pose_robot::ConstPtr& msg){
        pos_x = msg->x;
        pos_y = msg->y;
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
    
    // Setup signal handlers
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    // Tăng timer lên 1.0s để giảm tải
    double publish_rate = 1.0;
    nh.getParam("publish_rate", publish_rate);
    
    sub_yaw = nh.subscribe("pose_robot", 1, CallBackYaw);
    sub_pose = nh.subscribe("pose_robot1", 1, CallBackPose);
    sub_vel = nh.subscribe("Guidance", 1, CallBackVel_stm);
    loopMqtt = nh.createTimer(ros::Duration(publish_rate), publishMqtt);
    
    ros::spin();
    
    cleanup_processes();
    return 0;
}