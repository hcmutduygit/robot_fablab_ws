#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <string>
#include <thread>
#include <chrono>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "set_initial_pose_cpp");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1, true);

    // Delay 3 giây để AMCL khởi động
    ROS_WARN("Waiting 2 seconds for AMCL to be ready...");
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // Load YAML
    // std::string yaml_file = "/home/huypi/robot_fablab_ws/src/robot_fablab/config/initial_pose.yaml";
    std::string yaml_file = "/home/nvidia/robot_fablab_ws/src/robot_fablab/config/initial_pose.yaml";
    YAML::Node config;
    try {
        config = YAML::LoadFile(yaml_file);
    } catch (const std::exception &e) {
        ROS_ERROR("Failed to load YAML: %s", e.what());
        return 1;
    }

    geometry_msgs::PoseWithCovarianceStamped pose_msg;

    // Frame id
    pose_msg.header.frame_id = config["header"]["frame_id"].as<std::string>();
    pose_msg.header.stamp = ros::Time::now();

    // Position
    pose_msg.pose.pose.position.x = config["pose"]["pose"]["position"]["x"].as<double>();
    pose_msg.pose.pose.position.y = config["pose"]["pose"]["position"]["y"].as<double>();
    pose_msg.pose.pose.position.z = config["pose"]["pose"]["position"]["z"].as<double>();

    // Orientation
    pose_msg.pose.pose.orientation.x = config["pose"]["pose"]["orientation"]["x"].as<double>();
    pose_msg.pose.pose.orientation.y = config["pose"]["pose"]["orientation"]["y"].as<double>();
    pose_msg.pose.pose.orientation.z = config["pose"]["pose"]["orientation"]["z"].as<double>();
    pose_msg.pose.pose.orientation.w = config["pose"]["pose"]["orientation"]["w"].as<double>();

    // Covariance
    const YAML::Node& cov = config["pose"]["covariance"];
    for (size_t i = 0; i < 36; ++i)
        pose_msg.pose.covariance[i] = cov[i].as<double>();

    // Publish 1 lần
    pub.publish(pose_msg);
    ROS_INFO("Published initial pose from YAML");

    ros::spinOnce();
    std::this_thread::sleep_for(std::chrono::seconds(1)); // Đảm bảo message gửi đi
    return 0;
}
