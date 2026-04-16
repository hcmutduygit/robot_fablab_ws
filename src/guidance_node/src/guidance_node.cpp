#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <sensor_msgs/LaserScan.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <cstdlib>
#include <vector>
#include <sstream>
#include <utility>

#include "guidance.h"
#include "pid.h"

#define PI 3.14159265358979323846

// ================= FORWARD DECLARATION =================
void rotate_to_angle(double target_angle);

// ================= ENUM =================
enum State {
    FREE = 0,
    WARNING,
    DANGER
};

// ================= GLOBAL =================
PID pid_controller;
double warning_distance_ = 0.7;
double danger_distance_ = 0.6;
State prev_state_ = FREE;
bool is_home = false;

std_msgs::Bool is_safety_stop;
std_msgs::Bool is_safety_slow;

// ================= UTILS =================
double low_pass_filter(double pre_value, double new_value, double alpha = 0.2) {
    return alpha * new_value + (1 - alpha) * pre_value;
}

float get_heading(double x1, double y1, double x2, double y2) {
    return atan2(y2 - y1, x2 - x1);
}

double normalize_angle(double angle) {
    angle = fmod(angle + PI, 2.0 * PI);
    if (angle < 0) angle += 2.0 * PI;
    return angle - PI;
}

double limit(double value, double min_val, double max_val) {
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

// ================= ROTATE CALLBACK =================
void rotateAngleCallback(const std_msgs::Float64::ConstPtr& msg) {
    double target_angle_deg = msg->data;
    double target_angle_rad = target_angle_deg * PI / 180.0;

    ROS_WARN("[RotateAngle] Nhận lệnh xoay tới góc %.2f độ (%.2f rad)",
             target_angle_deg, target_angle_rad);

    rotate_to_angle(target_angle_rad);
}

// ================= ROTATE FUNCTION =================
void rotate_to_angle(double target_angle) {
    ros::Rate rate(50);
    double error;

    do {
        error = normalize_angle(target_angle - theta);

        double angular_cmd = pid_controller.pid(error, KD, ANGULAR_SPEED);
        angular_cmd = limit(angular_cmd, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED);

        utils::cmd_vel cmd;
        cmd.v_left = -(0.0 - (angular_cmd * 0.57 / 2)) * drive;
        cmd.v_right = (0.0 + (angular_cmd * 0.57 / 2)) * drive;

        pub.publish(cmd);

        ROS_INFO("Target: %.2f, Current: %.2f, Error: %.2f",
                 target_angle, theta, error);

        rate.sleep();
        ros::spinOnce();

    } while (fabs(error) > 0.01);
}

// ================= SCAN CALLBACK =================
void CallBackScan(const sensor_msgs::LaserScan::ConstPtr& msg) {
    State state_ = FREE;
    int danger_count = 0;

    for (const auto &range : msg->ranges) {
        if (!std::isinf(range) && 0.5 <= range && range <= danger_distance_) {
            danger_count++;
        }
        if (danger_count == 30) {
            state_ = DANGER;
            break;
        }
    }

    if (state_ != prev_state_) {
        if (state_ == DANGER) {
            is_safety_stop.data = true;
        } else {
            is_safety_stop.data = false;
        }
        prev_state_ = state_;
    }
}

// ================= POSE CALLBACK =================
void CallBackPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;

    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w
    );

    double roll, pitch, yaw;
    tf::Matrix3x3 m(q);  // ⚡ FIX: dùng biến trung gian
    m.getRPY(roll, pitch, yaw);

    theta = yaw;
}

// ================= MAIN =================
int main(int argc, char **argv) {
    ros::init(argc, argv, "Guidance_node");

    ros::NodeHandle nh;
    ros::NodeHandle arg_nh("~");

    // ===== LOAD PARAMS =====
    arg_nh.getParam("linear_speed", LINEAR_SPEED);
    arg_nh.getParam("angular_speed", ANGULAR_SPEED);
    arg_nh.getParam("goal_radius", GOAL_RADIUS);
    arg_nh.getParam("cycle", cycle);
    arg_nh.getParam("linear_speed_max", MAX_LINEAR_SPEED);
    arg_nh.getParam("angular_speed_max", MAX_ANGULAR_SPEED);
    arg_nh.getParam("KD", KD);
    arg_nh.getParam("drive", drive);
    arg_nh.getParam("min_speed_linear", min_speed);
    arg_nh.getParam("direct", direct);

    ROS_INFO("Guidance node started");

    // ===== PUB / SUB =====
    pub = nh.advertise<utils::cmd_vel>("Cmd_vel", 10);

    sub_scan = nh.subscribe("scan", 10, CallBackScan);
    sub_amcl = nh.subscribe("amcl_pose", 10, CallBackPose);
    ros::Subscriber rotate_angle_sub = nh.subscribe("/robot/rotate", 10, rotateAngleCallback);

    ros::spin();
    return 0;
}