#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/tf.h>
#include <math.h>
#include <vector>
#include <string>
#include <sstream>
#include <cstdlib>
#include "pid.h"
#include "utils/waypoints.h"
#include "utils/cmd_vel.h"

#define PI 3.14159265358979323846

enum State {
    FREE = 0,
    WARNING,
    DANGER
};

// ===== PID controller =====
PID pid_controller;

// ===== Robot / Control variables =====
double x = 0.0, y = 0.0, theta = 0.0;
double linear_x = 0.0, angular_z = 0.0, filtered_angular_z = 0.0;
double target_angle = 0.0, heading_error = 0.0;

// ===== Waypoints =====
std::vector<std::pair<double,double>> wp;
size_t cnt = 0;
bool has_published_arrival = false;
bool is_home = false;

// ===== Safety / Laser =====
double warning_distance_ = 0.7;
double danger_distance_ = 0.6;
State prev_state_ = FREE;
std_msgs::Bool is_safety_stop;
std_msgs::Bool is_safety_slow;

// ===== Control parameters =====
double LINEAR_SPEED = 0.3;
double MAX_LINEAR_SPEED = 0.5;
double ANGULAR_SPEED = 0.5;
double MAX_ANGULAR_SPEED = 1.0;
double KD = 0.5;
double GOAL_RADIUS = 0.2;
double min_speed = 0.05;
int direct = 1;
double drive = 1.0;
double cycle = 0.05;

// ===== LOS / cross track =====
double alpha_k = 0.0;
double s_k_1 = 0.0;
double cross_track = 0.0;
double long_track = 0.0;
double delta = 0.0;
double delta_min = 0.1, delta_max = 1.0;
double perc_dist = 0.0;
double dist_to_goal = 0.0;

// ===== ROS publishers/subscribers =====
ros::Publisher pub;
ros::Subscriber sub_scan;
ros::Subscriber sub_amcl;
ros::Subscriber sub_wp;
ros::Timer loopControl;

// ===== Helper functions =====
double normalize_angle(double angle) {
    angle = fmod(angle + PI, 2.0 * PI);
    if (angle < 0) angle += 2.0 * PI;
    return angle - PI;
}

double limit(double value, double min_val, double max_val) {
    if (value < min_val) return min_val;
    else if (value > max_val) return max_val;
    else return value;
}

double low_pass_filter(double pre_value, double new_value, double alpha = 0.2) {
    return alpha * new_value + (1 - alpha) * pre_value;
}

float get_heading(double x1, double y1, double x2, double y2) {
    return atan2(y2 - y1, x2 - x1);
}

// ===== LOS control =====
void control_los(float goal_x, float goal_y, float previous_x, float previous_y) {
    alpha_k = get_heading(previous_x, previous_y, goal_x, goal_y);
    s_k_1 = (goal_x - previous_x) * cos(alpha_k) + (goal_y - previous_y) * sin(alpha_k);
    cross_track = (-(x - previous_x) * sin(alpha_k) + (y - previous_y) * cos(alpha_k)) * direct;
    long_track = (x - previous_x) * cos(alpha_k) + (y - previous_y) * sin(alpha_k);
    delta = (delta_max - delta_min) * exp(-0.7 * pow(cross_track, 2)) + delta_min;

    target_angle = normalize_angle(alpha_k + atan(-cross_track/delta));
    heading_error = normalize_angle(target_angle - theta);
    filtered_angular_z = pid_controller.pid(heading_error, KD, ANGULAR_SPEED);
    filtered_angular_z = limit(filtered_angular_z, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED);

    dist_to_goal = fabs(s_k_1 - long_track);
    perc_dist = fabs(s_k_1 - long_track) / s_k_1;

    if (fabs(heading_error) > 0.1)
        linear_x = limit(MAX_LINEAR_SPEED * exp(-3 * fabs(heading_error)), min_speed, MAX_LINEAR_SPEED);
    else
        linear_x = limit(LINEAR_SPEED * perc_dist, min_speed, MAX_LINEAR_SPEED);

    filtered_angular_z = low_pass_filter(angular_z, filtered_angular_z);
    angular_z = filtered_angular_z;
}

// ===== Angle control =====
void angle_control() {
    heading_error = normalize_angle(target_angle - theta);
    filtered_angular_z = pid_controller.pid(heading_error, KD, ANGULAR_SPEED);
    filtered_angular_z = limit(filtered_angular_z, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED);
    filtered_angular_z = low_pass_filter(angular_z, filtered_angular_z);
    angular_z = filtered_angular_z;
}

// ===== Waypoint transfer =====
void tranfer_wp() {
    if (wp.size() == 0) {
        linear_x = 0.0;
        angular_z = 0.0;
        return;
    }

    if (cnt + 1 >= wp.size()) {
        linear_x = 0.0;
        angular_z = 0.0;
        is_home = true;
    } else {
        control_los(wp[cnt+1].first, wp[cnt+1].second, wp[cnt].first, wp[cnt].second);
    }

    if (dist_to_goal <= GOAL_RADIUS) cnt += 1;
}

// ===== Laser callback =====
void CallBackScan(const sensor_msgs::LaserScan::ConstPtr& msg) {
    State state_ = FREE;
    int danger_count = 0;

    for (const auto &range : msg->ranges) {
        if (!std::isinf(range) && 0.5 <= range && range <= danger_distance_) danger_count++;
        if (danger_count >= 30) {
            state_ = DANGER;
            break;
        }
    }

    if (state_ != prev_state_) {
        is_safety_stop.data = (state_ == DANGER);
        prev_state_ = state_;
    }
}

// ===== Pose callback =====
void CallBackPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;

    tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                     msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    double roll, pitch;
    tf::Matrix3x3(q).getRPY(roll, pitch, theta);
}

// ===== Waypoints callback =====
void CallBackWp(const utils::waypoints::ConstPtr& msg) {
    if (has_published_arrival && wp.size() > 0) {
        wp.clear(); cnt = 0; has_published_arrival = false; linear_x = 0; angular_z = 0;
    }
    if (wp.size() == 0) wp.push_back({x, y});  // starting position
    wp.push_back({msg->direction_x, msg->direction_y});
}

// ===== Control loop =====
void ControlVel(const ros::TimerEvent&) {
    utils::cmd_vel cmd;

    static bool test_pd = false;
    static double initial_theta = 0.0;
    static double prev_angle_deg = 0.0;
    double test_angle_deg = 0.0;

    ros::NodeHandle nh_priv("~");
    bool has_param = nh_priv.getParam("test_angle_deg", test_angle_deg);
    if (has_param && test_angle_deg != prev_angle_deg) { test_pd = true; initial_theta = theta; prev_angle_deg = test_angle_deg; }

    if (has_param && test_pd) {
        target_angle = initial_theta + test_angle_deg * PI / 180.0;
        angle_control();
        linear_x = 0.0;
        if (fabs(normalize_angle(target_angle - theta)) < 0.5 * PI / 180.0) test_pd = false;
    } else {
        tranfer_wp();
    }

    cmd.v_left = -(linear_x - angular_z * 0.57/2) * drive;
    cmd.v_right = (linear_x + angular_z * 0.57/2) * drive;
    if (!is_home) pub.publish(cmd);
}

// ===== Main =====
int main(int argc, char** argv) {
    ros::init(argc, argv, "Guidance_node");
    ros::NodeHandle nh, arg_nh("~");

    // Load parameters
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
    arg_nh.getParam("target_angle", target_angle);

    pub = nh.advertise<utils::cmd_vel>("Cmd_vel", 10);
    sub_scan = nh.subscribe("scan", 10, CallBackScan);
    sub_amcl = nh.subscribe("amcl_pose", 10, CallBackPose);
    sub_wp = nh.subscribe("waypoints", 100, CallBackWp);

    loopControl = nh.createTimer(ros::Duration(cycle), ControlVel);

    ROS_INFO("=== Guidance node ready ===");
    ros::spin();
    return 0;
}