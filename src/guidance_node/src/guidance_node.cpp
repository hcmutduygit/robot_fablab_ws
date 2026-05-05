#include <guidance.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <algorithm> 
#include <math.h>
#include <string>
#include <cstdlib>
#include <vector>
#include <sstream>
#include <utility>
#include "guidance.h"
#include "pid.h"

enum State{
  FREE = 0,
  WARNING,
  DANGER
};

PID pid_controller;

double warning_distance_ = 0.7;
double danger_distance_ = 0.6;
State prev_state_ = FREE;
bool is_home = false;

// ================= FIX VARIABLES =================
bool wait_before_los = false;
ros::Time los_wait_until;
double LOS_WAIT_SECONDS = 2.0;
// ===============================================

// ⚠️ KHÔNG khai báo lại wp, cnt, has_published_arrival ở đây

std_msgs::Bool is_safety_stop;
std_msgs::Bool is_safety_slow;

double low_pass_filter(double pre_value, double new_value, double alpha = 0.2){
    return alpha * new_value + (1 - alpha) * pre_value;
}

float get_heading (double x1, double y1, double x2, double y2){
    return atan2(y2 - y1, x2 - x1);
}

double normalize_angle(double angle) {
    angle = fmod(angle + PI, 2.0 * PI);
    if (angle < 0) angle += 2.0 * PI;
    return angle - PI;
}

double limit(double value, double min_val, double max_val){
    return std::max(min_val, std::min(value, max_val));
}

void control_los(float goal_x, float goal_y, float previous_x, float previous_y) {

    alpha_k = get_heading(previous_x, previous_y, goal_x, goal_y);

    s_k_1 = (goal_x - previous_x) * cos(alpha_k) + 
            (goal_y - previous_y) * sin(alpha_k);

    cross_track = (-(x - previous_x) * sin(alpha_k) + 
                   (y - previous_y) * cos(alpha_k)) * direct;

    long_track = (x - previous_x) * cos(alpha_k) + 
                 (y - previous_y) * sin(alpha_k);

    delta = (delta_max - delta_min) * exp(-0.7 * fabs(cross_track)) + delta_min;

    static double prev_target_heading = 0.0;

    double target_heading_raw = normalize_angle(alpha_k + atan(-cross_track/delta));
    double target_heading = low_pass_filter(prev_target_heading, target_heading_raw, 0.8);

    target_heading = normalize_angle(target_heading);
    prev_target_heading = target_heading;

    heading_error = normalize_angle(target_heading - theta);

    filtered_angular_z = pid_controller.pid(heading_error, KD, ANGULAR_SPEED);
    filtered_angular_z = limit(filtered_angular_z, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED);

    dist_to_goal = fabs(s_k_1 - long_track);

    // tránh chia 0
    if (fabs(s_k_1) > 1e-6)
        perc_dist = fabs(s_k_1 - long_track) / s_k_1;
    else
        perc_dist = 0.0;

    if (fabs(heading_error) > 0.1){
        linear_x = limit(MAX_LINEAR_SPEED * exp(-3 * fabs(heading_error)), min_speed, MAX_LINEAR_SPEED);
    }
    else {
        linear_x = limit(LINEAR_SPEED * perc_dist, min_speed, MAX_LINEAR_SPEED);
    }

    // FIX FILTER (đúng thứ tự)
    filtered_angular_z = low_pass_filter(angular_z, filtered_angular_z);
    angular_z = filtered_angular_z;

    // CURVATURE LIMIT
    const double L = 0.57;
    double v = linear_x;
    double omega = angular_z;

    double kappa = (fabs(v) > 1e-6) ? omega / fabs(v) : 0.0;
    double v_limit = MAX_LINEAR_SPEED / (1.0 + (L/2.0) * fabs(kappa));

    if (fabs(v) > v_limit) {
        v = (v > 0) ? v_limit : -v_limit;
    }
    linear_x = v;
}

void tranfer_wp() {

    if (wp.size() < 2) {
        linear_x = 0.0;
        angular_z = 0.0;
        return;
    }

<<<<<<< HEAD
    if (wait_before_los) {
        if (ros::Time::now() < los_wait_until) {
            linear_x = 0.0;
            angular_z = 0.0;
            return;
        }
        wait_before_los = false;
    }

    if (cnt + 1 >= wp.size()) {
=======
    if (cnt + 1 >= (wp.size())) {
        // ROS_INFO_THROTTLE(2, "Reached final waypoint #%d - Stopping Robot", cnt);
>>>>>>> 2abc993c554c7536b44e3d903e7d9e152375bdfd
        linear_x = 0.0;
        angular_z = 0.0;
        is_home = true;
        return;
    }

    control_los(wp[cnt+1].first, wp[cnt+1].second,
                wp[cnt].first, wp[cnt].second);

    double goal_radius_eff = GOAL_RADIUS;

    if (fabs(dist_to_goal) <= goal_radius_eff) {
        cnt++;

        if (cnt + 1 >= wp.size()) {
            if (!has_published_arrival) {
                system("python /home/nvidia/robot_fablab_ws/src/MQTT/publish_arrival.py");
                has_published_arrival = true;
            }
            linear_x = 0.0;
            angular_z = 0.0;
        }
    }
}

void CallBackPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){

    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;

    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w
    );

    double roll, pitch, yaw;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    theta = yaw;
}

void CallBackWp(const utils::waypoints::ConstPtr& msg) {

    if (has_published_arrival && wp.size() > 0) {
        wp.clear();
        cnt = 0;
        has_published_arrival = false;
    }
<<<<<<< HEAD

    if (wp.empty()) {
        wp.push_back({x, y});
=======
    
    // Neu day la waypoint dau tien cua mission moi, them vi tri hien tai lam diem xuat phat
    if (wp.size() == 0) {
        double current_x = x;
        double current_y = y;
        wp.push_back({current_x, current_y});
        ROS_WARN("✓✓✓ Added STARTING position as wp[0]: (%.3f, %.3f) ✓✓✓", current_x, current_y);
        
        // Reset flag khi bat dau mission moi
        has_published_arrival = false;
        is_home = false;
>>>>>>> 2abc993c554c7536b44e3d903e7d9e152375bdfd
    }

    wp.push_back({msg->direction_x, msg->direction_y});
<<<<<<< HEAD

    if (wp.size() == 2) {
        wait_before_los = true;
        los_wait_until = ros::Time::now() + ros::Duration(LOS_WAIT_SECONDS);
=======
    ROS_INFO("✓ Received waypoint #%zu: (%.3f, %.3f)", wp.size()-1, msg->direction_x, msg->direction_y);
    
    // In ra tat ca cac waypoint hien tai
    ROS_INFO("    Total waypoints: %zu", wp.size());
    for (size_t i = 0; i < wp.size(); i++) {
        if (i == 0) {
            ROS_INFO("      wp[%zu] = (%.3f, %.3f) <- STARTING POSITION", i, wp[i].first, wp[i].second);
        } else {
            ROS_INFO("      wp[%zu] = (%.3f, %.3f) <- GOAL", i, wp[i].first, wp[i].second);
        }
>>>>>>> 2abc993c554c7536b44e3d903e7d9e152375bdfd
    }
}

void ControlVel(const ros::TimerEvent&){

    utils::cmd_vel cmd;
    tranfer_wp();

    const double L = 0.57;

    if (is_safety_stop.data) {
        cmd.v_left = 0;
        cmd.v_right = 0;
    } else {
        cmd.v_left = -(linear_x - (angular_z * L / 2)) * drive;
        cmd.v_right = (linear_x + (angular_z * L / 2)) * drive;
    }

<<<<<<< HEAD
    if (is_home || (wait_before_los && ros::Time::now() < los_wait_until)) {
        return;
    }

    pub.publish(cmd);
=======
    // cmd.v_left = -(linear_x - (angular_z * 0.57/ 2)) * drive;
    // cmd.v_right = (linear_x + (angular_z * 0.57/ 2)) * drive;
   
    // ROS_INFO("v_left = %.2f, v_right = %.2f, ANGULAR = %.2f", cmd.v_left, cmd.v_right, angular_z);
    if (!is_home){
        pub.publish(cmd);
    }
>>>>>>> 2abc993c554c7536b44e3d903e7d9e152375bdfd
}

int main(int argc, char **argv){

    ros::init(argc,argv,"Guidance_node");

    ros::NodeHandle arg_nh("~");

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

    arg_nh.getParam("los_wait_seconds", LOS_WAIT_SECONDS);

    ros::NodeHandle nh;

    pub = nh.advertise<utils::cmd_vel>("Cmd_vel", 10);

    sub_amcl = nh.subscribe("amcl_pose", 10, CallBackPose);
    sub_wp = nh.subscribe("waypoints", 100, CallBackWp);

    loopControl = nh.createTimer(ros::Duration(cycle), ControlVel);

    ros::spin();
    return 0;
}