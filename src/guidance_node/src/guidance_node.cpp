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
double warning_distance_ = 0.75;
double danger_distance_ = 0.6;
State prev_state_ = FREE;

std_msgs::Bool is_safety_stop;
std_msgs::Bool is_safety_slow;

double low_pass_filter(double pre_value, double new_value, double alpha = 0.2){
    return alpha * new_value + (1 - alpha) * pre_value;
}

float get_heading (double x1, double y1, double x2, double y2){
    return atan2( y2 - y1, x2 - x1);
}

double normalize_angle(double angle) {
    angle = fmod(angle + PI, 2.0 * PI);
    if (angle < 0)
        angle += 2.0 * PI;
    return angle - PI;
}

double limit(double value, double min_val, double max_val)
{
    if (value < min_val) return min_val;
    else if (value > max_val) return max_val;
    else return value;
}

void control_los(float goal_x, float goal_y, float previous_x, float previous_y) {
    alpha_k = get_heading(previous_x, previous_y, goal_x, goal_y);
    s_k_1 = (goal_x - previous_x) * cos(alpha_k) + (goal_y - previous_y) * sin(alpha_k); 

    cross_track = (-(x - previous_x) * sin(alpha_k) + (y - previous_y) * cos(alpha_k)) * direct;
    long_track = (x - previous_x) * cos(alpha_k) + (y - previous_y) * sin(alpha_k);
    delta = (delta_max - delta_min) * exp(-0.7 * pow(cross_track, 2)) + delta_min;

    target_heading = normalize_angle(alpha_k + atan(-cross_track/delta));
    heading_error  = normalize_angle(target_heading - theta);

    // ROS_INFO("CrossTrack = %.2f, LongTrack = %.2f, HeadingDesire = %.2f, HeadingErr = %.2f, Theta = %.2f",cross_track, long_track,target_heading,heading_error,theta);

    filtered_angular_z = pid_controller.pid(heading_error, KD, ANGULAR_SPEED);
    // filtered_angular_z = limit(filtered_angular_z, -MAX_ANGULAR_SPEED, ANGULAR_SPEED);
    // std::cout << "filtered_angular_z = " << filtered_angular_z << "\n";
    filtered_angular_z = limit(filtered_angular_z, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED);

    dist_to_goal = abs(s_k_1 - long_track);
    perc_dist = abs(s_k_1 - long_track)/s_k_1;

    if (abs(heading_error) > 0.1){
        // linear_x = MAX_LINEAR_SPEED/2;
        linear_x = limit(MAX_LINEAR_SPEED * exp(-3 * abs(heading_error)), min_speed, MAX_LINEAR_SPEED);
    }
    else {
        linear_x = limit(LINEAR_SPEED*perc_dist, min_speed, MAX_LINEAR_SPEED);
    }
    filtered_angular_z = low_pass_filter(angular_z, filtered_angular_z);
    // filtered_angular_z = low_pass_filter(filtered_angular_z, angular_z);
    angular_z = filtered_angular_z;
}

void tranfer_wp() {
    ROS_INFO_THROTTLE(2, "=== tranfer_wp DEBUG === wp.size=%zu, cnt=%d, robot=(%.2f, %.2f)", wp.size(), cnt, x, y);
    
    if (wp.size() == 0) {
        ROS_WARN_THROTTLE(5, "No waypoints received yet!");
        linear_x = 0.0;
        angular_z = 0.0;
        return;
    }
    
    if (wp.size() == 1) {
        ROS_WARN_THROTTLE(5, "Only 1 waypoint! Need at least 2 for navigation. Current wp[0]=(%.2f, %.2f)", 
                         wp[0].first, wp[0].second);
        linear_x = 0.0;
        angular_z = 0.0;
        return;
    }
    
    if (cnt + 1 >= (wp.size())) {
        // ROS_INFO_THROTTLE(2, "Reached final waypoint #%d - Stopping Robot", cnt);
        linear_x = 0.0;
        angular_z = 0.0;
    }
    else {
        // ROS_INFO_THROTTLE(2, "Moving to waypoint #%d: (%.2f, %.2f) from wp[%d]=(%.2f, %.2f)", 
        //                  cnt+1, wp[cnt+1].first, wp[cnt+1].second, 
        //                  cnt, wp[cnt].first, wp[cnt].second);
        control_los(wp[cnt+1].first, wp[cnt+1].second, wp[cnt].first, wp[cnt].second);
    }

    if (dist_to_goal <= GOAL_RADIUS) {
        // ROS_INFO("Reached waypoint #%d: (%.2f, %.2f) ✓✓✓", cnt+1, wp[cnt+1].first, wp[cnt+1].second);
        cnt +=1;
        
        // Neu da den waypoint cuoi cung
        if (cnt + 1 >= wp.size()) {
            // Chi publish MQTT 1 lan duy nhat
            if (!has_published_arrival) {
                ROS_WARN("========================================");
                ROS_WARN("✓✓✓ REACHED FINAL DESTINATION ✓✓✓");
                ROS_WARN("Publishing arrival status to MQTT...");
                ROS_WARN("========================================");
                
                // Goi Python script de publish MQTT arrival
                std::string script_path = "python /home/nvidia/robot_fablab_ws/src/MQTT/publish_arrival.py";
                int result = system(script_path.c_str());
                
                if (result == 0) {
                    ROS_WARN("Successfully published arrival status to MQTT");
                } else {
                    ROS_ERROR("Failed to publish arrival status (exit code: %d)", result);
                }
                
                // Danh dau da publish de khong spam
                has_published_arrival = true;
                
                ROS_WARN("========================================");
                ROS_WARN("🔄 Robot stopped - Ready for new waypoints!");
                ROS_WARN("Current position: (%.3f, %.3f)", x, y);
                ROS_WARN("========================================");
            }
            
            // Dung robot
            linear_x = 0.0;
            angular_z = 0.0;
        }
    }
}

void CallBackScan(const sensor_msgs::LaserScan::ConstPtr& msg){
    State state_ = FREE;

    for (const auto &range : msg->ranges)
    {
        if (!std::isinf(range) && 0.5 < range && range <= warning_distance_)
        {
            state_ = State::WARNING;
            if (0.5 < range && range <= danger_distance_)
            {
                state_ = State::DANGER;
                break;
            }
        }
    }

    if (state_ != prev_state_)
    {
        if (state_ == State::WARNING)
        {
            is_safety_stop.data = false;
            is_safety_slow.data = true;

        }
        else if (state_ == State::DANGER)
        {
            is_safety_stop.data = true;
            is_safety_slow.data = false;
        }
        else if (state_ == State::FREE)
        {
            is_safety_stop.data = false;
            is_safety_slow.data = false;
        }

        prev_state_ = state_;
    }
}

void CallBackPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;
    double orientation_x = msg->pose.pose.orientation.x;
    double orientation_y = msg->pose.pose.orientation.y;
    double orientation_z = msg->pose.pose.orientation.z;
    double orientation_w = msg->pose.pose.orientation.w;

    tf::Quaternion q(orientation_x, orientation_y, orientation_z, orientation_w);
    double roll, pitch, amcl_yaw;
    tf::Matrix3x3(q).getRPY(roll, pitch, amcl_yaw);
    theta = amcl_yaw;
}

void CallBackWp(const utils::waypoints::ConstPtr& msg) {
    // Kiem tra neu da hoan thanh mission truoc do (da den waypoint cuoi) 
    // thi reset tat ca de bat dau mission moi
    if (has_published_arrival && wp.size() > 0) {
        ROS_WARN("========================================");
        ROS_WARN("🆕 NEW MISSION DETECTED - Resetting all waypoints!");
        ROS_WARN("========================================");
        wp.clear();
        cnt = 0;
        has_published_arrival = false;
        linear_x = 0.0;
        angular_z = 0.0;
    }
    
    // Neu day la waypoint dau tien cua mission moi, them vi tri hien tai lam diem xuat phat
    if (wp.size() == 0) {
        double current_x = x;
        double current_y = y;
        wp.push_back({current_x, current_y});
        ROS_WARN("✓✓✓ Added STARTING position as wp[0]: (%.3f, %.3f) ✓✓✓", current_x, current_y);
        
        // Reset flag khi bat dau mission moi
        cnt = 0;
        has_published_arrival = false;
    }
    
    wp.push_back({msg->direction_x, msg->direction_y});
    ROS_INFO("✓ Received waypoint #%zu: (%.3f, %.3f)", wp.size()-1, msg->direction_x, msg->direction_y);
    
    // In ra tat ca cac waypoint hien tai
    ROS_INFO("    Total waypoints: %zu", wp.size());
    for (size_t i = 0; i < wp.size(); i++) {
        if (i == 0) {
            ROS_INFO("      wp[%zu] = (%.3f, %.3f) <- STARTING POSITION", i, wp[i].first, wp[i].second);
        } else {
            ROS_INFO("      wp[%zu] = (%.3f, %.3f) <- GOAL", i, wp[i].first, wp[i].second);
        }
    }
}

void ControlVel(const ros::TimerEvent& event){
    utils::cmd_vel cmd;
    tranfer_wp();

    if (is_safety_stop.data == true && is_safety_slow.data == true){
        cmd.v_left = 0;
        cmd.v_right = 0;
    }
    else if (is_safety_stop.data == false && is_safety_slow.data == true){
        cmd.v_left = 0.75 * (linear_x - (angular_z * 0.57/ 2)) * drive/2;
        cmd.v_right = 0.75 * (linear_x + (angular_z * 0.57/ 2)) * drive/2;
    }
    else {
        cmd.v_left = (linear_x - (angular_z * 0.57/ 2)) * drive;
        cmd.v_right = (linear_x + (angular_z * 0.57/ 2)) * drive;
    }
   
    // ROS_INFO("v_left = %.2f, v_right = %.2f, ANGULAR = %.2f", cmd.v_left, cmd.v_right, angular_z);
    pub.publish(cmd);
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



    ROS_INFO("Linear_speed_max = %.2f, Angular_speed_max= %.2f, goal_radius= %.2f. KD = %.2f",MAX_LINEAR_SPEED,MAX_ANGULAR_SPEED,GOAL_RADIUS,KD);
    ros::NodeHandle nh;

    pub = nh.advertise<utils::cmd_vel>("Cmd_vel", 10);
    sub_scan = nh.subscribe("scan", 10, CallBackScan);
    sub_amcl = nh.subscribe("amcl_pose", 10, CallBackPose); 

    // ========================================================================
    // CHON CHE DO DOC WAYPOINTS
    // ========================================================================
    // waypoint_mode = 0: Doc tu PARAM (cach cu)
    // waypoint_mode = 1: Doc tu TOPIC (cach moi - MQTT)
    int waypoint_mode = 1; // Mac dinh dung topic
    arg_nh.getParam("waypoint_mode", waypoint_mode);

    if (waypoint_mode == 0) {
        // ========================================================================
        // TRUONG HOP 1: Doc waypoints tu LAUNCH FILE PARAMS - Cach cu
        // ========================================================================
        ROS_WARN("=== WAYPOINT MODE: Reading from LAUNCH FILE PARAMS ===");
        
        std::string waypoints_x_str, waypoints_y_str;
        if (arg_nh.getParam("waypoints_x", waypoints_x_str) && arg_nh.getParam("waypoints_y", waypoints_y_str)) {
            
            // THEM DIEM XUAT PHAT (vi tri hien tai cua robot) - GIONG NHU FILE CU
            double current_x = x;
            double current_y = y;
            wp.push_back({current_x, current_y});
            ROS_WARN("✓ Added STARTING position as wp[0]: (%.3f, %.3f)", current_x, current_y);
            
            std::vector<double> waypoints_x_temp;
            std::vector<double> waypoints_y_temp;
            
            std::stringstream ss_x(waypoints_x_str);
            std::stringstream ss_y(waypoints_y_str);
            std::string segment;

            while(std::getline(ss_x, segment, ',')) {
                waypoints_x_temp.push_back(std::stod(segment));
            }

            while(std::getline(ss_y, segment, ',')) {
                waypoints_y_temp.push_back(std::stod(segment));
            }

            if (waypoints_x_temp.size() == waypoints_y_temp.size()) {
                for (size_t i = 0; i < waypoints_x_temp.size(); ++i) {
                    wp.push_back({waypoints_x_temp[i], waypoints_y_temp[i]});
                    // ROS_INFO("Loaded waypoint #%zu from param: (%.3f, %.3f)", wp.size()-1, waypoints_x_temp[i], waypoints_y_temp[i]);
                }
            }
            
            ROS_WARN("Total waypoints loaded: %zu (including starting position)", wp.size());
        } else {
            ROS_WARN("waypoint_mode=0 but no waypoints_x/waypoints_y params found!");
        }
    } 
    else if (waypoint_mode == 1) {
        // ========================================================================
        // TRUONG HOP 2: Doc waypoints tu TOPIC (MQTT) - Cach moi
        // ========================================================================
        ROS_WARN("=== WAYPOINT MODE: Reading from TOPIC (MQTT) ===");
        
        std::string waypoints_topic = "waypoints";
        arg_nh.getParam("waypoints_topic", waypoints_topic);
        sub_wp = nh.subscribe(waypoints_topic, 100, CallBackWp);
        ROS_INFO("Subscribed to waypoints topic: %s", waypoints_topic.c_str());
        ROS_INFO("Waiting for waypoints from topic...");
    }
    else {
        ROS_ERROR("Invalid waypoint_mode=%d! Use 0 (param) or 1 (topic)", waypoint_mode);
    }
    
    // ========================================================================
    // Start control loop
    // ========================================================================
    loopControl = nh.createTimer(ros::Duration(cycle), ControlVel);
    
    ROS_INFO("=== Guidance node ready ===");
    ros::spin();
    return 0;

}