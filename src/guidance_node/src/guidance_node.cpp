#include <guidance.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
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

PID pid_controller;

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

    ROS_INFO("CrossTrack = %.2f, LongTrack = %.2f, HeadingDesire = %.2f, HeadingErr = %.2f, Theta = %.2f",cross_track, long_track,target_heading,heading_error,theta);

    filtered_angular_z = pid_controller.pid(heading_error, KP, ANGULAR_SPEED);
    // filtered_angular_z = limit(filtered_angular_z, -MAX_ANGULAR_SPEED, ANGULAR_SPEED);
    filtered_angular_z = limit(filtered_angular_z, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED);

    dist_to_goal = abs(s_k_1 - long_track);
    perc_dist = abs(s_k_1 - long_track)/s_k_1;

    if (abs(heading_error) > 0.1){
        linear_x = MAX_LINEAR_SPEED/2;
        // linear_x = limit(MAX_LINEAR_SPEED * exp(-3 * abs(heading_error)), min_speed, MAX_LINEAR_SPEED);
    }
    else {
        linear_x = limit(LINEAR_SPEED*perc_dist, min_speed, MAX_LINEAR_SPEED);
    }
    filtered_angular_z = low_pass_filter(angular_z, filtered_angular_z);
    // filtered_angular_z = low_pass_filter(filtered_angular_z, angular_z);
    angular_z = filtered_angular_z;
}

void tranfer_wp() {
    if (cnt + 1 >= (wp.size())) {
        // ROS_INFO ("Stopping Robot");
        linear_x = 0.0;
        angular_z = 0.0;
    }
    else {
        control_los(wp[cnt+1].first, wp[cnt+1].second, wp[cnt].first, wp[cnt].second);
    }

    if (dist_to_goal <= GOAL_RADIUS) {
        // ROS_INFO("Reached wp(%.2f, %.2f)",wp[cnt+1].first,wp[cnt+1].second);
        cnt +=1;
    }
}

void CallBackYaw (const utils::pose_robot::ConstPtr& msg){
    // x = msg->x;
    // y = msg->y;
    float theta_temp = (-(msg->yaw)*PI)/180;
    if ((theta_temp + 2.615) > PI) theta = theta_temp + 2.615 - 2*PI;
    else theta = theta_temp + 2.615;
}

void CallBackPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;
    double orientation_x = msg->pose.pose.orientation.x;
    double orientation_y = msg->pose.pose.orientation.y;
    double orientation_z = msg->pose.pose.orientation.z;
    double orientation_w = msg->pose.pose.orientation.w;

    tf::Quaternion q(orientation_x, orientation_y, orientation_z, orientation_w);
    double roll, pitch, yaw;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    // theta = yaw;
}

void ControlVel(const ros::TimerEvent& event){
    utils::cmd_vel cmd;
    tranfer_wp();
    double v_left = linear_x - (angular_z * 0.513 / 2);
    double v_right = linear_x + (angular_z * 0.513 / 2);
    
    // cmd.linear.x  = linear_x;        
    // cmd.angular.z = angular_z; 

    cmd.v_left = -v_left*drive; 
    cmd.v_right = v_right*drive; 
   
    ROS_INFO("v_left = %.2f, v_right = %.2f, ANGULAR = %.2f", cmd.v_left, cmd.v_right, angular_z);
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
    arg_nh.getParam("KP", KP);
    arg_nh.getParam("drive", drive);
    arg_nh.getParam("min_speed_linear", min_speed);
    arg_nh.getParam("direct", direct);



    ROS_INFO("Linear_speed_max = %.2f, Angular_speed_max= %.2f, goal_radius= %.2f. KP = %.2f",MAX_LINEAR_SPEED,MAX_ANGULAR_SPEED,GOAL_RADIUS,KP);
    ros::NodeHandle nh;

    pub = nh.advertise<utils::cmd_vel >("Cmd_vel", 1);
    sub = nh.subscribe("pose_robot", 10, CallBackYaw);
    sub_amcl = nh.subscribe("amcl_pose", 10, CallBackPose); //theo topic
    loopControl = nh.createTimer(ros::Duration(cycle), ControlVel);
    std::string waypoints_x_str, waypoints_y_str;
    
    if (arg_nh.getParam("waypoints_x", waypoints_x_str) && arg_nh.getParam("waypoints_y", waypoints_y_str)) {
        
        double current_x = x;
        double current_y = y;
        // wp.push_back({current_x, current_y});

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
            }
        } 
        // ROS_INFO("Đã đọc được %zu waypoint.", wp.size());
        // for (size_t i = 0; i < wp.size(); ++i) {
        //     ROS_INFO("Waypoint %zu: (%f, %f)", i, wp[i].first, wp[i].second);
        // }
    } 
    ros::spin();
    return 0;

}