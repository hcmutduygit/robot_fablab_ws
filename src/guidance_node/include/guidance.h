#ifndef GUIDANCE_H
#define GUIDANCE_H

#include <ros/ros.h>
#include <pid.h>
#include <utils/cmd_vel.h>
#include <utils/pose_robot.h>
#include <utils/waypoints.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/LaserScan.h>

// using namespace utils;
#define PI 3.14159265358979323846

double x = 0.0;
double y = 0.0;
double roll = 0.0;
double pitch = 0.0;

double alpha_k = 0.0;
double s_k_1 = 0.0;
double cross_track = 0.0;
double long_track = 0.0;
double delta = 0.0;
double delta_max = 0.8;
double delta_min = 0.5;
double target_heading;
double heading_error = 0.0;

double angular_z = 0.0;
double linear_x = 0.0;
double dist_to_goal = 0.0;
double perc_dist = 0.0; 
double LINEAR_SPEED;
double ANGULAR_SPEED;
double GOAL_RADIUS;
double GOAL_RADIUS_HOME;
double MAX_LINEAR_SPEED, min_speed ;
double MAX_ANGULAR_SPEED;
float cycle;
float KD;
double filtered_linear_x = 0.0;
double filtered_angular_z = 0.0;
int cnt = 0;
int drive, direct;
float theta;
std::vector<std::pair<double, double>> wp;
std::vector<bool> wp_is_home;
bool has_published_arrival = false;  // Flag de tranh spam MQTT arrival



// double wp[][2] = {
//     {13.1728434585, 11.4516193091},
//     {7.69544647905, 17.9154257326}
// };
ros::Timer loopControl; 
ros::Subscriber sub_scan;
ros::Subscriber sub_wp;
ros::Publisher pub;
ros::Subscriber sub_amcl;
ros::Publisher zones_pub;
#endif