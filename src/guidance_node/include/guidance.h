#ifndef GUIDANCE_H
#define GUIDANCE_H

#include <ros/ros.h>
#include <pid.h>
#include <utils/cmd_vel.h>
#include <utils/pose_robot.h>
// using namespace utils;
#define PI 3.14159265358979323846

double x = 0.0;
double y = 0.0;
double theta = 0.0;
double roll = 0.0;
double pitch = 0.0;

double alpha_k = 0.0;
double s_k_1 =0.0;
double cross_track =0.0;
double long_track =0.0;
double delta =0.0;
double delta_max = 0.8;
double delta_min =0.5;
double target_heading;
double heading_error = 0.0;

double angular_z = 0.0;
double linear_x = 0.0;
double dist_to_goal =0.0;
double perc_dist = 0.0; 
double LINEAR_SPEED = 1.5;
double ANGULAR_SPEED = 2.0;
double GOAL_RADIUS = 1.5;
double MAX_LINEAR_SPEED = 2.0;
double MAX_ANGULAR_SPEED = 2.5;

float KP = 0.8;
double filtered_linear_x = 0.0;
double filtered_angular_z = 0.0;
int cnt = 0;
double wp[][2] = {
    {0, 0},
    {8, 0},
    {8, 6.5},
    {25, 6.5},
    {25, 24},
    {30, 24}
};

ros::Timer loopControl; 
ros::Subscriber sub;
ros::Publisher pub;

#endif