#pragma once

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <mutex>

// ===== encoder-only state =====
extern float x;
extern float y;
extern float wheel_yaw;
extern std::mutex wheel_odom_mutex;

// ===== API =====
double normalizeAngle(double a);

void updateWheelOdometry(float vel_left, float vel_right, float yaw, ros::Publisher& wheel_odom_pub, ros::Time& last_time);
