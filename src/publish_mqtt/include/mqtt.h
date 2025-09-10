#ifndef MQTT_H
#define MQTT_H
#include <ros/ros.h>
#include <utils/pose_robot.h>
#include <utils/cmd_vel.h>
#define PI 3.14159265358979323846

 ros::Subscriber sub_pose;
 ros::Subscriber sub_yaw;
 ros::Subscriber sub_vel;
 ros::Timer loopMqtt;
double theta;
double vel_right, vel_left;
double pos_x,pos_y;
#endif 
