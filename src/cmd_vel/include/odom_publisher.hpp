#pragma once
#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_datatypes.h>
#include <cmath>
#include <mutex>
#define PI 3.14159265358979323846

extern float x, y, yaw, yaw_offset, yaw_prev, yaw_imu;
extern std::mutex odom_mutex;
extern bool initialized;
extern int odom_count;

inline void updateOdometry(float vel_left, float vel_right, ros::Publisher& odom_pub, ros::Time& lasttime) {
    std::lock_guard<std::mutex> lock(odom_mutex);
    odom_count += 1;
    // std::cout << "odom_count" << odom_count << "\n";
    yaw_imu = -yaw_imu * PI / 180;
    // if (!initialized) {
    //     yaw_offset = imu_yaw;  // Hướng ban đầu
    //     initialized = true;
    // }
    std::cout << "yaw_imu: " << yaw_imu << "\n";

    vel_left = -vel_left/20;
    vel_right = vel_right/20;
    vel_left = (std::abs(vel_left)<3e-3) ? 0.0 : vel_left; 
    vel_left = (std::abs(vel_right)<3e-3) ? 0.0 : vel_right; 

    ros::Time cur_time = ros::Time::now();
    float dt = (cur_time - lasttime).toSec();
    lasttime = cur_time;

    // Robot velocities
    float v = (vel_right + vel_left) / 2.0;
    // std::cout << "v: " << v << "\n";
    float omega = (vel_right - vel_left) / 0.595;
    // std::cout << "omega: " << omega << "\n";

    // // Fuse IMU yaw if available
    // float yaw_enc = yaw_prev + omega * dt;
    // yaw_enc = atan2(sin(yaw_enc), cos(yaw_enc));
    // float alpha = 0.98;  // 0.9–0.99: tin encoder nhiều hơn
    // float yaw_temp = alpha * yaw_enc + (1 - alpha) * yaw_imu;
    // yaw = atan2(sin(yaw_temp), cos(yaw_temp));
    // std::cout << "fused_yaw: " << yaw << "\n";

    // Use only yaw from encoder
    float yaw_enc = yaw_prev + omega*dt;
    yaw_enc = atan2(sin(yaw_enc), cos(yaw_enc));
    yaw = yaw_enc;
    std::cout << "yaw_encoder = " << yaw * 180 / PI << "\n";

    // yaw = yaw_imu;

    // Integrate position
    // float dyaw = (yaw_prev == 0) ? 0.001 : (yaw - yaw_prev);
    float dyaw = omega*dt;
    yaw_prev = yaw;
    const double eps = 1e-6;
    if (std::abs(omega) < eps) {
        x += v * cos(yaw) * dt;
        y += v * sin(yaw) * dt;
    } else {
        float r = v / omega;
        // std::cout << "dyaw: " << dyaw << "\n";
        x += r * (sin(yaw + dyaw) - sin(yaw));
        y += -r * (cos(yaw + dyaw) - cos(yaw));
    }

    // std::cout << "x: " << x << ", y: " << y << "\n";

    // Publish odom
    nav_msgs::Odometry odom;
    odom.header.stamp = cur_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    // std::cout << "pose.orientation: " << odom.pose.pose.orientation << "\n";
    odom.twist.twist.linear.x = v;
    odom.twist.twist.angular.z = omega;

    // covariance (optional but useful for AMCL)
    for (int i = 0; i < 36; i++) odom.pose.covariance[i] = 0.0;
    odom.pose.covariance[0]  = 0.01;
    odom.pose.covariance[7]  = 0.01; 
    odom.pose.covariance[35] = 0.02;
    odom_pub.publish(odom);

    // Broadcast TF
    static tf::TransformBroadcaster odom_broadcaster;
    geometry_msgs::TransformStamped odom_tf;
    odom_tf.header.stamp = cur_time;
    odom_tf.header.frame_id = "odom";
    odom_tf.child_frame_id = "base_footprint";
    odom_tf.transform.translation.x = x;
    odom_tf.transform.translation.y = y;
    odom_tf.transform.translation.z = 0.0;
    odom_tf.transform.rotation = tf::createQuaternionMsgFromYaw(yaw);
    odom_broadcaster.sendTransform(odom_tf);

    // std::cout << "yaw: " << yaw << "\n";
    std::cout << "v_left odom =" << vel_left << "(m/s), v_right odom =" << vel_right << "(m/s)\n";
    // std::cout << "current_time: " << cur_time << "\n";
    // std::cout << "dt: " << dt << "\n";
    // std::cout << "v: " << v << "\n";
    // std::cout << "omega: " << omega << "\n";
    // std::cout << "x: " << x << ", y: " << y << "\n";
    // std::cout << "pose.orientation: " << odom.pose.pose.orientation << "\n";
    std::cout << "---------------------------" <<"\n";

}