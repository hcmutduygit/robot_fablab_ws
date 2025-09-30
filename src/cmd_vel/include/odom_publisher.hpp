#pragma once
#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_datatypes.h>
#include <cmath>
#include <mutex>

extern tf::TransformBroadcaster odom_broadcaster;
extern float x, y, yaw;
extern std::mutex odom_mutex;

inline void updateOdometry(float v_left, float v_right, float& x, float& y, ros::Publisher& odom_pub, ros::Time& last_time, float imu_yaw = NAN) {
    std::lock_guard<std::mutex> lock(odom_mutex);

    ros::Time cur_time = ros::Time::now();
    double dt = (cur_time - last_time).toSec();
    if (dt <= 0.0) return;
    if (dt > 0.2) dt = 0.2; // clamp to avoid huge jumps
    last_time = cur_time;

    // Robot velocities
    double v = (v_right + v_left) / 2.0;
    double omega = (v_right - v_left) / 0.72;

    // Fuse IMU yaw if available
    if (!std::isnan(imu_yaw)) {
        double alpha = 0.9;
        double yaw_pred = yaw + omega * dt;
        yaw = alpha * yaw_pred + (1.0 - alpha) * imu_yaw;
    } else {
        yaw += omega * dt;
    }

    // Integrate position
    const double eps = 1e-6;
    if (std::abs(omega) < eps) {
        x += v * cos(yaw) * dt;
        y += v * sin(yaw) * dt;
    } else {
        double r = v / omega;
        double dyaw = omega * dt;
        x += r * (sin(yaw + dyaw) - sin(yaw));
        y += -r * (cos(yaw + dyaw) - cos(yaw));
    }

    // Publish odom
    nav_msgs::Odometry odom;
    odom.header.stamp = cur_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    odom.twist.twist.linear.x = v;
    odom.twist.twist.angular.z = omega;

    // covariance (optional but useful for AMCL)
    for (int i = 0; i < 36; i++) odom.pose.covariance[i] = 0.0;
    odom.pose.covariance[0]  = 0.01;
    odom.pose.covariance[7]  = 0.01;
    odom.pose.covariance[35] = 0.02;
    odom_pub.publish(odom);

    // Broadcast TF
    geometry_msgs::TransformStamped odom_tf;
    odom_tf.header.stamp = cur_time;
    odom_tf.header.frame_id = "odom";
    odom_tf.child_frame_id = "base_footprint";
    odom_tf.transform.translation.x = x;
    odom_tf.transform.translation.y = y;
    odom_tf.transform.translation.z = 0.0;
    odom_tf.transform.rotation = tf::createQuaternionMsgFromYaw(yaw);
    odom_broadcaster.sendTransform(odom_tf);
}
