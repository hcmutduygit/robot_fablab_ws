#pragma once
#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_datatypes.h>
#include <cmath>
#include <mutex>
#define PI 3.14159265358979323846

extern float x, y, yaw, yaw_offset, yaw_prev;
extern std::mutex odom_mutex;
extern bool initialized;
extern int odom_count;

inline void updateOdometry(float v_left, float v_right, float& x, float& y, ros::Publisher& odom_pub, ros::Time& lasttime, float& yaw_offset, bool& initialized, float& yaw_prev, float imu_yaw) {
    std::lock_guard<std::mutex> lock(odom_mutex);
    odom_count += 1;
    std::cout << "odom_count" << odom_count << "\n";
    imu_yaw = -imu_yaw;
    // if (!initialized) {
    //     yaw_offset = imu_yaw;  // Hướng ban đầu
    //     initialized = true;
    // }
    // imu_yaw = imu_yaw - yaw_offset;  // Yaw tuyệt đối theo hướng robot ban đầu
    // std::cout << "yaw_offset" << yaw_offset << "\n";
    yaw = imu_yaw * PI / 180.0;
    std::cout << "yaw: " << yaw << "\n";

    float vel_left = -v_left/20;
    float vel_right = v_right/20;
    // std::cout << "v_left=" << v_left << "(m/s), v_right=" << v_right << "(m/s)\n";

    ros::Time cur_time = ros::Time::now();
    // std::cout << "current_time: " << cur_time << "\n";
    float dt = (cur_time - lasttime).toSec();
    // std::cout << "dt: " << dt << "\n";
    // if (dt <= 0.0) return;
    // if (dt > 0.2) dt = 0.2; // clamp to avoid huge jumps
    lasttime = cur_time;

    // Robot velocities
    float v = (vel_right + vel_left) / 2.0;
    // std::cout << "v: " << v << "\n";
    float omega = (vel_right - vel_left) / 0.513;
    // std::cout << "omega: " << omega << "\n";

    // // Fuse IMU yaw if available
    // if (!std::isnan(imu_yaw)) {
    //     double alpha = 0.9;
    //     double yaw_pred = yaw + omega * dt;
    //     yaw = alpha * yaw_pred + (1.0 - alpha) * imu_yaw;
    // } else {
    //     yaw += omega * dt;
    // }
    // std::cout << "fused_yaw: " << yaw << "\n";

    // Integrate position
    float dyaw = (yaw_prev == 0) ? 0.001 : (yaw - yaw_prev);
    yaw_prev = yaw;
    const double eps = 1e-6;
    if (std::abs(omega) < eps) {
        x += v * cos(yaw) * dt;
        y += v * sin(yaw) * dt;
    } else {
        float r = v / omega;
        // float dyaw = omega*dt;
        std::cout << "dyaw: " << dyaw << "\n";
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