// #pragma once
// #include "ros/ros.h"
// #include <nav_msgs/Odometry.h>
// #include <tf/transform_broadcaster.h>
// #include <geometry_msgs/TransformStamped.h>
// #include <tf/transform_datatypes.h>
// #include <cmath>
// #include <mutex>
// #define PI 3.14159265358979323846

// extern float x, y, yaw, yaw_offset, yaw_prev, yaw_angle;
// extern std::mutex odom_mutex;
// extern bool initialized;
// extern int odom_count;

// inline void updateOdometry(float vel_left, float vel_right, ros::Publisher& odom_pub, ros::Time& lasttime) {
//     std::lock_guard<std::mutex> lock(odom_mutex);
//     odom_count += 1;
//     // std::cout << "odom_count" << odom_count << "\n";
//     // float yaw_imu = -yaw_angle * PI / 180 + 2.615;
//     // if ((yaw_imu + 2.615) > PI) yaw_imu = yaw_imu + 2.615 - 2*PI;
//     // else yaw_imu = yaw_imu + 2.615;
//     float yaw_imu = -yaw_angle * PI / 180;
//     // std::cout << "yaw_imu: " << yaw_imu << "\n";

//     float left_wheel = -vel_left/20;
//     float right_wheel = vel_right/20;
//     left_wheel = (std::abs(left_wheel) < 5e-4) ? 0.0 : left_wheel; 
//     right_wheel = (std::abs(right_wheel) < 5e-4) ? 0.0 : right_wheel; 

//     ros::Time cur_time = ros::Time::now();
//     float dt = (cur_time - lasttime).toSec();
//     lasttime = cur_time;

//     // Robot velocities
//     float v = (right_wheel + left_wheel) / 2.0;
//     // std::cout << "v: " << v << "\n";
//     float omega = (right_wheel - left_wheel) / 0.57;
//     // std::cout << "omega: " << omega << "\n";

//     // // Fuse IMU yaw if available
//     // float yaw_enc = yaw_prev + omega * dt;
//     // yaw_enc = atan2(sin(yaw_enc), cos(yaw_enc));
//     // float alpha = 0.5;  // 0.9–0.99: tin encoder nhiều hơn
//     // float yaw_temp = alpha * yaw_enc + (1 - alpha) * yaw_imu;
//     // yaw = atan2(sin(yaw_temp), cos(yaw_temp));
//     // std::cout << "fused_yaw: " << yaw << "\n";

//     // // Use only yaw from encoder
//     // float yaw_enc = yaw_prev + omega*dt;
//     // yaw_enc = atan2(sin(yaw_enc), cos(yaw_enc));
//     // yaw = yaw_enc;
//     // std::cout << "yaw_encoder = " << yaw * 180 / PI << "\n";

//     yaw = yaw_imu;

//     // Integrate position
//     // float dyaw = (yaw_prev == 0.0) ? 0.0 : (yaw - yaw_prev);

//     // float dyaw = (yaw_prev == 0) ? 0.0 : (yaw_imu - yaw_prev);
//     float dyaw = omega*dt;
//     yaw_prev = yaw;
//     // yaw += dyaw;
//     // if (yaw > PI) yaw -= (2*PI);
//     // else if (yaw < -PI) yaw += (2*PI);

//     const double eps = 1e-6;
//     if (std::abs(omega) < eps) {
//         x += v * cos(yaw) * dt;
//         y += v * sin(yaw) * dt;
//     } else {
//         float r = v / omega;
//         // std::cout << "dyaw: " << dyaw << "\n";
//         x += r * (sin(yaw + dyaw) - sin(yaw));
//         y += -r * (cos(yaw + dyaw) - cos(yaw));
//     }

//     // std::cout << "x: " << x << ", y: " << y << "\n";

//     // Publish odom
//     nav_msgs::Odometry odom;
//     odom.header.stamp = cur_time;
//     odom.header.frame_id = "odom";
//     odom.child_frame_id = "base_footprint";
//     odom.pose.pose.position.x = x;
//     odom.pose.pose.position.y = y;
//     odom.pose.pose.position.z = 0.0;
//     odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
//     // std::cout << "pose.orientation: " << odom.pose.pose.orientation << "\n";
//     odom.twist.twist.linear.x = v;
//     odom.twist.twist.angular.z = omega;

//     // covariance (optional but useful for AMCL)
//     for (int i = 0; i < 36; i++) odom.pose.covariance[i] = 0.0;
//     odom.pose.covariance[0]  = 0.02;
//     odom.pose.covariance[7]  = 0.02; 
//     odom.pose.covariance[35] = 0.05;
//     odom_pub.publish(odom);

//     // Broadcast TF
//     static tf::TransformBroadcaster odom_broadcaster;
//     geometry_msgs::TransformStamped odom_tf;
//     odom_tf.header.stamp = cur_time;
//     odom_tf.header.frame_id = "odom";
//     odom_tf.child_frame_id = "base_footprint";
//     odom_tf.transform.translation.x = x;
//     odom_tf.transform.translation.y = y;
//     odom_tf.transform.translation.z = 0.0;
//     odom_tf.transform.rotation = tf::createQuaternionMsgFromYaw(yaw);
//     odom_broadcaster.sendTransform(odom_tf);

//     // std::cout << "yaw: " << yaw << "\n";
//     // std::cout << "v_left odom =" << left_wheel << "(m/s), v_right odom =" << right_wheel << "(m/s)\n";
//     // std::cout << "current_time: " << cur_time << "\n";
//     // std::cout << "dt: " << dt << "\n";
//     // std::cout << "v: " << v << "\n";
//     // std::cout << "omega: " << omega << "\n";
//     // std::cout << "x: " << x << ", y: " << y << "\n";
//     // std::cout << "pose.orientation: " << odom.pose.pose.orientation << "\n";
//     // std::cout << "---------------------------" <<"\n";
// }


// #pragma once
// #include "ros/ros.h"
// #include <nav_msgs/Odometry.h>
// #include <tf/transform_broadcaster.h>
// #include <geometry_msgs/TransformStamped.h>
// #include <tf/transform_datatypes.h>
// #include <cmath>
// #include <mutex>
// #define PI 3.14159265358979323846

// extern float x, y, yaw, yaw_offset, yaw_angle;
// extern float yaw_prev, qx_prev, qy_prev, qz_prev, qw_prev;
// extern std::mutex odom_mutex;
// extern bool initialized;
// extern int odom_count;

// // --- Normalize angle to [-pi, pi]
// inline double normalizeAngle(double a) {
//     return atan2(sin(a), cos(a));
// }

// inline void updateOdometry(float vel_left, float vel_right,
//                            ros::Publisher& odom_pub, ros::Time& lasttime,
//                            float& qx, float& qy, float& qz, float& qw,
//                            double& quaternion_yaw)
// {
//     std::lock_guard<std::mutex> lock(odom_mutex);

//     // Convert wheel speeds
//     float left_wheel  = -vel_left  / 20.0;
//     float right_wheel =  vel_right / 20.0;
//     if (fabs(left_wheel) < 5e-4) left_wheel = 0;
//     if (fabs(right_wheel) < 5e-4) right_wheel = 0;

//     ros::Time cur_time = ros::Time::now();
//     float dt = (cur_time - lasttime).toSec();
//     lasttime = cur_time;

//     float v = (right_wheel + left_wheel) / 2.0;
//     float omega = (right_wheel - left_wheel) / 0.57;

//     // --- Detect orientation jump (IMU or quaternion anomaly)
//     static double last_yaw = quaternion_yaw;

//     double diff = normalizeAngle(quaternion_yaw - last_yaw);
//     if (fabs(diff) > (30.0 * PI / 180.0)) {
//         // Reject quaternion orientation — use previous orientation
//         quaternion_yaw = last_yaw;
//         qx = qx_prev;
//         qy = qy_prev;
//         qz = qz_prev;
//         qw = qw_prev;
//     } else {
//         last_yaw = quaternion_yaw;
//         qx_prev = qx;
//         qy_prev = qy;
//         qz_prev = qz;
//         qw_prev = qw;
//     }

//     // --- Integrate position using IMU yaw
//     if (fabs(omega) < 1e-6) {
//         x += v * cos(quaternion_yaw) * dt;
//         y += v * sin(quaternion_yaw) * dt;
//     } else {
//         float r = v / omega;
//         x += r * (sin(quaternion_yaw + omega*dt) - sin(quaternion_yaw));
//         y += -r * (cos(quaternion_yaw + omega*dt) - cos(quaternion_yaw));
//     }

//     // --- Convert yaw IMU → Quaternion 2D
//     tf::Quaternion q2d;
//     q2d.setRPY(0, 0, quaternion_yaw);
//     q2d.normalize();

//     geometry_msgs::Quaternion q_msg;
//     q_msg.x = q2d.x();
//     q_msg.y = q2d.y();
//     q_msg.z = q2d.z();
//     q_msg.w = q2d.w();

//     // --- Publish odometry
//     nav_msgs::Odometry odom;
//     odom.header.stamp = cur_time;
//     odom.header.frame_id = "odom";
//     odom.child_frame_id = "base_footprint";
//     odom.pose.pose.position.x = x;
//     odom.pose.pose.position.y = y;
//     odom.pose.pose.position.z = 0;
//     odom.pose.pose.orientation = q_msg;
//     odom.twist.twist.linear.x = v;
//     odom.twist.twist.angular.z = omega;
//     odom_pub.publish(odom);

//     // --- TF
//     static tf::TransformBroadcaster odom_broadcaster;
//     geometry_msgs::TransformStamped odom_tf;
//     odom_tf.header.stamp = cur_time;
//     odom_tf.header.frame_id = "odom";
//     odom_tf.child_frame_id = "base_footprint";
//     odom_tf.transform.translation.x = x;
//     odom_tf.transform.translation.y = y;
//     odom_tf.transform.translation.z = 0.0;
//     odom_tf.transform.rotation = q_msg;
//     odom_broadcaster.sendTransform(odom_tf);


//     // std::cout << "yaw: " << yaw << "\n";
//     // std::cout << "v_left odom =" << left_wheel << "(m/s), v_right odom =" << right_wheel << "(m/s)\n";
//     // std::cout << "current_time: " << cur_time << "\n";
//     // std::cout << "dt: " << dt << "\n";
//     // std::cout << "v: " << v << "\n";
//     // std::cout << "omega: " << omega << "\n";
//     // std::cout << "x: " << x << ", y: " << y << "\n";
//     // std::cout << "pose.orientation: " << odom.pose.pose.orientation << "\n";
//     // std::cout << "---------------------------" <<"\n";
// }

#pragma once
#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_datatypes.h>
#include <cmath>
#include <mutex>
#define PI 3.14159265358979323846

extern float x, y, yaw, yaw_offset, yaw_prev, yaw_angle;
extern std::mutex odom_mutex;
extern bool initialized;
extern int odom_count;

inline void updateOdometry(float vel_left, float vel_right,
                           ros::Publisher& odom_pub, ros::Time& lasttime,
                           float& qx, float& qy, float& qz, float& qw,
                           double& quaternion_yaw)     
{                         
    std::lock_guard<std::mutex> lock(odom_mutex);
    odom_count += 1;
    // std::cout << "odom_count" << odom_count << "\n";
    // float yaw_imu = -yaw_angle * PI / 180 + 2.615;
    // if ((yaw_imu + 2.615) > PI) yaw_imu = yaw_imu + 2.615 - 2*PI;
    // else yaw_imu = yaw_imu + 2.615;
    float yaw_imu = quaternion_yaw * PI / 180;
    // std::cout << "yaw_imu: " << yaw_imu << "\n";

    float left_wheel = -vel_left/20;
    float right_wheel = vel_right/20;
    left_wheel = (std::abs(left_wheel) < 5e-4) ? 0.0 : left_wheel; 
    right_wheel = (std::abs(right_wheel) < 5e-4) ? 0.0 : right_wheel; 

    ros::Time cur_time = ros::Time::now();
    float dt = (cur_time - lasttime).toSec();
    lasttime = cur_time;

    // Robot velocities
    float v = (right_wheel + left_wheel) / 2.0;
    // std::cout << "v: " << v << "\n";
    float omega = (right_wheel - left_wheel) / 0.57;
    // std::cout << "omega: " << omega << "\n";

    yaw = yaw_imu;

    // Integrate position
    // float dyaw = (yaw_prev == 0.0) ? 0.0 : (yaw - yaw_prev);

    // float dyaw = (yaw_prev == 0) ? 0.0 : (yaw_imu - yaw_prev);
    float dyaw = omega*dt;
    yaw_prev = yaw;
    // yaw += dyaw;
    // if (yaw > PI) yaw -= (2*PI);
    // else if (yaw < -PI) yaw += (2*PI);

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
    odom.pose.covariance[0]  = 0.02;
    odom.pose.covariance[7]  = 0.02; 
    odom.pose.covariance[35] = 0.05;
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
}