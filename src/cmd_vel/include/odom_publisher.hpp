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
// extern ros::Time lasttime;
extern bool initialized;
extern int odom_count;

inline void updateOdometry(float vel_left, float vel_right, ros::Publisher& odom_pub, double& quaternion_yaw, ros::Time& lasttime)     
{                         
    std::lock_guard<std::mutex> lock(odom_mutex);
    odom_count += 1;
    int16_t yaw_temp = round(quaternion_yaw * 180/PI);
    yaw = yaw_temp * PI/180;
    // std::cout << "yaw_temp: " << yaw_temp << ", yaw: " << yaw << "\n";
    float left_wheel = -vel_left/20;
    float right_wheel = vel_right/20;

    left_wheel  = (std::abs(left_wheel)  < 5e-4) ? 0.0 : left_wheel;
    right_wheel = (std::abs(right_wheel) < 5e-4) ? 0.0 : right_wheel;

    ros::Time cur_time = ros::Time::now();
    float dt = (cur_time - lasttime).toSec();
    lasttime = cur_time;

    float v = (right_wheel + left_wheel) / 2.0;
    float omega = (right_wheel - left_wheel) / 0.57;
    // std::cout << "yaw=" << yaw << ", dt=" << dt << "\n";


    float dyaw = omega * dt;
    yaw = (yaw == 0) ? yaw_prev : yaw;
    yaw_prev = yaw;
    if(fabs(yaw-yaw_prev)>0.7) {
        yaw=yaw_prev;
    }
    else yaw=yaw;
 
    yaw_prev = yaw;
   
    // --- Backup previous position ---
    float prev_x = x;
    float prev_y = y;

    // --- Integrate position ---
    const double eps = 1e-6;
    if (std::abs(omega) < eps) {
        x += v * cos(yaw) * dt;
        y += v * sin(yaw) * dt;
    } else {
        float r = v / omega;
        x += r * (sin(yaw + dyaw) - sin(yaw));
        y += -r * (cos(yaw + dyaw) - cos(yaw));
    }

    // --- NaN / Inf / overflow guard ---
    if (!std::isfinite(x) || !std::isfinite(y)||!std::isfinite(yaw)) {
        ROS_WARN("[ODOM] x,y became invalid! Reverting to previous values.");
        x = prev_x;
        y = prev_y;
        yaw = yaw_prev  ;
    }

    // --- Publish odom ---
    nav_msgs::Odometry odom;
    odom.header.stamp = cur_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    odom.twist.twist.linear.x = v;
    odom.twist.twist.angular.z = omega;

    for (int i = 0; i < 36; i++) odom.pose.covariance[i] = 0.0;
    odom.pose.covariance[0]  = 0.02;
    odom.pose.covariance[7]  = 0.02;
    odom.pose.covariance[35] = 0.05;
    odom_pub.publish(odom);

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
// // extern ros::Time lasttime;
// extern bool initialized;
// extern int odom_count;

// // --- Normalize angle to [-pi,pi]
// inline double normalizeAngle(double a) {
//     return atan2(sin(a), cos(a));
// }

// inline void updateOdometry(float vel_left, float vel_right, ros::Publisher& odom_pub, double& quaternion_yaw, ros::Time& lasttime)     
// {                         
//     std::lock_guard<std::mutex> lock(odom_mutex);
//     odom_count += 1;

//     // --- YAW FILTER: vector LPF (cos,sin) + jump rejection
//     static bool filter_initialized = false;
//     static double filt_x = 1.0, filt_y = 0.0;   // filtered unit vector
//     static double last_input_yaw = 0.0;
//     const double JUMP_THRESHOLD = 90.0 * PI / 180.0; // reject jumps >45 deg
//     const double ALPHA = 0.2; // weight for previous filtered vector (0..1). smaller -> faster response

//     double input_yaw = normalizeAngle(quaternion_yaw);

//     if (!filter_initialized) {
//         filt_x = cos(input_yaw);
//         filt_y = sin(input_yaw);
//         last_input_yaw = input_yaw;
//         filter_initialized = true;
//     } else {
//         // detect large discontinuity between last input and current input
//         double diff_input = normalizeAngle(abs(input_yaw) - abs(last_input_yaw));
//         // detect large discontinuity between filtered angle and new input
//         double filt_angle = atan2(filt_y, filt_x);
//         double diff_filtered = normalizeAngle(input_yaw - filt_angle);

//         bool reject = (std::abs(diff_input) > JUMP_THRESHOLD) && (std::abs(diff_filtered) > JUMP_THRESHOLD);

//         if (reject) {
//             ROS_WARN_THROTTLE(5.0, "[ODOM FILTER] Rejecting yaw jump: input diff %.3f deg", diff_input * 180.0 / PI);
//             // do not update filt_x/filt_y (keep previous filtered orientation)
//         } else {
//             // update filtered unit vector (LPF on vector components)
//             double meas_x = cos(input_yaw);
//             double meas_y = sin(input_yaw);
//             filt_x = ALPHA * filt_x + (1.0 - ALPHA) * meas_x;
//             filt_y = ALPHA * filt_y + (1.0 - ALPHA) * meas_y;
//             // renormalize to unit length
//             double norm = sqrt(filt_x*filt_x + filt_y*filt_y);
//             if (norm > 1e-12) {
//                 filt_x /= norm;
//                 filt_y /= norm;
//             } else {
//                 filt_x = cos(input_yaw);
//                 filt_y = sin(input_yaw);
//             }
//         }
//         last_input_yaw = input_yaw;
//     }

//     double filtered_yaw = atan2(filt_y, filt_x);
//     yaw = static_cast<float>(filtered_yaw); // update global yaw used downstream

//     float left_wheel = -vel_left/20;
//     float right_wheel = vel_right/20;

//     left_wheel  = (std::abs(left_wheel)  < 5e-4) ? 0.0 : left_wheel;
//     right_wheel = (std::abs(right_wheel) < 5e-4) ? 0.0 : right_wheel;

//     ros::Time cur_time = ros::Time::now();
//     float dt = (cur_time - lasttime).toSec();
//     lasttime = cur_time;

//     float v = (right_wheel + left_wheel) / 2.0;
//     float omega = (right_wheel - left_wheel) / 0.57;
//     // std::cout << "yaw=" << yaw << ", dt=" << dt << "\n";

//     float dyaw = omega * dt;
//     yaw_prev = yaw;

//     // --- Backup previous position ---
//     float prev_x = x;
//     float prev_y = y;

//     // --- Integrate position ---
//     const double eps = 1e-4;
//     if (std::abs(omega) < eps) {
//         x += v * cos(yaw) * dt;
//         y += v * sin(yaw) * dt;
//     } else {
//         float r = v / omega;
//         x += r * (sin(yaw + dyaw) - sin(yaw));
//         y += -r * (cos(yaw + dyaw) - cos(yaw));
//     }

//     // --- NaN / Inf / overflow guard ---
//     if (!std::isfinite(x) || !std::isfinite(y)) {
//         ROS_WARN("[ODOM] x,y became invalid! Reverting to previous values.");
//         x = prev_x;
//         y = prev_y;
//     }

//     // --- Publish odom ---
//     nav_msgs::Odometry odom;
//     odom.header.stamp = cur_time;
//     odom.header.frame_id = "odom";
//     odom.child_frame_id = "base_footprint";
//     odom.pose.pose.position.x = x;
//     odom.pose.pose.position.y = y;
//     odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
//     odom.twist.twist.linear.x = v;
//     odom.twist.twist.angular.z = omega;

//     for (int i = 0; i < 36; i++) odom.pose.covariance[i] = 0.0;
//     odom.pose.covariance[0]  = 0.02;
//     odom.pose.covariance[7]  = 0.02;
//     odom.pose.covariance[35] = 0.05;
//     odom_pub.publish(odom);

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
// }