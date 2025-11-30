#pragma once
#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_datatypes.h>
#include <cmath>
#include <mutex>
#include <limits>
#define PI 3.14159265358979323846

extern float x, y, yaw, yaw_offset, yaw_prev, yaw_angle;
extern std::mutex odom_mutex;
// extern ros::Time lasttime;
extern bool initialized;
extern int odom_count;

double normalizeAngle(double a) {
    return atan2(sin(a), cos(a));
}

inline void updateOdometry(float vel_left, float vel_right, ros::Publisher& odom_pub,
                           double quaternion_yaw, ros::Time& lasttime, float imu_gyro_z = NAN)
{
    std::lock_guard<std::mutex> lock(odom_mutex);
    odom_count += 1;

    // --- YAW FILTER (LPF on vector components) ---
    static bool filter_initialized = false;
    static double filt_x = 1.0, filt_y = 0.0;
    static double last_input_yaw = 0.0;
    const double JUMP_THRESHOLD = 90.0 * PI / 180.0;
    double input_yaw = normalizeAngle(quaternion_yaw);

    if (!filter_initialized) {
        filt_x = cos(input_yaw); filt_y = sin(input_yaw);
        last_input_yaw = input_yaw;
        filter_initialized = true;
    } else {
        double diff_input = normalizeAngle(input_yaw - last_input_yaw);
        double filt_angle = atan2(filt_y, filt_x);
        double diff_filtered = normalizeAngle(input_yaw - filt_angle);
        bool reject = (std::abs(diff_input) > JUMP_THRESHOLD) && (std::abs(diff_filtered) > JUMP_THRESHOLD);

        if (!reject) {
            // dynamic alpha: when angular rate large -> faster response (smaller alpha)
            double ang_rate = std::isfinite(imu_gyro_z) ? std::abs(imu_gyro_z) : std::abs(diff_input);
            double ALPHA_fast = 0.05;
            double ALPHA_smooth = 0.8;
            double rate_thresh = 0.5; // rad/s
            double ALPHA = (ang_rate > rate_thresh) ? ALPHA_fast : ALPHA_smooth;

            double meas_x = cos(input_yaw);
            double meas_y = sin(input_yaw);
            filt_x = ALPHA * filt_x + (1.0 - ALPHA) * meas_x;
            filt_y = ALPHA * filt_y + (1.0 - ALPHA) * meas_y;
            double norm = sqrt(filt_x*filt_x + filt_y*filt_y);
            if (norm > 1e-12) { filt_x /= norm; filt_y /= norm; }
            else { filt_x = cos(input_yaw); filt_y = sin(input_yaw); }
        } else {
            ROS_WARN_THROTTLE(5.0, "[ODOM FILTER] Rejecting yaw jump: diff=%.3f deg", diff_input*180.0/PI);
        }
        last_input_yaw = input_yaw;
    }

    double filtered_yaw = atan2(filt_y, filt_x);
    yaw = static_cast<float>(filtered_yaw);

    // --- wheel velocities scaling (giữ cấu trúc hiện tại) ---
    float left_wheel = -vel_left / 20.0f;
    float right_wheel = vel_right / 20.0f;
    left_wheel  = (std::abs(left_wheel)  < 5e-4f) ? 0.0f : left_wheel;
    right_wheel = (std::abs(right_wheel) < 5e-4f) ? 0.0f : right_wheel;

    // --- time & dt guard ---
    ros::Time cur_time = ros::Time::now();
    double dt = (cur_time - lasttime).toSec();
    if (dt <= 0.0 || !std::isfinite(dt)) dt = 1e-3;
    const double DT_MAX = 0.2;
    if (dt > DT_MAX) { ROS_WARN_THROTTLE(2.0, "[ODOM] large dt=%.3f, clamped", dt); dt = DT_MAX; }
    lasttime = cur_time;

    // --- compute linear velocity and LPF it ---
    float v_raw = (right_wheel + left_wheel) / 2.0f;
    static double v_filt = 0.0;
    const double ALPHA_V = 0.6; // mượt hơn
    v_filt = ALPHA_V * v_filt + (1.0 - ALPHA_V) * v_raw;
    float v = static_cast<float>(v_filt);

    // --- wheel-based omega ---
    const double WHEEL_BASE = 0.57; // tune nếu cần
    float omega_wheel = (right_wheel - left_wheel) / WHEEL_BASE;

    // --- blend gyro + wheel omega nếu gyro có sẵn ---
    double omega = omega_wheel;
    if (std::isfinite(imu_gyro_z)) {
        const double BETA = 0.85; // trust gyro more
        // optionally apply LPF on gyro as well (simple)
        static double gyro_filt = 0.0;
        const double GYRO_ALPHA = 0.5;
        gyro_filt = GYRO_ALPHA * gyro_filt + (1.0 - GYRO_ALPHA) * imu_gyro_z;
        omega = static_cast<float>(BETA * gyro_filt + (1.0 - BETA) * omega_wheel);
    }

    // --- integrate position: midpoint/closed-form for diff drive ---
    double dyaw = omega * dt;
    double prev_x = x, prev_y = y;
    const double eps = 1e-6;
    if (std::abs(omega) < eps) {
        x += v * cos(yaw) * dt;
        y += v * sin(yaw) * dt;
    } else {
        double r = v / omega;
        x += r * (sin(yaw + dyaw) - sin(yaw));
        y += -r * (cos(yaw + dyaw) - cos(yaw));
    }

    if (!std::isfinite(x) || !std::isfinite(y)) {
        ROS_WARN("[ODOM] x,y invalid; revert");
        x = prev_x; y = prev_y;
    }

    // --- publish odom ---
    nav_msgs::Odometry odom;
    odom.header.stamp = cur_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    odom.twist.twist.linear.x = v;
    odom.twist.twist.angular.z = omega;

    // Covariance (tune sau)
    for (int i = 0; i < 36; i++) odom.pose.covariance[i] = 0.0;
    odom.pose.covariance[0]  = 0.05; // var x
    odom.pose.covariance[7]  = 0.05; // var y
    odom.pose.covariance[35] = 0.1; // var yaw

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
