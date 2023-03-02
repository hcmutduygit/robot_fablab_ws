#include "wheel_odometry.hpp"
#include <tf/transform_datatypes.h>
#include <cmath>

#define PI 3.14159265358979323846

static constexpr double WHEEL_BASE = 0.57;
static constexpr double DT_MAX = 0.2;

double normalizeAngle(double a)
{
    return atan2(sin(a), cos(a));
}

void updateWheelOdometry(float vel_left, float vel_right,
                          ros::Publisher& wheel_odom_pub,
                          ros::Time& last_time)
{
    std::lock_guard<std::mutex> lock(wheel_odom_mutex);

    // scale encoder
    double v_l = -vel_left  / 20.0;
    double v_r =  vel_right / 20.0;
    

    if (std::abs(v_l) < 1e-3) v_l = 0.0;
    if (std::abs(v_r) < 1e-3) v_r = 0.0;
    // std::cout << "v_l=" << v_l << ", v_r=" << v_r << "\n";
    
    ros::Time now = ros::Time::now();
    double dt = (now - last_time).toSec();
    if (dt <= 0.0 || !std::isfinite(dt)) dt = 1e-3;
    if (dt > DT_MAX) dt = DT_MAX;
    last_time = now;

    double v     = (v_r + v_l) / 2.0;
    double omega = (v_r - v_l) / WHEEL_BASE;
  

    // lưu yaw cũ
    double yaw_old = wheel_yaw;

    // cập nhật posero
    if (std::abs(omega) < 5e-3) {
        x += v * cos(yaw_old) * dt;
        y += v * sin(yaw_old) * dt;
    } else {
        double r = v / omega;
        x += r * (sin(yaw_old + omega * dt) - sin(yaw_old));
        y += -r * (cos(yaw_old + omega * dt) - cos(yaw_old));
    }

    // cập nhật yaw SAU
    wheel_yaw = normalizeAngle(yaw_old + omega * dt);

    nav_msgs::Odometry odom;
    odom.header.stamp = now;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";

    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    tf::createQuaternionMsgFromYaw(wheel_yaw);

    odom.twist.twist.linear.x  = v;
    odom.twist.twist.linear.y = 0.0;
    // odom.twist.twist.linear.z = 0.0;
    odom.twist.twist.angular.z = omega;

    // covariance (wheel odom → không tin lắm)
    for (int i = 0; i < 36; i++) odom.pose.covariance[i] = 0.0;
    odom.pose.covariance[0]  = 0.05;
    odom.pose.covariance[7]  = 0.05;
    odom.pose.covariance[35] = 0.5;

    for (int i = 0; i < 36; i++) odom.twist.covariance[i] = 0.0;
    odom.twist.covariance[0]  = 0.2;   // vx
    odom.twist.covariance[35] = 1.0;   // vyaw (nếu có)

    wheel_odom_pub.publish(odom);
}

