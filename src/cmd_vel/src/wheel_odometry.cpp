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

void updateWheelOdometry(float vel_left, float vel_right, ros::Publisher& wheel_odom_pub, ros::Time& last_time)
{
    std::lock_guard<std::mutex> lock(wheel_odom_mutex);

    // scale encoder
    float v_l = -vel_left  / 20.0f;
    float v_r =  vel_right / 20.0f;

    if (std::abs(v_l) < 5e-4) v_l = 0.0f;
    if (std::abs(v_r) < 5e-4) v_r = 0.0f;

    ros::Time now = ros::Time::now();
    double dt = (now - last_time).toSec();
    if (dt <= 0.0 || !std::isfinite(dt)) dt = 1e-3;
    if (dt > DT_MAX) dt = DT_MAX;
    last_time = now;

    double v     = (v_r + v_l) / 2.0;
    double omega = (v_r - v_l) / WHEEL_BASE;

    wheel_yaw += omega * dt;
    wheel_yaw = normalizeAngle(wheel_yaw);
    std::cout << "wheel_yaw = " << wheel_yaw * 360/PI << std::endl;

    if (std::abs(omega) < 1e-6) {
        x += v * cos(wheel_yaw) * dt;
        y += v * sin(wheel_yaw) * dt;
    } else {
        double r = v / omega;
        x += r * (sin(wheel_yaw + omega * dt) - sin(wheel_yaw));
        y += -r * (cos(wheel_yaw + omega * dt) - cos(wheel_yaw));
    }

    nav_msgs::Odometry odom;
    odom.header.stamp = now;
    odom.header.frame_id = "odom";          
    odom.child_frame_id = "base_footprint"; 

    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(wheel_yaw);

    odom.twist.twist.linear.x  = v;
    odom.twist.twist.angular.z = omega;

    // covariance cao → EKF sẽ "ít tin"
    for (int i = 0; i < 36; i++) odom.pose.covariance[i] = 0.0;
    odom.pose.covariance[0]  = 0.05;
    odom.pose.covariance[7]  = 0.05;
    odom.pose.covariance[35] = 0.5;

    wheel_odom_pub.publish(odom);
}
