#include "waveshare_can.h"
#include "can_protocol.h"
#include "wheel_odometry.h"
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#define PI 3.14159265358979323846

float x = 0;
float y = 0;
float wheel_yaw = 0;
std::mutex wheel_odom_mutex; 

WaveshareCAN can("/dev/usbcan", 2000000, 2.0);

float ConvertVelocityFromPulse(int pulse)
{
    // v (m/s) = (pulse / PPR) * circumference(mm) / 1000
    const float wheel_radius = 100.0f;      // in millimeters
    const int pulse_per_revolution = 10000; // pulses per wheel revolution
    const float circumference_mm = 2.0f * PI * wheel_radius;

    float velocity_mps = (static_cast<float>(pulse) * circumference_mm) /
                         (static_cast<float>(pulse_per_revolution) * 1000);
    return velocity_mps;
}

void imuCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    wheel_yaw = tf::getYaw(msg->pose.pose.orientation);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "can_node");
    ros::NodeHandle nh;
    ros::Publisher wheel_odom_pub = nh.advertise<nav_msgs::Odometry>("wheel_odometry", 10);
    ros::Subscriber imu_sub = nh.subscribe<nav_msgs::Odometry>("imu/data", 10, imuCallback);

    ros::Time lasttime = ros::Time::now();

    can.open();
    can.start_receive_loop(
        [&](uint16_t can_id, const std::vector<uint8_t>& data)
        {
            EncoderData enc;
            if (!decodeEncoder(can_id, data, enc))
                return;

            float left_mps  = ConvertVelocityFromPulse(enc.left_pulse);
            float right_mps = ConvertVelocityFromPulse(enc.right_pulse);

            updateWheelOdometry(left_mps, right_mps, wheel_yaw, wheel_odom_pub, lasttime);
        });

    ros::spin();
    return 0;
}
