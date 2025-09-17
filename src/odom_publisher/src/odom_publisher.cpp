#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

class OdomPublisher
{
public:
    OdomPublisher()
    {
        ros::NodeHandle nh;

        // Subscriber: nhận pose từ can_node
        pose_sub_ = nh.subscribe("/pose_robot", 10, &OdomPublisher::poseCallback, this);

        // Publisher: phát ra nav_msgs/Odometry
        odom_pub_ = nh.advertise<nav_msgs::Odometry>("/odom", 10);
    }

    void poseCallback(const geometry_msgs::Pose::ConstPtr& msg)
    {
        nav_msgs::Odometry odom;

        odom.header.stamp = ros::Time::now();
        odom.header.frame_id = "odom";      // khung gốc
        odom.child_frame_id = "base_link";  // khung robot

        // Gán position & orientation từ pose_robot
        odom.pose.pose = *msg;

        // Nếu chưa có dữ liệu vận tốc từ CAN thì gán = 0
        odom.twist.twist.linear.x = 0.0;
        odom.twist.twist.linear.y = 0.0;
        odom.twist.twist.angular.z = 0.0;

        odom_pub_.publish(odom);

        // Broadcast TF (odom -> base_link)
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(
            msg->position.x,
            msg->position.y,
            msg->position.z
        ));
        tf::Quaternion q(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w
        );
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));
    }

private:
    ros::Subscriber pose_sub_;
    ros::Publisher odom_pub_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odom_publisher");
    OdomPublisher odom_publisher;
    ros::spin();
    return 0;
}
