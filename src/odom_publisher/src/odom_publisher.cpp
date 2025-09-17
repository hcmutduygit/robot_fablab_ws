#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <utils/pose_robot.h>   
#include <tf/transform_datatypes.h> 

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

    void poseCallback(const utils::pose_robot::ConstPtr& msg)
    {
        nav_msgs::Odometry odom;

        odom.header.stamp = ros::Time::now();
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";

        // Mapping dữ liệu từ custom msg sang Odometry
        odom.pose.pose.position.x = msg->x;
        odom.pose.pose.position.y = msg->y;
        odom.pose.pose.position.z = 0.0;   // không có trong msg, gán 0

        // Chuyển yaw -> quaternion
        geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(msg->yaw);
        odom.pose.pose.orientation = q;

        // Nếu chưa có dữ liệu vận tốc từ CAN thì gán = 0
        odom.twist.twist.linear.x = 0.0;
        odom.twist.twist.linear.y = 0.0;
        odom.twist.twist.angular.z = 0.0;

        odom_pub_.publish(odom);

        // Broadcast TF (odom -> base_link)
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(msg->x, msg->y, 0.0));
        tf::Quaternion q_tf;
        q_tf.setRPY(0, 0, msg->yaw);
        transform.setRotation(q_tf);
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
