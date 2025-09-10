#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <utils/pose_robot.h>
int main(int argc, char** argv)
{
    ros::init(argc, argv, "map_basefootprint_publisher");
    ros::NodeHandle nh;

    tf::TransformListener listener;
    ros::Publisher pose_pub = nh.advertise<utils::pose_robot>("/pose_robot1", 10);

    ros::Rate rate(10.0); // 10 Hz

    while (ros::ok())
    {
        tf::StampedTransform transform;
        try
        {
            // Lookup transform from map to base_footprint
            listener.lookupTransform("map", "base_footprint", ros::Time(0), transform);

            utils::pose_robot pose_msg;

            pose_msg.x = transform.getOrigin().x();
            pose_msg.y = transform.getOrigin().y();

            pose_pub.publish(pose_msg);
        }
        catch (tf::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
        }

        rate.sleep();
    }

    return 0;
}
