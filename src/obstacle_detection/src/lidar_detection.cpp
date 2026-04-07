#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

class ObstacleDetector
{
public:
    ObstacleDetector()
    {
        ros::NodeHandle nh;
        scan_sub_ = nh.subscribe("/scan", 10, &ObstacleDetector::scanCallback, this);

        // Parameters
        nh.param("front_angle_range", front_angle_range_, 15.0);   // ±15° phía trước
        nh.param("obstacle_threshold", obstacle_threshold_, 0.8);  // mét
    }

private:
    ros::Subscriber scan_sub_;
    double front_angle_range_;
    double obstacle_threshold_;

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
    {
        int total_beams = msg->ranges.size();

        // Each beam angle (radians)
        double angle_increment = msg->angle_increment;
        double angle_min = msg->angle_min;

        // Convert angle range to radians
        double range_rad = front_angle_range_ * M_PI / 180.0;

        // Indices for front sector
        int start_idx = (int)((0 - range_rad - angle_min) / angle_increment);
        int end_idx   = (int)((0 + range_rad - angle_min) / angle_increment);

        start_idx = std::max(0, start_idx);
        end_idx   = std::min(total_beams - 1, end_idx);

        bool obstacle_detected = false;
        double min_dist = 10.0; // large number

        for (int i = start_idx; i <= end_idx; i++)
        {
            double d = msg->ranges[i];
            if (std::isnan(d) || std::isinf(d)) continue;

            min_dist = std::min(min_dist, d);

            if (d < obstacle_threshold_)
            {
                obstacle_detected = true;
            }
        }

        if (obstacle_detected)
        {
            ROS_WARN("OBSTACLE AHEAD! Closest distance: %.2f m", min_dist);
        }
        else
        {
            ROS_INFO("Clear ahead. Min dist: %.2f m", min_dist);
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "front_obstacle_detector");
    ObstacleDetector detector;
    ros::spin();
    return 0;
}
