#include <guidance.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <algorithm> 
#include <math.h>

PID pid_controller;

double low_pass_filter(double pre_value, double new_value, double alpha = 0.2){
    return alpha * new_value + (1 - alpha) * pre_value;
}

float get_heading (double x1, double y1, double x2, double y2){
    return atan2( y2 - y1, x2 - x1);
}

double normalize_angle(double angle) {
    angle = fmod(angle + PI, 2.0 * PI);
    if (angle < 0)
        angle += 2.0 * PI;
    return angle - PI;
}

double limit(double value, double min_val, double max_val)
{
    if (value < min_val) return min_val;
    else if (value > max_val) return max_val;
    else return value;
}

float control_los(float goal_x, float goal_y, float previous_x, float previous_y){
    alpha_k = get_heading(previous_x,previous_y,goal_x,goal_y);
    s_k_1   = (goal_x - previous_x) * cos(alpha_k) + (goal_y - previous_y) * sin(alpha_k); 

    cross_track = -(x - previous_x) * sin(alpha_k) + (y - previous_y) * cos(alpha_k);
    long_track  =  (x - previous_x) * cos(alpha_k) + (y - previous_y) * sin(alpha_k);

    delta       =  (delta_max - delta_min) * exp(-0.7 * pow(cross_track, 2)) + delta_min;

    target_heading = normalize_angle(alpha_k + atan(-cross_track/delta));
    heading_error  = normalize_angle(target_heading - theta);

    filtered_angular_z = pid_controller.pid(heading_error, KP, ANGULAR_SPEED);
    filtered_angular_z = limit( filtered_angular_z, - MAX_ANGULAR_SPEED, ANGULAR_SPEED);

    dist_to_goal = abs(s_k_1 - long_track);
    perc_dist = abs(s_k_1 - long_track)/s_k_1;

    if (abs(heading_error) > 0.1){
        linear_x = MAX_LINEAR_SPEED/2;
    }
    else {
        linear_x = limit( LINEAR_SPEED*perc_dist,1.0,MAX_LINEAR_SPEED);
    }
    filtered_angular_z = low_pass_filter(filtered_angular_z, angular_z);
    angular_z = filtered_angular_z;

    return linear_x, angular_z, dist_to_goal;
}

double tranfer_wp (){
    if (cnt + 1 >= (sizeof(wp) / sizeof(wp[0]))){
        ROS_INFO ("Stopping Robot");
        linear_x = 0.0;
        angular_z = 0.0;
        ros::shutdown();
    }
    else {
        linear_x,angular_z,dist_to_goal = control_los(wp[cnt+1][0],wp[cnt+1][1],wp[cnt][0],wp[cnt][1]);
    }

    if (dist_to_goal <= GOAL_RADIUS){
        ROS_INFO("Reached wp(%.2f, %.2f)",wp[cnt+1][0],wp[cnt+1][1]);
        cnt +=1;
    }
    return linear_x, angular_z;
}

void CallBackPosition (const utils::pose_robot::ConstPtr& msg){
    // auto it = std::find(msg->name.begin(), msg->name.end(), "robot_fablab");
    // if (it == msg->name.end()) return;
    // size_t idx = std::distance(msg->name.begin(), it);

    // x = msg->pose[idx].position.x;
    // y = msg->pose[idx].position.y;
    // const auto& q = msg->pose[idx].orientation;
    // tf::Quaternion quat(q.x, q.y, q.z, q.w);
    // tf::Matrix3x3(quat).getRPY(roll, pitch, theta);
    float theta = msg->yaw;
    ROS_INFO("theta=%.2f",theta);
}

void ControlVel(const ros::TimerEvent& event){
    utils::cmd_vel cmd;
    linear_x, angular_z = tranfer_wp();
    double v_left = linear_x - (angular_z * 0.535 / 2);
    double v_right = linear_x + (angular_z * 0.535 / 2);
    ROS_INFO("v_left = %.2f", v_left);
    ROS_INFO("v_right = %.2f", v_right);
    // cmd.linear.x  = linear_x;        
    // cmd.angular.z = angular_z; 
    cmd.v_left = 5.15; 
    cmd.v_right = -6.15; 
    pub.publish(cmd);
}


int main(int argc, char **argv){
    ros::init(argc,argv,"Guidance");
    ros::NodeHandle nh;

    pub = nh.advertise<utils::cmd_vel >("Cmd_vel", 1);
    sub = nh.subscribe("pose_robot",10, CallBackPosition);
    loopControl = nh.createTimer(ros::Duration(0.02), ControlVel);

    ros::spin();
    return 0;

}