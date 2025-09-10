#include <pid.h>

double PID::pid (double error, float kp, float ki){
    current_time = ros::Time::now().toSec();
    dt = (last_time > 0) ? (current_time - last_time) : 0.01;

    derivate = (dt > 0) ? (error - last_value)/dt : 0.0;
    
    last_value = error;
    last_time = current_time;
    return   ki * error + kp * derivate;
}