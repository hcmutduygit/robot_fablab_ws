#include "pid.h"

double PID::pid(double error, float kd, float kp) {
    current_time = ros::Time::now().toSec();
    dt = (last_time > 0) ? (current_time - last_time) : 0.01;

    // derivate = (dt > 0) ? (error - last_value)/dt : 0.0;
    if (dt < 0.001) dt = 0.001;
    derivate = (error - last_value) / dt;
    double alpha_d=0.2;
    filter_derivate=alpha_d*derivate+(1-alpha_d)*filter_derivate;
    last_value = error;
    last_time = current_time;
    // return   ki * error + kp * derivate;
    return kp * error + kd * derivate;
}