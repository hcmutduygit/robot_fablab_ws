#ifndef PID_H
#define PID_H

#include <ros/ros.h>

class PID {
private:
    double last_value = 0;
    double dt = 0.1;
    double current_time = 0;
    double last_time = 0;
    double derivate = 0;

public:
    double pid(double error, float kp, float ki);
};
#endif