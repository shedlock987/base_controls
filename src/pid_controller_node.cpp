#include <ros/ros.h>

#include "pid_controller/pid_controller.h"
using namespace controls;
/*
int main(int argc, char **argv)
{
    ros::init(argc, argv, "pid_controller_node");

    ros::NodeHandle nh;
    ros::NodeHandle p_nh("~");

    PID_Controller::PID_Controller controllerNode( nh, p_nh);
}
*/

int main () 
{
    ros::Rate loop_rate(100); // 10ms

    double kp = 1.1;
    double ki = 1.2;
    double kd = 1.3;
    double pidmax = 10;
    double pidmin = -9;
    double imax = 5;
    double imin = -4.5;
    double coef = 1;
    double init = 1.12;
    double err = 0.1;
    double cmd = 0;

    PID_Controller::PID_Controller myControl(kp, ki, _kd, pidmax, pidmin, imax, imin, init, coef);
    while (ros::ok())
    {
        cmd = myControl.Step(err);
        loop_rate.sleep();
        ros::spinOnce();
    }
}