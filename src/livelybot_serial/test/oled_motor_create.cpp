#include "ros/ros.h"
#include "../include/serial_struct.h"
#include "../include/hardware/robot.h"
#include <iostream>
#include <thread>
#include <condition_variable>
#include "std_msgs/Float32MultiArray.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_motor_feedback");
    ros::NodeHandle n;
    ros::Rate r(300);
    livelybot_serial::robot rb;
    std::vector<std::string> motor_name{"null","5046","4538","5047_36","5047_9"}; 
    ROS_INFO("motor num %ld" ,rb.Motors.size());
    while (ros::ok())
    {
        ros::spinOnce();
    }
}