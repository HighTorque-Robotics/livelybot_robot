#include "ros/ros.h"
#include "../include/serial_struct.h"
#include "../include/hardware/robot.h"
#include <iostream>
#include <thread>
#include <condition_variable>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "my_serial_port");
    ros::NodeHandle n;
    ros::Rate r(500);
    livelybot_serial::robot rb;
    ROS_INFO("\033[1;32mSTART\033[0m");
    // ========================== singlethread send =====================
    while (ros::ok()) // 此用法为逐个电机发送控制指令
    {
        // ROS_INFO_STREAM("START");
        /////////////////////////send
        // for (motor *m : rb.Motors)
        // {
            // m->fresh_cmd_int16(0.0, 0.0, 0.0, 0.0, 0.0, 0, 0, 0);
        // }
        rb.motor_send_2();
        ////////////////////////recv
        for (motor *m : rb.Motors)
        {
            motor_back_t motor;
            motor = *m->get_current_motor_state();
            // ROS_INFO("ID:%d pos: %8f,vel: %8f,tor: %8f", motor.ID, motor.position, motor.velocity, motor.velocity);
        }
        // ROS_INFO_STREAM("END"); //
        r.sleep();
    }

    ros::spin();
    return 0;
}
