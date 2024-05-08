#include "ros/ros.h"
#include "../include/serial_struct.h"
#include "../include/hardware/robot.h"
#include <iostream>
#include <thread>
#include <condition_variable>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_motor");
    ros::NodeHandle n;
    ros::Rate r(100);
    livelybot_serial::robot rb;
    ROS_INFO("\033[1;32mSTART\033[0m");
    // ========================== singlethread send =====================
    float derta = 0.01;//角度
    int cont = 0;
    float angle = 0.0;
    while (ros::ok()) // 此用法为逐个电机发送控制指令
    {
        /////////////////////////send
        for (motor *m : rb.Motors)
        {   
            // printf("id %d pos %f vel %f tqe %f\n", m->get_current_motor_state()->ID, m->get_current_motor_state()->position, m->get_current_motor_state()->velocity, m->get_current_motor_state()->torque);
            printf("%4.2f ", m->get_current_motor_state()->position);
            m->fresh_cmd_int16(0, 0, 0, 5.0, 0, 0, 0.1, 0, 0.5);
        }
        printf("\n");
        rb.motor_send_2();
        
        r.sleep();
    }

    ROS_INFO_STREAM("END"); 
    ros::spin();
    return 0;
}
