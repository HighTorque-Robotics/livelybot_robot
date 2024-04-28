#include "ros/ros.h"
#include "../include/serial_struct.h"
#include "../include/hardware/robot.h"
#include <iostream>
#include <thread>
#include <condition_variable>

#define Robot_motors 12    //机器人电机总数

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_motor");
    ros::NodeHandle n;
    ros::Rate r(100);
    livelybot_serial::robot rb;
    ROS_INFO("\033[1;32mSTART\033[0m");
    // ========================== singlethread send =====================
    int cont = 0, i = 0;
    float angle = 0.2;
    while (ros::ok()) // 此用法为逐个电机发送控制指令
    {
        /////////////////////////send
        for (motor *m : rb.Motors)
        {
            if(i < (int)Robot_motors/2)
            {
                // rb.Motors[i]->fresh_cmd_int16(angle, 0.0, 0.0, 15.0, 0, 0.1, 0, 0, 0);
                rb.Motors[i]->pos_vel_MAXtqe(angle, angle, 10);
            }
            else
            {
                // rb.Motors[i]->fresh_cmd_int16(-angle, 0.0, 0.0, 15.0, 0, 0.1, 0, 0, 0);
                rb.Motors[i]->pos_vel_MAXtqe(-angle, angle, 10);
            }
            i++;
        }
        i = 0;
        cont++;
        if(cont==200)
        {
            cont = 0;
            angle*=-1;
        }
        rb.motor_send_2();
        ////////////////////////recv
        for (motor *m : rb.Motors)
        {
            motor_back_t motor;
            motor = *m->get_current_motor_state();
            ROS_INFO("ID:%d pos: %8f,vel: %8f,tor: %8f", motor.ID, motor.position, motor.velocity, motor.torque);
        }
        r.sleep();
    }

    ros::spin();
    return 0;
}




