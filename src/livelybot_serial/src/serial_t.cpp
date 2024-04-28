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
    // rb.test_ser_motor();
    // while (0)
    while (ros::ok()) // 此用法为逐个电机发送控制指令
    {
        // ROS_INFO_STREAM("START");
        /////////////////////////send
        // for (motor *m : rb.Motors)
        // {
            // m->fresh_cmd_int16(0.0, 0.0, 0.0, 0.0, 0.0, 0, 0, 0);
        // }
        rb.motor_send();
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

    for (auto &thread : rb.ser_recv_threads)
    {
        thread.join();
    }

    // ========弃用=========== multithread send =====================
    // rb.init_mutithread_send();
    // while (ros::ok())// 此用法为逐个电机多个串口并行发送控制指令
    // {
    //     // ROS_INFO_STREAM("START");
    //     ROS_INFO("===0===");
    //     for (motor &m : rb.Motors)
    //     {
    //        m.fresh_cmd(0.0, 0.0, 0.0, 0.0, 0.0);
    //     }
    //     rb.enable_send_multithread();
    //     // rb.CANPorts[0].enable_send_multithread();
    //     cv.notify_all(); // 通知子线程可以发送数据了
    //     ROS_INFO_STREAM("note"); // STEP2 -> END 1.7ms  START -> END 1.71
    //     r.sleep();
    // }

    ros::spin();
    return 0;
}
