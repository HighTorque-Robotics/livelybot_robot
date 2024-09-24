#include <stdio.h>
#include <string.h>
// #include <wiringPi.h>
// #include <wiringSerial.h>
#include <stdlib.h>
#include <unistd.h>
#include "sensor_actuator_status.hpp"
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <tf/transform_datatypes.h>
#include "ip_addr.hpp"

#define _2PI_ 6.2831853

Sensor_actuator_status *status;

uint8_t imu_flag = 0;
float rpy[3];

int can_port_num;
int motor_num[4];

void imu_callback(const sensor_msgs::Imu::ConstPtr& data);
void motor_callback(const sensor_msgs::JointState& data);
void battery_voltage_callback(const std_msgs::Float32::ConstPtr& msg);
void fsm_state_callback(const std_msgs::Int32::ConstPtr& msg);

int main(int argc, char **argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "oled_run");

    // 创建节点句柄
    ros::NodeHandle n;
    ros::Rate r(10.0);
    
    // 获取配置参数，CAN线与电机数量
    if (n.getParam("robot/CANboard/No_" + std::to_string(1) + "_CANboard/CANport_num", can_port_num))
    {
        ROS_INFO("Robot has %d CAN", can_port_num);
        for(int i = 0; i < can_port_num; i++)
        {
            if (n.getParam("robot/CANboard/No_" + std::to_string(1) + "_CANboard/CANport/CANport_" + std::to_string(i+1) + "/motor_num", motor_num[i]))
            {
                //TODO
                ROS_INFO("can_%d has %d motor", i+1, motor_num[i]);
            }
            else
            {
                ROS_ERROR("Faile to get params motor_num");
            }
        }
    }
    else
    {
        ROS_ERROR("Faile to get params can_port_num");
    }

    // 创建电机status的对象
    status = new Sensor_actuator_status(motor_num[0], motor_num[1], motor_num[2], motor_num[3]);

    // 订阅imu_data话题
    ros::Subscriber sub = n.subscribe("/imu/data", 1, imu_callback);
    ros::Subscriber m_sub = n.subscribe("/error_joint_states", 1, motor_callback); // error_joint_states
    ros::Subscriber battery_volt_sub = n.subscribe("/battery_voltage", 1, battery_voltage_callback);
    ros::Subscriber fsm_state_sub = n.subscribe("/fsm_state", 1, fsm_state_callback);
    int i = 0;
    update_ip_addr();
    // unsigned char motor_status[12] = {1,0,1,0,1,1,0,1,1,0,0,1};
    // 循环等待消息
    while(ros::ok())
    {
        r.sleep();
        ros::spinOnce();
        i++;
        if(i>=10)
        {
            i = 0;
            update_ip_addr();
        }
        // status.send_motor_status(motor_status);
        status->send_ip_addr(get_ip_data_u32_all(), 3);
        if(imu_flag)
        {
            status->send_imu_status(0, rpy);
        }
        imu_flag = 1;
    }

    return 0;
}

void imu_callback(const sensor_msgs::Imu::ConstPtr& data) {
    // 提取IMU姿态角
    static int cnt = 0;
    double roll, pitch, yaw;
    tf::Quaternion q(
        data->orientation.x,
        data->orientation.y,
        data->orientation.z,
        data->orientation.w
    );
    tf::Matrix3x3 rot_mat(q);
    rot_mat.getRPY(roll, pitch, yaw);
    // ROS_INFO("Roll: %.2f, Pitch: %.2f, Yaw: %.2f", roll, pitch, yaw);
    float rpy[3];
    rpy[0] = roll;
    rpy[1] = pitch;
    rpy[2] = yaw;
    // for(int i = 0; i < 3; i++)
    // {
    //     if(rpy[i] > _2PI_)
    //     {
    //         rpy[i] -= _2PI_;
    //     }else if(rpy[i] < 0)
    //     {
    //         rpy[i] += _2PI_;
    //     }
    // }
    // status.send_imu_actuator_status(1, rpy, motor_status);
    status->send_imu_status(1, rpy);
    imu_flag = 0;
}

void motor_callback(const sensor_msgs::JointState& data)
{
    unsigned char motor_status[12] = {1,0,1,0,1,1,0,1,1,0,0,1};
    for(int i = 0; i < 12; i++)
    {
        if(data.position[i] < -900.0)
        {
            motor_status[i] = 0;
        }
        else
        {
            motor_status[i] = 1;
        }
    }
    status->send_motor_status(motor_status);
}

void battery_voltage_callback(const std_msgs::Float32::ConstPtr& msg)
{
    ROS_INFO("receive voltage:%f", msg->data);
    status->send_battery_volt(msg->data);
}

void fsm_state_callback(const std_msgs::Int32::ConstPtr& msg)
{
    ROS_INFO("receive fsm_state:%d", msg->data);
    status->send_fsm_state(msg->data);
}
