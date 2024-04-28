#include "ros/ros.h"
#include <sensor_msgs/Imu.h>

void doMsg(const sensor_msgs::Imu::ConstPtr &msg)
{
    double angular_velocity_x = msg->angular_velocity.x;
    double angular_velocity_y = msg->angular_velocity.y;
    double angular_velocity_z = msg->angular_velocity.z;

    double linear_acceleration_x = msg->linear_acceleration.x;
    double linear_acceleration_y = msg->linear_acceleration.y;
    double linear_acceleration_z = msg->linear_acceleration.z;

    double orientation_w = msg->orientation.w;
    double orientation_x = msg->orientation.x;
    double orientation_y = msg->orientation.y;
    double orientation_z = msg->orientation.z;

    std::cout << msg->header.stamp << std::endl;
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "recv_imu");
    ros::NodeHandle n;
    ros::Subscriber sub_imu = n.subscribe<sensor_msgs::Imu>("/lvbot_imu", 1, doMsg);
    ros::AsyncSpinner subSpinner(1);
    subSpinner.start();
    ros::Rate r(1000);

    while (ros::ok())
    {
        ///////////////////////////

        r.sleep();
    }
    subSpinner.stop();
    return 0;
}
