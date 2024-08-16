#include "sensor_actuator_status.hpp"
#include <iostream>

Sensor_actuator_status::Sensor_actuator_status(int can_nums, int motor_nums)
{
    this->motor_status.can_nums = can_nums;
    this->motor_status.motor_nums = motor_nums;
    this->send_buff.resize(21 + can_nums*motor_nums);
    this->_ser.setPort("/dev/ttyS1");
    this->_ser.setBaudrate(115200);
    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    this->_ser.setTimeout(to);
    try
    {
        this->_ser.open();
    }
    catch (const std::exception &e)
    {
        //
    }
    if(this->_ser.isOpen())
    {
        this->suc_flag = 1;
    }

}

Sensor_actuator_status::~Sensor_actuator_status()
{
    this->_ser.close();
}

void Sensor_actuator_status::send_imu_actuator_status(bool imu_exist, float *rpy, unsigned char *motor_status)
{
    this->imu_status.imu_con_sta = imu_exist;
    memcpy(this->imu_status.rpy, rpy,12);
    memcpy(this->motor_status.motor_status, motor_status, this->motor_status.can_nums*this->motor_status.motor_nums);

    this->send_buff[0] = 0xA5;
    this->send_buff[1] = 0x5A;
    this->send_buff[2] = 15 + this->motor_status.can_nums * this->motor_status.motor_nums;
    this->send_buff[3] = this->imu_status.imu_con_sta;
    // + 12
    for(int i = 0; i < 3; i++)
    {
        *(((float*)(&this->send_buff[4]))+i) = this->imu_status.rpy[i];
    }
    this->send_buff[16] = this->motor_status.can_nums;
    this->send_buff[17] = this->motor_status.motor_nums;
    for(int i = 0; i < this->motor_status.can_nums * this->motor_status.motor_nums; i++)
    {
        this->send_buff[18 + i] = this->motor_status.motor_status[i];
    }
    this->send_buff[this->send_buff[2] + 3] = 0x66;
    this->send_buff[this->send_buff[2] + 4] = 0x47;
    this->send_buff[this->send_buff[2] + 5] = 0x74;
    for (auto it = this->send_buff.begin(); it != this->send_buff.end(); it ++)
    {
        this->_ser.write(&(*it), 1);
        printf("0x%x ", *it);
    }
    std::cout << std::endl;
}

void Sensor_actuator_status::send_ip_addr(unsigned int* ip_data, unsigned char len)
{
    this->send_buff[0] = 0xA5;
    this->send_buff[1] = 0x5A;
    this->send_buff[2] = len * 4 + 1;
    memcpy(&this->send_buff[3], ip_data, len * 4);
    this->_ser.write(&(*this->send_buff.begin()), this->send_buff[2] + 2); // 数据包长度+帧头两个字节
}
