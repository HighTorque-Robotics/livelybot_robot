#include <stdio.h>
#include <string.h>
#include "serial/serial.h"

#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
#include <vector>

typedef struct
{
    bool imu_con_sta;
    float rpy[3];
}imu_status_s;

typedef struct
{
    int can_nums;
    int motor_nums;
    unsigned char motor_status[80];
}motor_status_s;

class Sensor_actuator_status
{
public:
    imu_status_s imu_status;
    motor_status_s motor_status;
    serial::Serial _ser;
    int suc_flag;
    std::vector<unsigned char> send_buff;
    Sensor_actuator_status(int can_nums, int motor_nums);
    ~Sensor_actuator_status();
    void send_imu_actuator_status(bool iomu_exist, float *rpy, unsigned char *m_status);
    void send_ip_addr(unsigned int* ip_data, unsigned char len);
};

