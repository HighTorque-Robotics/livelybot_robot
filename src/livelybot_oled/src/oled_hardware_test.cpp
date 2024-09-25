#include "sensor_actuator_status.hpp"
#include <iostream>
#include <unistd.h>
#include <ctime>

int main(int argc, char** argv)
{
    Sensor_actuator_status status(6, 6, 0, 0);
    float imu[3] = {1.23f, 2.45f, 5.6f};
    uint8_t motor_status[12] = {1,1,0,0,1,1,0,1,0,1,0,1};
    uint32_t ip_addr[3] = {345735, 837438, 998877};
    uint8_t fsm_status = 0;
    float battery = 23.2f;
  
    std::clock_t delay = 0.1 * CLOCKS_PER_SEC;
    std::clock_t start = std::clock();
    while(true)
    {

        // IMU
        status.send_imu_status(true, imu);
        // Motor state
        status.send_motor_status(motor_status);
        // IP
        status.send_ip_addr(ip_addr, 3);
        // Battery
        status.send_battery_volt(battery);
        // Robot State
        status.send_fsm_state(fsm_status);
        
        std::clock_t start = std::clock();
        while(std::clock() - start < delay);

        std::cout << "Send" << std::endl;
    }
}