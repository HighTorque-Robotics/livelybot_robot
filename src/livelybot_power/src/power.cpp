#include "livelybot_can_driver.hpp"
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/BatteryState.h>
#include <livelybot_power/Power_switch.h>
#include <livelybot_power/Power_detect.h>

ros::Publisher battery_volt_pub;
ros::Publisher power_detect_pub;
ros::Publisher power_switch_pub;
ros::Subscriber power_switch_sub;

livelybot_can::CAN_Driver can_handler("can0");  

void can_recv_parse(int id, unsigned char*data, unsigned char len);
void power_switch_callback(const livelybot_power::Power_switch& msg);

int main(int argc, char**argv)
{
    ros::init(argc, argv, "power_mission");
    ros::NodeHandle n;
    ros::Rate r(10.0);
    battery_volt_pub = n.advertise<std_msgs::Float32>("battery_voltage", 1);
    power_switch_pub = n.advertise<livelybot_power::Power_switch>("power_switch_state", 1);
    power_detect_pub = n.advertise<livelybot_power::Power_detect>("power_detect_state", 1);
    power_switch_sub = n.subscribe("power_switch_control", 1, power_switch_callback);
    can_handler.start_callback(can_recv_parse);
    while (ros::ok())
    {
        r.sleep();
        ros::spinOnce();
    }
    return 0;
}

#define ORANGEPI_ADDR       0x01
#define BMS_ADDR            0x06
#define POWER_SWITCH_ADDR   0x07

void can_recv_parse(int can_id, unsigned char*data, unsigned char dlc)
{
    uint8_t dev_addr;
    uint8_t data_type;
    uint8_t append_flag;

    dev_addr = can_id >> 7;
    data_type = (can_id>>1) & 0x3F;
    append_flag = can_id >> 10;

    // printf("接受到CAN数据 ADDR-TYPE-FLAG:%d, %d, %d\n", dev_addr, data_type, append_flag);

    if(!append_flag)
    {
        switch (dev_addr)
        {
            case BMS_ADDR:
                switch(data_type)
                {
                    case 0x01:      // BMS状态
                    {
                        // printf("cell-v-c-t:%.2fV, %.2fA, %.2f°C\n", (*(int16_t*)&data[0])/100.0f, (*(int16_t*)&data[2])/100.0f, (*(int16_t*)&data[4])/100.0f);
                        std_msgs::Float32 msg;
                        msg.data = (*(int16_t*)&data[0])/100.0f;
                        battery_volt_pub.publish(msg);
                        // sensor_msgs::BatteryState battery_state;
                        // battery_state.voltage = (*(int16_t*)&data[0])/100.0f;// # 示例电压值
                        // battery_state.charge = 1.0f; // # 示例电荷量
                        // battery_state.percentage = 50.0; //# 示例电量百分比
                        // battery_state.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_FULL;// # 示例状态
                        // battery_state.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD;// # 示例健康状态
                        // battery_state.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION;// # 示例电池技术
                        // battery_volt_pub_d.publish(battery_state);
                    }
                    break;
                    default:
                        printf("Error Type\n");
                    break;
                }
                break;
            case POWER_SWITCH_ADDR:
                switch(data_type)
                {
                    case 0x01:
                    {
                        // printf("power-v-c:%.2fV, %.2fA\n", (*(int16_t*)&data[0])/100.0f, (*(int16_t*)&data[2])/100.0f);
                        livelybot_power::Power_detect power_detect_msg;
                        power_detect_msg.voltage = (*(int16_t*)&data[0])/100.0f;
                        power_detect_msg.current = (*(int16_t*)&data[2])/100.0f;
                        power_detect_msg.power = power_detect_msg.voltage * power_detect_msg.current;
                        power_detect_pub.publish(power_detect_msg);
                        break;
                    }
                    case 0x02:
                    {
                        // printf("switch status:%d, %d\r\n", data[0], data[1]);
                        livelybot_power::Power_switch power_switch_msg;
                        power_switch_msg.control_switch = data[0];
                        power_switch_msg.control_switch = data[1];
                        power_switch_pub.publish(power_switch_msg);
                        break;
                    }
                }
                break;
            default:
                break;
        }
    }
}

void power_switch_callback(const livelybot_power::Power_switch& msg)
{
    uint32_t can_id = (ORANGEPI_ADDR<<7) | (1 << 1) | 0;
    uint8_t data[2];
    
    data[0] = msg.control_switch;
    data[1] = msg.power_switch;

    can_handler.send(can_id, data, 2);
}
