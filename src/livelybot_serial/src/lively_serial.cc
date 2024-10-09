#include "../include/lively_serial.h"
lively_serial::~lively_serial()
{
}

void lively_serial::recv_1for6_42()
{
    uint8_t CRC8 = 0;
    uint16_t CRC16 = 0;
    SOF_t SOF = {0};
    cdc_tr_message_data_s cdc_rx_message_data = {0};
    while (ros::ok() && init_flag)
    {
        _ser.read(&(SOF.head), 1); 
        if (SOF.head == 0xF7)      //  head
        {
            _ser.read(&(SOF.cmd), 4);
            if (SOF.CRC8 == Get_CRC8_Check_Sum((uint8_t *)&(SOF.cmd), 3, 0xFF)) // cmd_id
            {
                _ser.read((uint8_t *)&CRC16, 2);
                _ser.read((uint8_t *)&cdc_rx_message_data, SOF.data_len);
                if (CRC16 != crc_ccitt(0xFFFF, (const uint8_t *)&cdc_rx_message_data, SOF.data_len))
                {
                    memset(&cdc_rx_message_data, 0, sizeof(cdc_rx_message_data) / sizeof(int));
                }
                else
                {
                    // printf("cmd %02X  ", SOF.cmd);
                    // for (int i = 0; i < SOF.data_len; i++)
                    // {
                    //     printf("0x%02X ", cdc_rx_message_data.data[i]);
                    // }
                    // printf("\n");

                    switch (SOF.cmd)
                    {
                        case (MODE_RESET_ZERO):
                        case (MODE_CONF_WRITE):
                        case (MODE_CONF_LOAD):
                            *p_mode_flag = SOF.cmd;
                            for (int i = 0; i < SOF.data_len; i++)
                            {
                                // printf("0x%02X ", cdc_rx_message_data.data[i]);
                                p_motor_id->insert(cdc_rx_message_data.data[i]);
                            }
                            break;

                        case(MODE_SET_NUM):
                            *p_port_version = cdc_rx_message_data.data[2];
                            *p_port_version += (float)cdc_rx_message_data.data[3] * 0.1f;
                            break;

                        case(MODE_MOTOR_STATE):
                            for (size_t i = 0; i < SOF.data_len / sizeof(cdc_rx_motor_state_s); i++)
                            {
                                auto it = Map_Motors_p.find(cdc_rx_message_data.motor_state[i].id);
                                if (it != Map_Motors_p.end())
                                {
                                    it->second->fresh_data(cdc_rx_message_data.motor_state[i].pos,
                                                        cdc_rx_message_data.motor_state[i].val,
                                                        cdc_rx_message_data.motor_state[i].tqe);
                                }
                                else
                                {
                                    // ROS_ERROR("id not find");
                                }
                            }
                            break;
                    }
                }
            }
            else
            {
                // ROS_ERROR("clcl");
            }
        }
    }
}

void lively_serial::send_2(cdc_tr_message_s *cdc_tr_message)
{
    cdc_tr_message->head.s.crc8 = Get_CRC8_Check_Sum(&(cdc_tr_message->head.data[1]), 3, 0xFF);
    cdc_tr_message->head.s.crc16 = crc_ccitt(0xFFFF, &(cdc_tr_message->data.data[0]), cdc_tr_message->head.s.len);

    // uint8_t *byte_ptr = (uint8_t *)&cdc_tr_message->head.s.head;
    // printf("send:\n");
    // for (size_t i = 0; i < cdc_tr_message->head.s.len + 7; i++)
    // {
    //     printf("0x%.2X ", byte_ptr[i]);
    // }
    // printf("\n\n");

    _ser.write((const uint8_t *)&cdc_tr_message->head.s.head, cdc_tr_message->head.s.len + sizeof(cdc_tr_message_head_s));
}
