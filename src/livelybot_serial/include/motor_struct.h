#ifndef _MOTOR_STRUCT_H_
#define _MOTOR_STRUCT_H_
#include <stdint.h>
#pragma pack(1)
typedef struct motor_back_struct
{
    double time;
    uint8_t ID;
    float position;
    float velocity;
    float torque;
} motor_back_t;
#pragma pack()
#endif