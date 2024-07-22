#ifndef __MOTOR_CTL_H
#define __MOTOR_CTL_H

#include "delay.h"

#define Motor1_EN GPIO_Pin_0
#define Motor2_EN GPIO_Pin_1
#define Motor1_Dir GPIO_Pin_3
#define Motor2_Dir GPIO_Pin_4
#define Motor1_Speed GPIO_Pin_9
#define Motor2_Speed GPIO_Pin_10

void Motor_Init(void);
void continuous(unsigned int motor1_dir, unsigned int motor2_dir);
void point_mode(unsigned int motor1_dir, unsigned int motor2_dir);
void motor_stop(void);

#endif
