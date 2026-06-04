#ifndef __CHASSISR_TASK_H
#define __CHASSISR_TASK_H

#include "main.h"
#include "dm4310_drv.h"




typedef struct
{
  Joint_Motor_t joint_motor[10];
	
	uint8_t start_flag;//∆Ù∂Ø±Í÷æ

} chassis_t;


extern void ChassisR_init(chassis_t *chassis);
extern void ChassisR_task(void);

extern void mySaturate(float *in,float min,float max);


#endif




