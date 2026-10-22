#ifndef _AXIS_MOTOR_H
#define _AXIS_MOTOR_H

#include "rp_config.h"
#include "RM_motor.h"

#define ID_AXIS_RM_2006		0x204

typedef enum
{
	AXIS_MOTOR,

	AXIS_MOTOR_LIST,
}axis_motor_list_e;	

extern Motor_RM_t axis_motor[AXIS_MOTOR_LIST];
void axis_motor_init(void);
void axis_motor_heart_beat(void);
bool axis_motor_stick_check(Motor_RM_t *motor);

#endif
