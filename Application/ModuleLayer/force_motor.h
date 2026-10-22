#ifndef _FORCE_MOTOR_H
#define _FORCE_MOTOR_H

#include "rp_config.h"
#include "RM_motor.h"

#define ID_FORCE_RM_3508	0x203

typedef enum
{
	FORCE_MOTOR,

	FORCE_MOTOR_LIST,
}force_motor_list_e;	

extern Motor_RM_t force_motor[FORCE_MOTOR_LIST];
void force_motor_init(void);
void force_motor_heart_beat(void);
bool force_motor_stick_check(Motor_RM_t *motor);

#endif
