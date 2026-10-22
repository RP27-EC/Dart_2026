#ifndef __DEVICE_H
#define __DEVICE_H

/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"
#include "RM_motor.h"
#include "DM_Motor.h"
#include "launcher_motor.h"
#include "axis_motor.h"
#include "rc_sensor.h"
#include "force_motor.h" 
// #include "imu_sensor.h"
/* Exported macro ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/


/* Exported functions --------------------------------------------------------*/
typedef enum{
	DEVICE_01,
	DEVICE_02,
	DEVICE_03,
	DEVICE_04,
	DEVICE_05,
	// DEVICE_06,
	// DEVICE_07,
	// DEVICE_08,
	// DEVICE_09,
	// DEVICE_10,
	// DEVICE_11,
	// DEVICE_12,
	DEVICE_CNT,
}device_cnt_e;


typedef struct{
	Motor_RM_t						*motor;
	motor_state_e   	work_sate;
}device_t;


extern device_t device[DEVICE_CNT];

void DEVICE_Init(void);
//void Device_Init(void);
void Device_HeartBeat(void);
void Device_Work(void);




#endif

