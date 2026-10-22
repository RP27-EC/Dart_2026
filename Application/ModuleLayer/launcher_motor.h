/**
 * @file launcher_motor.h
 * @author 
 * @brief 
 * @version 0.1
 * @date 2025-12-22
 * 
 * 
 */
 
#ifndef _LAUNCHER_MOTOR_H
#define _LAUNCHER_MOTOR_H

#include "rp_config.h"
#include "RM_motor.h"
#include "tim.h"
 
#define ID_LAUNCHER_RM_3508L   	    0x201                           
#define ID_LAUNCHER_RM_3508R   	  	0x202
#define ID_LOAD_RM_6020				0x205


/**
 *
 *brief 供弹电机
 */
typedef enum
{
	LAUNCHER_MOTOR_RM3508L,
	LAUNCHER_MOTOR_RM3508R,	
	FEEDING_MOTOR_RM6020,
	
	LAUNCHER_MOTOR_LIST,
	
}launcher_motor_list_e;

/**
 *@brief 舵机结构体
 */
typedef struct{
	uint16_t sleep;      //睡眠模式
	uint16_t lock;       //锁定模式
	uint16_t release;    //释放模式
	uint16_t shoot_target;
}servo_config_t;

//feeding_second_servo
void servo_feeding2_1_sleep(void);
void servo_feeding2_1_lock(void);
void servo_feeding2_1_release(void);
void servo_feeding2_1_shoot(void);
//void servo_feeding2_2_sleep(void);
//void servo_feeding2_2_lock(void);
//void servo_feeding2_2_release(void);

//feeding_thrid_servo
void servo_feeding3_1_sleep(void);
void servo_feeding3_1_lock(void);
void servo_feeding3_1_release(void);
void servo_feeding3_1_shoot(void);
//void servo_feeding3_2_sleep(void);
//void servo_feeding3_2_lock(void);
//void servo_feeding3_2_release(void);

//feeding_fourth_servo
void servo_feeding4_1_sleep(void);
void servo_feeding4_1_lock(void);
void servo_feeding4_1_release(void);
void servo_feeding4_1_shoot(void);
//void servo_feeding4_2_sleep(void);
//void servo_feeding4_2_lock(void);
//void servo_feeding4_2_release(void);

// 发射舵机
void servo_shoot_release(void);
void servo_shoot_lock(void);
void servo_shoot_sleep(void);

void electromagnet2_on(void);
void electromagnet2_off(void);
void electromagnet3_on(void);
void electromagnet3_off(void);
void electromagnet4_on(void);
void electromagnet4_off(void);


extern Motor_RM_t launcher_motor[LAUNCHER_MOTOR_LIST];
extern servo_config_t launcher_shootservo_config;
extern servo_config_t feeding_secondservo_config;
extern servo_config_t feeding_thirdservo_config;
extern servo_config_t feeding_fourthservo_config;

void launcher_motor_init(void);

void launcher_motor_heart_beat(void); 
bool motor_stick_check(Motor_RM_t* motor);

#endif
