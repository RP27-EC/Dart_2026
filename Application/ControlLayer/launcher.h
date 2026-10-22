#ifndef _LAUNCHER_H
#define _LAUNCHER_H

#include "rp_config.h"
#include "rp_device_config.h"
#include "launcher_motor.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "car.h"

typedef enum
{
	LAUNCHER_SUBMODULE_OFFLINE,		 // 离线
	LAUNCHER_SUBMODULE_INITIALIZING, // 正在初始化
	LAUNCHER_SUBMODULE_INITED,		 // 完成初始化
	LAUNCHER_SUBMODULE_MOVING,		 // 正在移动中
	LAUNCHER_SUBMODULE_DONE,		 // 动作完成
} launcher_submodule_state_e;

/**
 *@brief 3508(推板/上膛)电机状态枚举
 */
typedef enum
{
	LAUNCHER_3508_UP, // 推板在前
	LAUNCHER_3508_MIDDLE,
	LAUNCHER_3508_DOWN, // 推板在后，上膛
	LAUNCHER_3508_UNFORCE,
} launcher_3508_state_e;

/**
 *@brief 6020(供弹体)电机状态枚举
 */
typedef enum
{
	LAUNCHER_6020_SECOND_STANDBY, // 第二发准备
	LAUNCHER_6020_SECOND_LOAD,	  // 第二发填充
	LAUNCHER_6020_THIRD_STANDBY,  // 第三发准备
	LAUNCHER_6020_THIRD_LOAD,	  // 第三发填充
	LAUNCHER_6020_FOURTH_STANDBY, // 第四发准备
	LAUNCHER_6020_FOURTH_LOAD,	  // 第四发填充

} launcher_6020_state_e;

/**
 *@brief 发射模块内舵机状态枚举
 */
typedef enum
{
	LAUNCHER_SERVO_SLEEP,
	LAUNCHER_SERVO_LOCK,
	LAUNCHER_SERVO_RELEASE,
	LAUNCHER_SERVO_SHOOT_TARGET,
} launcher_servo_state_e;

/**
 *@brief 微动开关状态枚举
 */
typedef enum
{
	LAUNCHER_MICROSWITCH_ON,
	LAUNCHER_MICROSWITCH_OFF,
} launcher_microswitch_tate_e;

/**
 *@brief 微动开关信息结构体
 *
 */
typedef struct
{
	launcher_microswitch_tate_e microswitch_last_state;
	launcher_microswitch_tate_e microswitch_state;
	uint16_t min_keep_cnt;
} launcher_microswitch_info_t;

/**
 *@brief 发射模块状态枚举
 */
typedef enum
{
	LAUNCHER_OFFLINE,
	LAUNCHER_INITIALIZING,
	LAUNCHER_FREE,

	LAUNCHER_LOADING,
	LAUNCHER_LOADED,
	LAUNCHER_SHOOTING,
} launcher_state_e;

/**
 *@brief 发射模块初始化状态枚举
 */
typedef enum
{
	LAUNCHER_INIT_START, // 设置6020电机复位和舵机复位
	LAUNCHER_INIT_STEP_WAIT_FEEDING_SERVO_TO_RELEASE,
	LAUNCHER_INIT_STEP_WAIT_6020_TO_STANDBY,		// 检查6020是否到准备位置
	LAUNCHER_INIT_STEP_WAIT_SHOOT_SERVO_TO_RELEASE, // 设置3508电机复位速度，
	LAUNCHER_INIT_STEP_WAIT_3508_TO_RELEASE,		// 检查3508电机是否复位
	LAUNCHER_INIT_DONE,								// 发射模块初始化完成
} launcher_initializing_state_e;

/**
 *@brief 装弹状态枚举
 */
typedef enum
{
	LAUNCHER_LOAD_START, // 设置6020到装弹位置
	// LAUNCHER_WAIT_6020_TO_LOAD,		   // 等待6020到位，设置3508上膛
	LAUNCHER_WAIT_6020_TO_LOAD,		   // 等待6020到位，设置3508上膛
	LAUNCHER_SET_3508_TO_DOWN,		   // 设置3508上膛
	LAUNCHER_WAIT_3508_TO_DOWN,		   // 等待3508上膛,设置发射舵机（撒放器）锁定
	LAUNCHER_WAIT_SHOOT_SERVO_TO_LOCK, // 等待发射舵机（撒放器）锁定，设置3508电机复位，设置舵机装弹
	LAUNCHER_WAIT_3508_TO_MIDDLE,
	LAUNCHER_WAIT_3508_TO_UP_FEEDING_SERVO_TO_TARGET, // 等待3508电机复位，等待臂舵机到位，
	LAUNCHER_SET_ELECTROMAGNET_OFF,					  // 设置电磁铁断电
	LAUNCHER_SET_FEEDING_SERTVO_TO_RELEASE,			  // 设置舵机复位
	LAUNCHER_WAIT_FEEDING_SERVO_TO_RELEASE,			  // 等待舵机复位，设置6020到下一个准备位置
	LAUNCHER_WAIT_6020_TO_STANDBY,					  // 等待6020到准备位置
	LAUNCHER_WAIT_SERVO_TO_TARGET,
	LAUNCHER_LOAD_OK, // 装弹完成
} launcher_load_state_e;

/**
 *@brief 发射模块内电磁铁状态枚举
 *
 */
typedef enum
{
	LAUNCHER_ELECTROMAGNET_ON,	// 电磁铁开
	LAUNCHER_ELECTROMAGNET_OFF, // 电磁铁关

} launcher_electromagnet_state_e;

/**
 *@brief 发射状态枚举
 */
typedef enum
{
	LAUNCHER_SHOOT_START,
	LAUNCHER_SET_SHOOT_SERVO_RELEASE,
	LAUNCHER_WAIT_SHOOT_SERVO_TO_RELEASE,
	LAUNCHER_FEEDING_SERVO_BACK,
	LAUNCHER_SHOOT_OK,
} launcher_shoot_state_e;

/**
 *@brief 发射任务标志位结构体
 */
typedef struct launcher_task_flag
{
	uint8_t Shoot_Flag;
	uint8_t Load_Flag;
} launcher_task_flag_t;

/**
 *@brief 发射任务结构体
 */
typedef struct
{
	launcher_initializing_state_e launcher_initializing_state;
	launcher_load_state_e launcher_load_state;
	launcher_shoot_state_e launcher_shoot_state;
	launcher_task_flag_t launcher_task_flag;
	eTaskState LauncherTaskState;
} launcher_task_t;

typedef struct
{
	Motor_RM_t *motor6020;
	Motor_RM_t *motor3508L;
	Motor_RM_t *motor3508R;
} launcher_motor_group_t;

/**
 * @brief  发射6020基本信息定义
 */
typedef struct launcher_motor6020_info
{
	int16_t output_current; // 供弹6020电机输出电流
	int16_t target_angle;	// 供弹6020电机目标角度
	int16_t target_speed;

	launcher_submodule_state_e state;	// 供弹6020电机状态
	launcher_6020_state_e target_state; // 供弹6020电机目标

	motor_ctrl_mode_e ctrl_mode;

	float target_angle_smoothed; // 平滑后的目标角度，实际送入PID
} launcher_motor6020_info_t;

/**
 * @brief  上膛3508基本信息定义
 */
typedef struct launcher_motor3508_info
{
	int16_t motor3508L_output_current; // 左3508电机输出电流
	int16_t motor3508R_output_current; // 右3508电机输出电流

	int32_t motor3508R_target_position; // 右3508电机目标位置
	int32_t motor3508L_target_position; // 左3508电机目标位置

	int16_t target_speed; // 上膛3508电机目标速度

	int32_t motor3508R_init_angle_sum;
	int32_t motor3508R_position; // 右3508电机位置
	int32_t motor3508L_init_angle_sum;
	int32_t motor3508L_position; // 左3508电机位置

	launcher_submodule_state_e state;	// 发射模块电机状态
	launcher_3508_state_e target_state; // 上膛3508电机状

	motor_ctrl_mode_e ctrl_mode; // 上膛3508电机控制模式

	int32_t err_between_3508;

} launcher_motor3508_info_t;

/**
 * @brief  6020上舵机信息定义
 */
typedef struct launcher_feedServo_info
{
	launcher_servo_state_e second_servo1_target;
	launcher_servo_state_e third_servo1_target;
	launcher_servo_state_e fourth_servo1_target;
	//	launcher_servo_state_e second_servo2_target;
	//	launcher_servo_state_e third_servo2_target;
	//	launcher_servo_state_e fourth_servo2_target;
} launcher_feedServo_info_t;

typedef enum
{
	SECOND_SERVO,
	THIRD_SERVO,
	FOURTH_SERVO,
} feeding_target_servo_e;

/**
 *@brief   电磁铁信息定义
 *
 */
typedef struct launcher_electromagnet_info
{
	launcher_electromagnet_state_e second_electromagnet_target;
	launcher_electromagnet_state_e third_electromagnet_target;
	launcher_electromagnet_state_e fourth_electromagnet_target;
} launcher_electromagnet_info_t;

/**
 * @brief  发射舵机信息定义
 *
 */
typedef struct launcher_fireServo_info
{
	launcher_servo_state_e shoot_servo_target;
} launcher_fireServo_info_t;

typedef struct launcher_config_t
{
	int16_t motor6020_standby_angle[3]; // 供弹6020准备角度
	int16_t motor6020_load_angle[3];	// 供弹6020加载角度
	int16_t motor6020_angle_ramp_rate;	// 6020目标角度平滑速率 (counts/ms)，0=禁用

	int16_t motor3508_initmaxspeed;
	int16_t motor3508_firstmaxspeed;
	int16_t motor3508_to_middle_maxspeed;
	int16_t motor3508_othermaxspeed;
	int32_t motor3508_up_position;
	int32_t motor3508_middle_position;
	int32_t motor3508_down_position;
	int32_t motor3508_protect_position;

} launcher_config_t;

/**
 * @brief 发射类信息
 *
 */
typedef struct launcher_class_t
{
	/*注册电机*/
	launcher_motor_group_t motors;

	launcher_motor3508_info_t motor3508_info;
	launcher_motor6020_info_t motor6020_info;
	launcher_feedServo_info_t feedServo_info;
	launcher_fireServo_info_t fireServo_info;
	launcher_electromagnet_info_t electromagnet_info;
	launcher_microswitch_info_t microswitch_info;

	launcher_config_t config;

	launcher_task_t task_info; // 发射任务信息

	launcher_state_e state; // 发射状态

	feeding_target_servo_e feeding_target_servo;

	void (*work)(struct launcher_class_t *launcher);

} launcher_t;

extern launcher_t launcher;
extern TaskHandle_t xLauncherTaskHandle;
extern TaskHandle_t xLauncherGymTaskHandle;
void mircoswitch_check(void);
void Laucher_Init_Task(launcher_t *launcher);
HAL_StatusTypeDef Launcher_Task_Start(void);
uint16_t Launcher_6020_Init_Angle_Cal(launcher_t *launcher);
void Launcher_Motor6020_State_Controler(launcher_t *launcher);
void Launcher_Motor3508_State_Controler(launcher_t *launcher);
#endif
