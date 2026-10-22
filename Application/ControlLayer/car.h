#ifndef _CAR_H
#define _CAR_H

#include "rp_config.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "main.h"
#include "judge_protocol.h"
#include "RM_motor.h"
#include "DM_Motor.h"

#define RC_OFF_PROTECTION 1
#define CAR_DEBUG_MODE_ENABLE 1 // 车控制调试开关(用于6020供弹调试）
//#define OUTPOST_ALIVE_CHECK

/**
 * @brief 车控制模式枚举
 * 
 */
typedef enum 
{
  CAR_SLEEP_MODE,     //休眠模式
  CAR_INIT_MODE,
  CAR_MT_MODE,      //手动波箱
  CAR_AMT_MODE,     //手自一体波箱
  CAR_AT_MODE,      //自动波箱 
}car_ctrl_mode_e;

/**
 * @brief 波轮状态枚举
 * 
 */
typedef enum{
  CAR_WHEEL_KEEP,
  CAR_WHEEL_UP,
  CAR_WHEEL_DOWN,
  CAR_WHEEL_HOLD_DOWN,
}car_wheel_status_e;

/**
 * @brief 裁判系统飞镖舱门状态
 * 
 */
typedef enum{
  CAR_LAUNCH_ON = 0,
  CAR_LAUNCH_OFF,
  CAR_LAUNCH_OPENING,
  CAR_LAUNCH_CLOSING,
}car_launch_opening_status_e;


/**
 * @brief 比赛模式任务状态枚举
 * 
 */
typedef enum{
  CAR_AT_STEP_INIT_STATE_CHECK,
  CAR_AT_STEP_GAME_IN_CHECKING,
  CAR_AT_STEP_WAIT_FOR_CMD,
  CAR_AT_STEP_LAUNCH_OPENING,
  CAR_AT_STEP_LAUNCH_ON,
  CAR_AT_STEP_LAUNCH_DONE,
} car_at_step_e;

//typedef enum{
//	FEED_SERVO_CONTROL_OFF,
//	FEED_SERVO_CONTROL_ON,
//}car_feed_servo_control_state_e;

/**
 * @brief 车任务状态枚举
 * 
 */
typedef struct car_task_info
{
  car_at_step_e at_step; 								// 自动波箱步骤
  uint8_t first_widows_total_shoots; 					// 首次窗口总发射次数
  eTaskState FourShotStatic_TaskState;
  eTaskState FourShotDynamic_TaskState;
  eTaskState SingleShotDynamic_TaskState;
  eTaskState SingleShotStatic_TaskState;
  eTaskState TotalTaskState;
}car_task_info_t;


typedef struct 
{
  dev_work_state_t judge_state;
  game_progress_e game_progress;						//比赛进程状态
  car_launch_opening_status_e launch_opening_status; 	// 飞镖发射站状态
  dart_target_type_e dart_target_type;					//飞镖目标状态
  uint8_t dart_remaining_time;						// 己方飞镖发射剩余时间，单位：秒
  uint32_t launch_open_tick;
  uint32_t my_dart_open_time;
} car_judge_info_t;


typedef struct
{
  dev_work_state_t total_state;
  dev_work_state_t rc_state; 							// 遥控器状态
  dev_work_state_t motor3508L_state; 						// 3508电机状态
  dev_work_state_t motor3508R_state; 						// 3508电机状态
  dev_work_state_t motor6020_state; 						// 供弹6020电机状态
  dev_work_state_t motorAxis_state; 						// 轴电机状态
  dev_work_state_t motorForce_state; 						// 力矩电机状态
  dev_work_state_t vision_state; 						// 视觉状态
  dev_work_state_t judge_state; 						// 裁判系统状态
  // dev_work_state_t imu_state;         // IMU status
} car_module_state_t;



/**
 * @brief car类结构体
 * 
 */
typedef struct
{
	car_ctrl_mode_e  car_ctrl_mode;
	car_wheel_status_e wheel_status;
	car_task_info_t task_info;
	uint8_t target_id; 									// 目标id
	uint8_t reassign_falg;
//	uint8_t is_round_two;

	car_judge_info_t judge_info;
	car_module_state_t module_state; 						// 模块状态
//	car_feed_servo_control_state_e car_servo_control;
}car_t;


extern car_t car;

void Car_Ctrl(car_t *car);








#endif
