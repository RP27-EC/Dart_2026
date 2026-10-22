#ifndef _FORCE_H
#define _FORCE_H

#include "rp_config.h"
#include "force_motor.h"
#include "FreeRTOS.h"
#include "semphr.h"

/**
 *@brief 电机目标枚举
 *
 */
typedef enum
{
  FORCE_INIT_POSITION,
  FORCE_FIRST_POSITION,
  FORCE_SECOND_POSITION,
  FORCE_THIRD_POSITION,
  FORCE_FOURTH_POSITION,
} force_motor_target_e;

/*
 *@brief 变力电机任务枚举
 *
 */
typedef enum{
  FORCE_MOVE_TASK_DONE, // 轴移动务完成
  FORCE_MOVE_TASK_START, // 轴移动任务开始
  FORCE_MOVE_TASK_SET_OFFSET, // 设置轴目标位置
  FORCE_MOVE_TASK_WAIT_MOVE_TO_OFFSET, // 任等待轴移动
} force_move_task_step_e;


typedef enum
{
	FORCE_OFFLINE,
	FORCE_FREE,
	FORCE_MOVING,
} force_state_e;


typedef struct force_task_info
{
	force_move_task_step_e move_task_step; // 轴移动任务步骤
	eTaskState ForceTaskState;
} force_task_info_t;

typedef enum
{
	FORCE_REASSIGN_DONE,
	FORCE_REASSIGN_START,
	FORCE_REASSIGN_WAIT_TO_STICKED,
	FORCE_REASSIGN_WAIT_TO_TARGET,
}force_reassign_task_step_e;

typedef struct force_motor_info
{
	int32_t target_position; // 目标位置
	int16_t target_speed;
	int32_t current_position; // 当前位置
	
	int32_t force_reassign_position;
	int32_t force_sticked_position;
	uint8_t force_reassign_flag;
	
	int16_t output_current; // 输出电流
	motor_ctrl_mode_e ctrl_mode; // 控制模式
	
} force_motor_info_t;

typedef struct force_config
{
	int32_t force_all_offset; // 所有偏移
	int32_t force_first_offset; // 第一个位置
	int32_t force_second_offset; // 第二个位置
	int32_t force_third_offset; // 第三个位置
	int32_t force_fourth_offset; // 第四个位置
} force_config_t;


/**
 * @brief 轴类信息
 * 
 */
typedef struct force_class_t
{	
	/*注册电机*/
	Motor_RM_t *force_motor;

	force_motor_info_t motor_info; // 电机信息
	force_task_info_t task_info; // 轴任务信息
	force_state_e state; // 轴状态
	force_config_t config; // 轴配置

	force_reassign_task_step_e force_reassign_task_step;
	
	void     (*work)(struct force_class_t *force);

}force_t;


extern force_t force;


/* Exported functions --------------------------------------------------------*/

void Force_Work(force_t *force);
void Force_Reassign_Task(force_t *force);
HAL_StatusTypeDef Force_Task_Start(void);



#endif
