#ifndef _AXIS_H
#define _AXIS_H

#include "rp_config.h"
#include "axis_motor.h" 
#include "FreeRTOS.h"
#include "semphr.h"

/**
 *@brief 轴目标位置设置
 *
 */
typedef enum
{
	AXIS_INIT_POSITION,
	AXIS_FIRST_POSITION,
	AXIS_SECOND_POSITION,
	AXIS_THIRD_POSITION,
	AXIS_FOURTH_POSITION,
}axis_motor_target_e;

/**
 *@brief 轴任务步骤
 *
 */
typedef enum
{
	AXIS_MOVE_TASK_DONE,                     //轴移动任务完成
	AXIS_MOVE_TASK_START,                    //轴移动任务开始
	AXIS_MOVE_TASK_WAIT_MOVE_TO_VERSION,     //轴移动到视觉位置
	AXIS_MOVE_TASK_SET_OFFSET,               //轴设置移动偏置
	AXIS_MOVE_TASK_WAIT_MOVE_TO_SET_OFFSET,       
}axis_move_task_step_e;

/**
 *@brief 轴模块状态
 *
 */
typedef enum
{
	AXIS_OFFLINE,
	AXIS_FREE,
	AXIS_MOVING,
}axis_state_e;

/**
 *@brief 轴模块标志位
 *
 */
typedef struct axis_task_arg
{
  uint8_t is_vision_on;   //视觉开启标志
  uint8_t is_axis_delay;  //延迟标志
}axis_task_arg_t;

/**
 *@brief 轴模块任务信息
 *
 */
typedef struct axis_task_info
{
  axis_move_task_step_e move_task_step; //轴移动任务步骤
  axis_task_arg_t axis_task_arg;        //
  eTaskState AxisTaskState;             //任务状态
} axis_task_info_t;

typedef enum
{
	AXIS_REASSIGN_DONE,
	AXIS_REASSIGN_START,
	AXIS_REASSIGN_WAIT_TO_STICKED,
	AXIS_REASSIGN_WAIT_TO_TARGET,
}axis_reassign_task_step_e;

/**
 *@brief 轴电机信息
 *
 */
typedef struct axis_motor_info
{
  int32_t target_position; // 目标位置
  int16_t target_speed;
  int32_t current_position; // 当前位置
	
	int32_t axis_reassign_position;
	int32_t axis_sticked_position;
	uint8_t axis_reassign_flag;

	int32_t load_position;
	uint32_t load_flag;
	uint32_t load_ok_flag;
	
  int16_t output_current; // 输出电流
  motor_ctrl_mode_e ctrl_mode; // 控制模式

  int16_t vision_move_init_pos; //视觉对位后的位置

  float vision_Kp; // 视觉PID比例系数
} axis_motor_info_t;

/**
 *@brief 轴偏置信息
 *
 */
typedef struct axis_config
{
  int32_t axis_all_offset; // 所有轴偏移
  int32_t axis_first_offset; // 第一个位置
  int32_t axis_second_offset; // 第二个位置
  int32_t axis_third_offset; // 第三个位置
  int32_t axis_fourth_offset; // 第四个位置4
} axis_config_t;

/**
 * @brief 轴模块信息
 * 
 */
typedef struct axis_class_t
{	
	/*注册电机*/
	Motor_RM_t *axis_motor;

	axis_motor_info_t motor_info; // 电机信息
	axis_task_info_t task_info; // 轴任务信息
	axis_state_e state; // 轴状态
	axis_config_t config; // 轴配置(偏置信息)
	
	axis_reassign_task_step_e axis_reassign_task_step;
	
	void     (*work)(struct axis_class_t *axis);

}axis_t;

extern axis_t axis;
void Axis_Work(axis_t *axis);
void Axis_Reassign_Task(axis_t *axis);
HAL_StatusTypeDef Axis_Task_Start(void);


#endif
