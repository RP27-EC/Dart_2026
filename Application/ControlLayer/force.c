#include "force.h"
#include "device.h"
#include "rp_math.h"
#include "car.h"
#include "tuner.h"

static TaskHandle_t xForceTaskHandle;


/**
 *@brief 变力电机定义
 *
 */
force_t force=
{
	.force_motor=&force_motor[FORCE_MOTOR],
	.config ={
		.force_all_offset=0,
		.force_first_offset=0,
		.force_second_offset=0,
		.force_third_offset=0,
		.force_fourth_offset=0,
	},
	.work = Force_Work,
	.motor_info ={
		.force_reassign_position = 4786247
	},
};

/**
 * @brief 发射PID计算
 *
 * @param force
 */
void Force_Pid_Calculating(force_t *force)
{
	switch (force->motor_info.ctrl_mode)
	{
	case MOTOR_UNFORCE:
		force->motor_info.output_current = 0;
		break;
	case MOTOR_PID_POSITION:
		force->force_motor->ctrl->angle_ctrl_outer->target = force->motor_info.target_position;
		force->force_motor->ctrl->angle_ctrl_outer->measure = force->force_motor->rx_info->encoder_sum;
		force->force_motor->ctrl->angle_ctrl_outer->err=force->force_motor->ctrl->angle_ctrl_outer->target-force->force_motor->ctrl->angle_ctrl_outer->measure;
		single_pid_ctrl(force->force_motor->ctrl->angle_ctrl_outer);
		force->force_motor->ctrl->angle_ctrl_inner->target=force->force_motor->ctrl->angle_ctrl_outer->out;
		force->force_motor->ctrl->angle_ctrl_inner->measure = force->force_motor->rx_info->encoder_speed;
		force->force_motor->ctrl->angle_ctrl_inner->err=force->force_motor->ctrl->angle_ctrl_inner->target-force->force_motor->ctrl->angle_ctrl_inner->measure;
		single_pid_ctrl(force->force_motor->ctrl->angle_ctrl_inner);
		force->motor_info.output_current=force->force_motor->ctrl->angle_ctrl_inner->out;
		break;
  case MOTOR_PID_SPEED:
	  force->force_motor->ctrl->speed_ctrl->target  = -force->motor_info.target_speed;
		force->force_motor->ctrl->speed_ctrl->measure = force->force_motor->rx_info->encoder_speed;  
		force->force_motor->ctrl->speed_ctrl->err=force->force_motor->ctrl->speed_ctrl->target-force->force_motor->ctrl->speed_ctrl->measure;
		single_pid_ctrl(force->force_motor->ctrl->speed_ctrl);
		force->motor_info.output_current=force->force_motor->ctrl->speed_ctrl->out;
		force->force_motor->tx_info->torque_current = force->force_motor->ctrl->speed_ctrl->out;
    break;
	default:
		break;
	}
}

/**
 * @brief force复位
 * 
 */
void Force_Reset(force_t *force)
{
  force->state = FORCE_OFFLINE; // 轴状态设置为离线
  force->task_info.move_task_step = FORCE_MOVE_TASK_DONE;

  force->motor_info.target_position = 0; // 目标位置归零
  force->motor_info.output_current = 0; // 输出电流归零
  force->motor_info.target_speed = 0; // 目标速度归零
  force->motor_info.ctrl_mode = MOTOR_UNFORCE; 
}

/**
 * @brief 電機位置更新
 * 
 * @param force 
 */
void Force_External_Info_Update(force_t *force)
{
  // 轴电机位置计算s
  force->motor_info.current_position = force->force_motor->rx_info->encoder_sum;

  tuner_flash_info_t *flash_info = &tuner.tuner_read_flash_info;
  force->config.force_all_offset = flash_info->force_all_offset; 
  force->config.force_first_offset = flash_info->force_first_offset;
  force->config.force_second_offset = flash_info->force_second_offset;
  force->config.force_third_offset = flash_info->force_third_offset;
  force->config.force_fourth_offset = flash_info->force_fourth_offset;
}

/**
 *@brief force任务函数
 *
 */
void Force_Offset_Move_Task(force_t *force, int32_t offset)
{
  int32_t offset_reference_position = 0;
	int32_t pos_err;
  switch (force->task_info.move_task_step)
  {
  case FORCE_MOVE_TASK_START:
    force->task_info.move_task_step = FORCE_MOVE_TASK_SET_OFFSET; // 设置任务步骤为设置目标位置
    break;
  case FORCE_MOVE_TASK_SET_OFFSET:
    force->motor_info.target_position = offset_reference_position + offset;
    force->task_info.move_task_step = FORCE_MOVE_TASK_WAIT_MOVE_TO_OFFSET; // 设置任务步骤为完成
    break;
  case FORCE_MOVE_TASK_WAIT_MOVE_TO_OFFSET:
    pos_err = abs(force->motor_info.current_position - force->motor_info.target_position);
    if (pos_err < 3000 && abs(force->force_motor->rx_info->speed) < 100)
    {
      force->task_info.move_task_step = FORCE_MOVE_TASK_DONE; 
    }
    break;
    case FORCE_MOVE_TASK_DONE:
    break;
  default:
    break;
  }

}

/**
 *@brief 任务状态更新
 *
 */
void Force_TaskStatus_Update(force_t *force)
{
  if (xForceTaskHandle != NULL)
  {
    force->task_info.ForceTaskState = eRunning; // 任务状态设置为运行中
  }
  else
  {
    force->task_info.ForceTaskState = eDeleted; // 任务状态设置为已删除
  }
  
}

/**
 *@brief 轴堵转任务
 *
 */
void Force_Reassign_Task(force_t *force)
{
	switch(force->force_reassign_task_step)
	{
		case FORCE_REASSIGN_START:
			force->motor_info.force_reassign_flag = 0;
			force->motor_info.ctrl_mode = MOTOR_PID_SPEED;
			force->motor_info.target_speed = 1500;
			force->force_reassign_task_step = FORCE_REASSIGN_WAIT_TO_STICKED;
			break;
		case FORCE_REASSIGN_WAIT_TO_STICKED:
			if(force_motor_stick_check(force->force_motor) == true)
			{
				force->motor_info.ctrl_mode = MOTOR_PID_POSITION;
				force->motor_info.force_sticked_position = force->motor_info.current_position;
				force->motor_info.target_position = force->motor_info.current_position + force->motor_info.force_reassign_position;
				force->force_reassign_task_step = FORCE_REASSIGN_WAIT_TO_TARGET;
			}
			break;
		case FORCE_REASSIGN_WAIT_TO_TARGET:
			if(abs(force->motor_info.target_position-force->motor_info.current_position)<2000)
			{
				force->motor_info.force_reassign_flag = 1;
				force->force_reassign_task_step =FORCE_REASSIGN_DONE;
			}
			break;
		case FORCE_REASSIGN_DONE:			
			break;
		default:
			break;
	}
}

/**
 *@brief 任务管理
 *
 */
void Force_TaskManager(void *pvParameters)
{
  uint8_t target_id = car.target_id;
  int32_t all_offset = force.config.force_all_offset; // 所有偏移
  int32_t offset = 0;
  force.motor_info.ctrl_mode = MOTOR_PID_POSITION;
  force.state = FORCE_MOVING;
  switch (target_id)
  {
  case 1:
    offset = force.config.force_first_offset + all_offset;
    break;
  case 2:
    offset = force.config.force_second_offset + all_offset;
    break;
  case 3:
    offset = force.config.force_third_offset + all_offset;
    break;
  case 4:
    offset = force.config.force_fourth_offset + all_offset;
		break;
  default:
    force.task_info.move_task_step = FORCE_MOVE_TASK_DONE;
    force.state = FORCE_FREE;
    xForceTaskHandle = NULL;
    vTaskDelete(NULL);
    break;
  }

  force.task_info.move_task_step = FORCE_MOVE_TASK_START; //重置
  for(;;)
  {		
    if(car.car_ctrl_mode < CAR_MT_MODE){
      xForceTaskHandle = NULL;
			vTaskDelete(NULL);
		}

    Force_Offset_Move_Task(&force, offset);
    if (force.task_info.move_task_step == FORCE_MOVE_TASK_DONE)
    {
      force.state = FORCE_FREE;
      xForceTaskHandle = NULL;
      vTaskDelete(NULL);
    }
  }

}


HAL_StatusTypeDef Force_Task_Start(void){

	// 检查任务状态
	if (xForceTaskHandle != NULL) {
    // 任务还在
    return HAL_BUSY;
	}
	// 任务已删除
	if (xTaskCreate(Force_TaskManager, 
									"ForceTask", 1028, 
									NULL, tskIDLE_PRIORITY + 2, 
									&xForceTaskHandle) != pdPASS) {
		return HAL_ERROR;
	}	
	else
	{
    force.task_info.ForceTaskState = eRunning; // 任务状态设置为运行中
		return HAL_OK;
	}
}

/**
 * @brief 轴总控
 *
 * @param force
 */
void Force_Work(force_t *force)
{
	Force_TaskStatus_Update(force);
	Force_External_Info_Update(force); // 计算轴电机位置

	if (car.car_ctrl_mode == CAR_SLEEP_MODE)
	{
		Force_Reset(force); // 如果没有开控，则重置轴
	}
	else
	{
		if (force->state == FORCE_OFFLINE)
		{
		force->state = FORCE_FREE;
		}
	}
	if (car.car_ctrl_mode != CAR_SLEEP_MODE) // 开控
	{
		Force_Pid_Calculating(force);
//		force->force_motor->base_info.motor_out = force->motor_info.output_current;
	}
}

