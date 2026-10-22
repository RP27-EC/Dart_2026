#include "axis.h"
#include "rp_math.h"
#include "device.h"
#include "car.h"
#include "vision_protocol.h"
#include "tuner.h"

static TaskHandle_t xAxisTaskHandle;

axis_t axis =
    {
        .axis_motor = &axis_motor[AXIS_MOTOR],
        .work = Axis_Work,
        .motor_info = {
		.vision_Kp = -100.0f, // �Ӿ�PID����ϵ��
		.axis_reassign_position = -6203432,
		.load_position = 1500000 
        },
};

/**
 * @brief ����PID����
 *
 * @param axis
 */
void Axis_Pid_Calculating(axis_t *axis)
{
  float vision_error,vision_target_speed;
  switch (axis->motor_info.ctrl_mode)
  {
  case MOTOR_UNFORCE:
    axis->motor_info.output_current = 0;
    break;
  case MOTOR_PID_POSITION:
    axis->axis_motor->ctrl->angle_ctrl_outer->target = axis->motor_info.target_position;
    axis->axis_motor->ctrl->angle_ctrl_outer->measure = axis->axis_motor->rx_info->encoder_sum;
    axis->axis_motor->ctrl->angle_ctrl_outer->err = axis->axis_motor->ctrl->angle_ctrl_outer->target - axis->axis_motor->ctrl->angle_ctrl_outer->measure;
    single_pid_ctrl(axis->axis_motor->ctrl->angle_ctrl_outer);
    axis->axis_motor->ctrl->angle_ctrl_inner->target = axis->axis_motor->ctrl->angle_ctrl_outer->out;
    axis->axis_motor->ctrl->angle_ctrl_inner->measure = axis->axis_motor->rx_info->encoder_speed;
    axis->axis_motor->ctrl->angle_ctrl_inner->err = axis->axis_motor->ctrl->angle_ctrl_inner->target - axis->axis_motor->ctrl->angle_ctrl_inner->measure;
    single_pid_ctrl(axis->axis_motor->ctrl->angle_ctrl_inner);
    axis->motor_info.output_current = axis->axis_motor->ctrl->angle_ctrl_inner->out;
    axis->axis_motor->tx_info->torque_current =  axis->axis_motor->ctrl->angle_ctrl_inner->out;
    break;
  case MOTOR_PID_SPEED:
    axis->axis_motor->ctrl->speed_ctrl->target = -axis->motor_info.target_speed;
    axis->axis_motor->ctrl->speed_ctrl->measure = axis->axis_motor->rx_info->encoder_speed;
    axis->axis_motor->ctrl->speed_ctrl->err = axis->axis_motor->ctrl->speed_ctrl->target - axis->axis_motor->ctrl->speed_ctrl->measure;
    single_pid_ctrl(axis->axis_motor->ctrl->speed_ctrl);
    axis->motor_info.output_current = axis->axis_motor->ctrl->speed_ctrl->out;
    axis->axis_motor->tx_info->torque_current =  axis->axis_motor->ctrl->speed_ctrl->out;
    break;
  case MOTOR_PID_VISION:
    vision_error = vision.rx_info->yaw_error;
    vision_target_speed = vision_error * axis->motor_info.vision_Kp;
    axis->axis_motor->ctrl->vision_ctrl->target = vision_target_speed;
    axis->axis_motor->ctrl->vision_ctrl->measure = axis->axis_motor->rx_info->encoder_speed;
    axis->axis_motor->ctrl->vision_ctrl->err = axis->axis_motor->ctrl->vision_ctrl->target - axis->axis_motor->ctrl->vision_ctrl->measure;
    single_pid_ctrl(axis->axis_motor->ctrl->vision_ctrl);
	axis->motor_info.output_current =axis->axis_motor->ctrl->vision_ctrl->out;
    axis->axis_motor->tx_info->torque_current =  axis->axis_motor->ctrl->vision_ctrl->out;
    break;
  default:
    break;
  }
}

/**
 * @brief ���λ�ø���
 *
 * @param axis
 *
 */
void Axis_External_Info_Update(axis_t *axis)
{
  axis->motor_info.current_position = axis->axis_motor->rx_info->encoder_sum;

	tuner_flash_info_t *flash_info = &tuner.tuner_read_flash_info;
	axis->config.axis_all_offset = flash_info->axis_all_offset;
	axis->config.axis_first_offset = flash_info->axis_first_offset;
	axis->config.axis_second_offset = flash_info->axis_second_offset;
	axis->config.axis_third_offset = flash_info->axis_third_offset;
	axis->config.axis_fourth_offset = flash_info->axis_fourth_offset;
}

/**
 * @brief ��λ
 *
 * @param axis
 */
void Axis_Reset(axis_t *axis)
{
  axis->state = AXIS_OFFLINE;                           // ��״̬����Ϊ����
  axis->task_info.move_task_step = AXIS_MOVE_TASK_DONE; // ����������Ϊ���

  axis->motor_info.target_position = 0; // Ŀ��λ�ù���
  axis->motor_info.output_current = 0;  // �����������
  axis->motor_info.target_speed = 0;
  axis->motor_info.ctrl_mode = MOTOR_UNFORCE;
}

/**
 * @brief �΄ՠ�B����
 *
 * @param axis
 */
void Axis_TaskStatus_Update(axis_t *axis)
{
  if (xAxisTaskHandle != NULL)
  {
    axis->task_info.AxisTaskState = eRunning; // ����״̬����Ϊ������
  }
  else
  {
    axis->task_info.AxisTaskState = eDeleted; // ����״̬����Ϊ��ɾ��
  }
}

/**
 *@brief ���ת����
 *
 */
void Axis_Reassign_Task(axis_t *axis)
{
	switch(axis->axis_reassign_task_step)
	{
		case AXIS_REASSIGN_START:
			axis->motor_info.axis_reassign_flag = 0;
			axis->motor_info.ctrl_mode = MOTOR_PID_SPEED;
			axis->motor_info.target_speed = -2500;
			axis->axis_reassign_task_step = AXIS_REASSIGN_WAIT_TO_STICKED;
			break;
		case AXIS_REASSIGN_WAIT_TO_STICKED:
			if(axis_motor_stick_check(axis->axis_motor) == true)
			{
				axis->motor_info.ctrl_mode = MOTOR_PID_POSITION;
				axis->motor_info.axis_sticked_position = axis->motor_info.current_position;
				axis->motor_info.target_position =axis->motor_info.current_position + axis->motor_info.axis_reassign_position;
				axis->axis_reassign_task_step = AXIS_REASSIGN_WAIT_TO_TARGET;
			}
			break;
		case AXIS_REASSIGN_WAIT_TO_TARGET:
			if(abs(axis->motor_info.target_position-axis->motor_info.current_position)<2000)
			{
				axis->motor_info.axis_reassign_flag = 1;
				axis->axis_reassign_task_step =AXIS_REASSIGN_DONE;
			}
			break;
		case AXIS_REASSIGN_DONE:			
			break;
		default:
			break;
	}
}


/**
 *@brief ���yaw���ƶ�
 *        offset    ƫ��ֵ
 */
void Axis_move_task(axis_t *axis, int32_t offset)
{
   int32_t offset_reference_position = 0;
  int32_t pos_err;
  switch (axis->task_info.move_task_step)
  {
  case AXIS_MOVE_TASK_START:
    axis->task_info.move_task_step = AXIS_MOVE_TASK_SET_OFFSET;
    break;
  case AXIS_MOVE_TASK_SET_OFFSET:
    axis->motor_info.ctrl_mode = MOTOR_PID_POSITION; // ���ÿ���ģʽΪλ��PID
    axis->motor_info.target_position = offset_reference_position + offset;
    axis->task_info.move_task_step = AXIS_MOVE_TASK_WAIT_MOVE_TO_SET_OFFSET; // ����������Ϊ���
    break;
  case AXIS_MOVE_TASK_WAIT_MOVE_TO_SET_OFFSET:
    pos_err = abs(axis->motor_info.current_position - axis->motor_info.target_position);
    if (pos_err < 3000 && abs(axis->axis_motor->rx_info->speed) < 100)
    {
      axis->task_info.move_task_step = AXIS_MOVE_TASK_DONE;
    }
    break;
  case AXIS_MOVE_TASK_DONE:
    break;
  default:
    break;
  }
}

static uint32_t axis_cnt = 0;

void Axis_Offset_Move_Task(axis_t *axis, uint8_t is_vision_on, int32_t offset, uint8_t is_delay)
{
  static int32_t offset_reference_position = 0;
  int32_t pos_err;
  static uint32_t vision_start_tick = 0;
  switch (axis->task_info.move_task_step)
  {
  case AXIS_MOVE_TASK_START:
    if (is_delay == 1)
    {
      vTaskDelay(pdMS_TO_TICKS(400)); // �ȴ��ƶ��е�λ
    }
	if(car.judge_info.game_progress >= GAME_IN_CHECKING)
	{
		if (car.judge_info.launch_opening_status == CAR_LAUNCH_ON)
		{
			vision_start_tick = HAL_GetTick();
			axis->task_info.move_task_step = AXIS_MOVE_TASK_WAIT_MOVE_TO_VERSION; // ����������Ϊ����Ŀ��λ��
		}			
	}
	else
	{
		vision_start_tick = HAL_GetTick();	
		axis->task_info.move_task_step = AXIS_MOVE_TASK_SET_OFFSET;		
	}
	break;
  case AXIS_MOVE_TASK_WAIT_MOVE_TO_VERSION:
    if (is_vision_on == 1 && vision.status->rx_state == DEV_ONLINE) // �Ӿ�����
    {

		axis->motor_info.ctrl_mode = MOTOR_PID_VISION; // ���ÿ���ģʽΪ�Ӿ�PID
		if((vision.rx_info->yaw_error < 0.25f && vision.rx_info->yaw_error > -0.25f && vision.rx_info->target_status == 1))
		{
			axis_cnt++;
		}
		if ( axis_cnt>=10||(car.judge_info.launch_opening_status == CAR_LAUNCH_ON && (HAL_GetTick() - vision_start_tick) > 2000)) // ���Ŵ�ʱ�ӳ�ʱ�˳�
		{
			axis_cnt = 0;
			axis->motor_info.vision_move_init_pos = axis->motor_info.current_position;
			offset_reference_position = axis->motor_info.current_position;
			axis->task_info.move_task_step = AXIS_MOVE_TASK_SET_OFFSET; // ����������Ϊ���õ��offset
		}
//		if ((vision.rx_info->yaw_error < 0.25f && vision.rx_info->yaw_error > -0.25f && vision.rx_info->target_status == 1) ||
//			(car.judge_info.launch_opening_status == CAR_LAUNCH_ON && (HAL_GetTick() - vision_start_tick) > 2000)) // ���Ŵ�ʱ�ӳ�ʱ�˳�
//		{
//			axis->motor_info.vision_move_init_pos = axis->motor_info.current_position;
//			offset_reference_position = axis->motor_info.current_position;
//			axis->task_info.move_task_step = AXIS_MOVE_TASK_SET_OFFSET; // ����������Ϊ���õ��offset
//		}
    }
    else // û���Ӿ�
    {
		if (vision.status->rx_state == DEV_OFFLINE) 
		{
		  offset_reference_position = 0;
		}
		axis->task_info.move_task_step = AXIS_MOVE_TASK_SET_OFFSET; // ����������Ϊ���õ��offset
    }
    break;
  case AXIS_MOVE_TASK_SET_OFFSET:
    axis->motor_info.ctrl_mode = MOTOR_PID_POSITION; // ���ÿ���ģʽΪλ��PID
    axis->motor_info.target_position = offset_reference_position + offset;
    axis->task_info.move_task_step = AXIS_MOVE_TASK_WAIT_MOVE_TO_SET_OFFSET; // ����������Ϊ���
    break;
  case AXIS_MOVE_TASK_WAIT_MOVE_TO_SET_OFFSET:
    pos_err = abs(axis->motor_info.current_position - axis->motor_info.target_position);
    if (pos_err < 3000 && abs(axis->axis_motor->rx_info->speed) < 100)
    {
      axis->task_info.move_task_step = AXIS_MOVE_TASK_DONE;
    }
    break;
  case AXIS_MOVE_TASK_DONE:
    break;
  default:

    break;
  }
}

/**
 *@brief �������
 *
 */
void Axis_TaskManager(void *pvParameters)
{
  uint8_t is_vision_on = axis.task_info.axis_task_arg.is_vision_on;
  uint8_t is_delay = axis.task_info.axis_task_arg.is_axis_delay;
  uint8_t target_id = car.target_id;
  int32_t all_offset = axis.config.axis_all_offset; // ����ƫ��
  int32_t offset = 0;
  axis.motor_info.ctrl_mode = MOTOR_PID_POSITION;
  axis.state = AXIS_MOVING;
  switch (target_id)
  {
  case 1:
    offset = axis.config.axis_first_offset + all_offset;
    break;
  case 2:
    offset = axis.config.axis_second_offset + all_offset;
    break;
  case 3:
    offset = axis.config.axis_third_offset + all_offset;
    break;
  case 4:
    offset = axis.config.axis_fourth_offset + all_offset;
    break;
  default:
    axis.task_info.move_task_step = AXIS_MOVE_TASK_DONE;
    axis.state = AXIS_FREE;
    xAxisTaskHandle = NULL;
    vTaskDelete(NULL);
    break;
  }

  axis.task_info.move_task_step = AXIS_MOVE_TASK_START; // ����
  for (;;)
  {
    if (car.car_ctrl_mode < CAR_MT_MODE)
    {
      xAxisTaskHandle = NULL;
      vTaskDelete(NULL);
    }

    Axis_Offset_Move_Task(&axis, is_vision_on, offset, is_delay);
    if (axis.task_info.move_task_step == AXIS_MOVE_TASK_DONE)
    {
      axis.state = AXIS_FREE;
      xAxisTaskHandle = NULL;
      vTaskDelete(NULL);
    }
  }
}

HAL_StatusTypeDef Axis_Task_Start(void)
{

  // �������״̬
  if (xAxisTaskHandle != NULL)
  {
    // ������
    return HAL_BUSY;
  }
  // ������ɾ��
  if (xTaskCreate(Axis_TaskManager,
                  "AxisTask", 1028,
                  NULL, tskIDLE_PRIORITY + 2,
                  &xAxisTaskHandle) != pdPASS)
  {
    return HAL_ERROR;
  }
  else
  {
    axis.task_info.AxisTaskState = eRunning; // ����״̬����Ϊ������
    return HAL_OK;
  }
}

/**
 * @brief ���ܿ�
 *
 * @param axis
 */
void Axis_Work(axis_t *axis)
{
	Axis_TaskStatus_Update(axis);
  Axis_External_Info_Update(axis); // ��������λ��

  if (car.car_ctrl_mode == CAR_SLEEP_MODE)
  {
    Axis_Reset(axis); // ���û�п��أ���������
  }
  else
  {
    if (axis->state == AXIS_OFFLINE)
    {
      axis->state = AXIS_FREE;
    }
  }

  if (car.car_ctrl_mode != CAR_SLEEP_MODE) // ����
  {
    Axis_Pid_Calculating(axis);
    //		axis->axis_motor->base_info.motor_out = axis->motor_info.output_current;
  }
}
