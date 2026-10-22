/**
 * @file vision_protocol.c
 * @author Isaac
 * @brief �Ӿ�ͨ��Э�飬������ܷ��ͺ�������
 * @version 0.1
 * @date 2023-11-21
 *
 * @copyright Copyright (c) 2023
 *
 */
#include "car.h"
#include "vision_protocol.h"
#include "stdbool.h"
#include "string.h"
#include "crc.h"
#include "usbd_cdc_if.h"
#include "car.h"
#include "launcher.h"
#include "axis.h"
#include "force.h"

Vision_Tx_Info_t vision_tx_info =
	{
		.SOF = 0xA5, // ֡���ֽ�
		.mode = 1,
};

Vison_Rx_Info_t vision_rx_info;
Vision_Status_t vision_status =
	{
		.offline_cnt_max = VISION_OFFLINE_CNT_MAX,
		.rx_state = DEV_OFFLINE,
		.tx_state = DEV_OFFLINE,
};
Vision_t vision =
	{
		.tx_info = &vision_tx_info,
		.rx_info = &vision_rx_info,
		.status = &vision_status,
};

uint8_t vision_txBuf[80];

uint8_t vision_RxBuf[80];
uint8_t size_test;
/**
 * @brief �Ӿ�ͨ����
 *
 * @param rxBuf
 * @return �յ����ݣ�DEV_OK û�е����ݣ�DEV_NO
 * @note
 */
void Vision_DataRx(uint8_t *rxBuf)
{
	size_test = sizeof(Vison_Rx_Info_t);
	memcpy(vision_RxBuf, rxBuf, sizeof(Vison_Rx_Info_t));
	/* ֡���ֽ��Ƿ�Ϊ0xA5 */
	if (rxBuf[0] == 0xA5)
	{
		/* ֡ͷCRC8У��*/
		if (Verify_CRC8_Check_Sum(rxBuf, 3) == true)
		{
			/* ֡βCRC16У�� */
			if (Verify_CRC16_Check_Sum(rxBuf, sizeof(Vison_Rx_Info_t)) == true)
			{
				memcpy(&vision_rx_info, vision_RxBuf, sizeof(Vison_Rx_Info_t));
				vision.status->offline_cnt = 0;
			}
		}
	}
}

/**
 * @name    Vision_DataTx
 * @brief   �Ӿ�ͨ�ŷ�(����3)
 * @param   &huart3
 * @retval  void
 */
void Vision_DataTx(void)
{
#if ENABLE_VISION_RECORD
	if (car.judge_info.game_progress >= GAME_IN_CHECKING && car.judge_info.judge_state == DEV_ONLINE)
	{
		// vision_tx_info.dune = judge.dart_client_cmd.dart_launch_opening_status;
		vision_tx_info.dune = 0;
	}
#else
	vision_tx_info.dune = 1; // ����¼
#endif

	vision_tx_info.car_ctrl_mode = car.car_ctrl_mode; // ���ó��Ŀ���ģʽ
	vision_tx_info.load_task_step = launcher.task_info.launcher_load_state;
	vision_tx_info.shoot_task_step = launcher.task_info.launcher_shoot_state;
	vision_tx_info.LauncherTaskState = launcher.task_info.LauncherTaskState;

	/**------------��������--------------**/
	vision_tx_info.rc_state = car.module_state.rc_state;
	vision_tx_info.dart_id = car.target_id;

	vision_tx_info.launcher_current_position_L = launcher.motor3508_info.motor3508L_position;
	vision_tx_info.launcher_current_position_R = launcher.motor3508_info.motor3508R_position;
	vision_tx_info.launcher_target_position = launcher.motor3508_info.motor3508L_target_position;
	vision_tx_info.launcher_motor_state = car.module_state.motor3508L_state * 10 + car.module_state.motor3508R_state;
	vision_tx_info.force_target_position = force.motor_info.target_position;
	vision_tx_info.force_motor_state = car.module_state.motorForce_state;
	vision_tx_info.axis_ctrl_mode = axis.motor_info.ctrl_mode;
	vision_tx_info.axis_target_position = axis.motor_info.target_position;
	vision_tx_info.axis_motor_state = car.module_state.motorAxis_state;
	vision_tx_info.feeding_motor_state = car.module_state.motor6020_state;
	vision_tx_info.feeding_motor_position_err = (int32_t)launcher.motors.motor6020->ctrl->angle_ctrl_outer->err;
	vision_tx_info.load_flag = launcher.task_info.launcher_task_flag.Load_Flag;
	vision_tx_info.shoot_flag = launcher.task_info.launcher_task_flag.Shoot_Flag;

	/**----------�������ݽ���---------------**/
	vision_tx_info.axis_move_task_step = axis.task_info.move_task_step;
	vision_tx_info.AxisTaskState = axis.task_info.AxisTaskState;
	vision_tx_info.axis_current_position = axis.motor_info.current_position;
	vision_tx_info.force_move_task_step = force.task_info.move_task_step;
	vision_tx_info.ForceTaskState = force.task_info.ForceTaskState;
	vision_tx_info.force_current_position = force.motor_info.current_position;
	vision_tx_info.at_step = car.task_info.at_step;
	vision_tx_info.game_progress = car.judge_info.game_progress;
	vision_tx_info.launch_opening_status = car.judge_info.launch_opening_status;
	vision_tx_info.dart_target_type = car.judge_info.dart_target_type;
	vision_tx_info.dart_remaining_time = car.judge_info.dart_remaining_time;

	memcpy(vision_txBuf, &vision_tx_info, sizeof(Vision_Tx_Info_t)); // ���÷�����Ϣ
	Append_CRC8_Check_Sum(vision_txBuf, 3);							 // ����CRC8У����
	Append_CRC16_Check_Sum(vision_txBuf, sizeof(Vision_Tx_Info_t));	 // ����CRC16У����

	if (CDC_Transmit_FS(vision_txBuf, sizeof(Vision_Tx_Info_t)) == USBD_OK) // ���ڷ���
	{
		vision.status->tx_state = DEV_OK;
	}
	else
	{
		vision.status->tx_state = DEV_NO;
	}
}

/**
 * @name    Vision_HeartBeat
 * @brief   �Ӿ�ͨ������
 * @note    �ɼ���������
 */
void Vision_HeartBeat(void)
{
	vision.status->offline_cnt++;
	if (vision.status->offline_cnt >
		vision.status->offline_cnt_max)
	{
		vision.status->offline_cnt =
			vision.status->offline_cnt_max;

		vision.status->rx_state = DEV_OFFLINE;
	}
	else if (vision.status->rx_state == DEV_OFFLINE)
	{
		vision.status->rx_state = DEV_ONLINE;
	}
	if (vision.status->rx_state == DEV_ONLINE)
	{
		HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
	}
}
