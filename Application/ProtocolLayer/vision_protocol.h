/**
 * @file vision_protocol.h
 * @author Isaac
 * @brief 视觉通信协议
 * @version 0.1
 * @date 2023-11-21
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef __VISION_PROTOCOL_H
#define __VISION_PROTOCOL_H

#include "rp_config.h"
#include "main.h"
#include "stdbool.h"

#define ENABLE_VISION_RECORD 1

#define VISION_OFFLINE_CNT_MAX (2000) // 离线最大计数(ms)

/* 与lzs的视觉协定
	lzs吃屎
*/

/**
 * @brief 视觉通信 接受信息结构体
 *
 */
typedef struct
{
	uint8_t SOF;
	uint8_t CRC8;

	float yaw_error;
	uint8_t target_status; // 0:lost 1:found

	uint16_t CRC16;

} __attribute__((packed)) Vison_Rx_Info_t;

/**
 * @brief 视觉通信 发送信息结构体
 *
 */
typedef struct
{
	uint8_t SOF;
	uint8_t mode; // 模式
	uint8_t CRC8;

	uint8_t status;
	uint8_t number; // 第几号飞镖
	uint8_t dune;
	uint8_t yaw_error;

	uint8_t car_ctrl_mode;

	uint8_t load_task_step;
	uint8_t shoot_task_step;
	uint8_t LauncherTaskState;

	/**--------新增数据----------**/

	uint8_t rc_state;	//遥控器状态
	uint8_t dart_id;	//第几号飞镖
	
	int32_t launcher_current_position_L;
	int32_t launcher_current_position_R;
	int32_t launcher_target_position;
	uint8_t launcher_motor_state;

	int32_t force_target_position;
	uint8_t force_motor_state;
	uint8_t axis_ctrl_mode;
	int32_t axis_target_position;
	uint8_t axis_motor_state;

	uint8_t feeding_motor_state;		// 供弹电机状态
	int32_t feeding_motor_position_err; // 供弹电机位置误差

	uint8_t load_flag;	// 装弹标志
	uint8_t shoot_flag; // 发射标志

	/**------新增数据结束--------**/

	uint8_t axis_move_task_step;
	uint8_t AxisTaskState;
	int32_t axis_current_position;

	uint8_t force_move_task_step;
	uint8_t ForceTaskState;
	int32_t force_current_position;

	uint8_t at_step;

	uint8_t game_progress;
	uint8_t launch_opening_status; // 飞镖发射站状态
	uint8_t dart_target_type;
	uint8_t dart_remaining_time; // 己方飞镖发射剩余时间，单位：秒

	uint16_t CRC16;
} __attribute__((packed)) Vision_Tx_Info_t;

/**
 * @brief 视觉通信 状态结构体
 *
 */
typedef struct Vision_Status
{
	dev_work_state_t tx_state; // 发送状态
	dev_work_state_t rx_state; // 接受状态
	uint16_t offline_cnt;	   // 接受离线计数
	uint16_t offline_cnt_max;  // 接受离线最大计数
} Vision_Status_t;

/**
 * @brief 视觉通信 总结构体
 *
 */
typedef struct Vision
{
	/* data */
	Vision_Tx_Info_t *tx_info;
	Vison_Rx_Info_t *rx_info;
	Vision_Status_t *status;
} Vision_t;

extern Vision_t vision;

void Vision_DataTx(void);
void Vision_DataRx(uint8_t *rxBuf);
void Vision_HeartBeat(void);
#endif
