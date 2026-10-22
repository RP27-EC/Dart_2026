/**
 *@file uart_screen_protocol.c
 *@brief 串口触摸屏通信
 *@version 1.0
 *@date 2025-11-18
 */
#include "car.h"
#include "stdbool.h"
#include "string.h"
#include "usart.h"
#include "crc.h"
#include "tuner.h"
#include "semphr.h"
#include "launcher_motor.h"
#include "launcher.h"
#include "axis.h"
#include "force.h"
#include <stdio.h>
#include <stdint.h>
#include "uartscreen_protocol.h"

static void tjc_temp_upodate(void);

#define POINT_COUNT 2 // 虚拟浮点数位数

/**
 * @brief  计算CRC-16/Modbus校验值,陶晶驰串口屏使用CRC-16/Modbus校验。
 * @param  pData: 数据指针
 * @param  Length: 数据长度
 * @retval CRC16校验值
 */
// uint16_t CRC16_Modbus(uint8_t *pData, uint16_t Length)
//{
//     uint16_t crc = 0xFFFF;  // 初始值
//     uint16_t i, j;

//    if (pData == NULL) {
//        return 0;
//    }

//    for (j = 0; j < Length; j++) {
//        crc ^= (uint16_t)pData[j];  // 与数据异或

//        for (i = 0; i < 8; i++) {
//            if (crc & 0x0001) {     // 如果最低位为1
//                crc = (crc >> 1) ^ 0xA001;  // 右移并与反转多项式异或
//            } else {
//                crc = crc >> 1;     // 否则直接右移
//            }
//        }
//    }
//
//    return crc;
//}

uint16_t CRC16_Modbus(uint8_t *pData, uint16_t Length)
{
	uint16_t crc = 0xFFFF; // 初始值
	uint16_t i, j;

	if (pData == NULL)
	{
		return 0;
	}

	for (j = 0; j < Length; j++)
	{
		crc ^= pData[j]; // 与数据字节异或

		for (i = 0; i < 8; i++)
		{
			if (crc & 0x0001)
			{							   // 如果最低位为1
				crc = (crc >> 1) ^ 0xA001; // 右移并与反转多项式异或
			}
			else
			{
				crc = crc >> 1; // 否则直接右移
			}
		}
	}

	return crc;
}

tjc_rx_all_info_t tjc_rx_info = {0};
tjc_tx_info_t tjc_tx_info = {0};

tjc_t tjc = {
	.tjc__all_rx_info = &tjc_rx_info,
	.tjc_tx_info = &tjc_tx_info,
	.CRC16 = 0,
};

/*------------------------接收函数-----------------------------------------*/

///**
// *@brief 接收数据函数
// */
// void Tjc_Rx_Data(uint8_t *rxBuf)
//{
//
//	uint16_t rx_CRC16=CRC16_Modbus(rxBuf,TJC_DATA_LENGTH - 2);      //接收到数据计算出来的crc
//
//	memcpy(tjc.tjc_packet_header, rxBuf, sizeof(tjc_packet_header_t));
//	if(!(tjc.tjc_packet_header->SOF==TJC_PACKET_HEAD && tjc.tjc_packet_header->pack_id==TJC_PACKET_ID && tjc.tjc_packet_header->protocol_id==TJC_PROTOCOL_ID))
//	{
//		return;
//	}
//	else
//	{
//		memcpy(tjc.tjc__all_rx_info,rxBuf+TJC_HEAD_LENGTH,sizeof(tjc_rx_all_info_t));
//
//		if(tjc.CRC16 == rx_CRC16)
//		{
////			uint16_t divitioon =1;       //根据小数位数缩小倍数
////			for(int i=0;i<POINT_COUNT;i++)
////			{
////				divitioon*=10;
////			}
////			tjc.tjc_finaldata->axis_all_offset = (float)tjc.tjc_rx_info->axis_all_offset/divitioon;
////			tjc.tjc_finaldata->axis_first_offset =(float)tjc.tjc_rx_info->axis_first_offset/divitioon;
////			tjc.tjc_finaldata->axis_second_offset =(float)tjc.tjc_rx_info->axis_second_offset/divitioon;
////			tjc.tjc_finaldata->axis_third_offset =(float)tjc.tjc_rx_info->axis_third_offset/divitioon;
////			tjc.tjc_finaldata->axis_fourth_offset =(float)tjc.tjc_rx_info->axis_fourth_offset/divitioon;
////			tjc.tjc_finaldata->force_all_offset =(float)tjc.tjc_rx_info->force_all_offset/divitioon;
////			tjc.tjc_finaldata->force_first_offset =(float)tjc.tjc_rx_info->force_first_offset/divitioon;
////			tjc.tjc_finaldata->force_second_offset =(float)tjc.tjc_rx_info->force_second_offset/divitioon;
////			tjc.tjc_finaldata->force_third_offset =(float)tjc.tjc_rx_info->force_third_offset/divitioon;
////			tjc.tjc_finaldata->force_fourth_offset =(float)tjc.tjc_rx_info->force_fourth_offset/divitioon;
////			if (car.car_ctrl_mode == CAR_SLEEP_MODE)  //关控条件下写入flash（写操作需要一定时间，建议在关控情况下进行）
////            {
//                Tuner_UpdateTask_Start(); // 启动更新任务
////            }
//		}
//		else
//		{
//			memset(tjc.tjc_packet_header,0,sizeof(tjc_packet_header_t));
//			memset(tjc.tjc__all_rx_info,0,sizeof(tjc_rx_all_info_t));
//			return;
//		}
//	}
//
//}

void process_received_data(uint8_t *buffer, tjc_rx_all_info_t *parsed_data)
{
	uint16_t CRC16;
	uint16_t recieve_CRC;
	// 1. 解析包头
	memcpy(&parsed_data->packet_header, buffer, sizeof(tjc_packet_header_t));
	// 2. 验证 SOF 是否正确
	if (parsed_data->packet_header.SOF != 0xA6)
	{
		// 数据包头 SOF 错误，不处理
		return;
	}

	// 3. 根据 pack_id 解析不同的数据包
	uint8_t pack_id = parsed_data->packet_header.pack_id;
	uint16_t offset = sizeof(tjc_packet_header_t);

	switch (parsed_data->packet_header.pack_id)
	{
	case 0x00:
		if (parsed_data->packet_header.length == sizeof(tjc_rx_all_info_t))
		{
			CRC16 = CRC16_Modbus((buffer), sizeof(tjc_rx_all_info_t));
			recieve_CRC = buffer[sizeof(tjc_rx_all_info_t)] | buffer[sizeof(tjc_rx_all_info_t) + 1] << 8;
			if (recieve_CRC != CRC16)
			{
				return;
			}
			else
			{
				memcpy(parsed_data, buffer, sizeof(tjc_rx_all_info_t));
			}
		}
		else
		{
			return;
		}
		break;
	case 0x01: // launcher 包
		if (parsed_data->packet_header.length == sizeof(tjc_rx_launcher_info_t))
		{
			CRC16 = CRC16_Modbus(buffer, (sizeof(tjc_rx_launcher_info_t) + offset));
			recieve_CRC = buffer[offset + sizeof(tjc_rx_launcher_info_t)] | buffer[offset + sizeof(tjc_rx_launcher_info_t) + 1] << 8;
			if (recieve_CRC != CRC16)
			{
				return;
			}
			else
			{
				memcpy(&parsed_data->launcher_info, buffer + offset, sizeof(tjc_rx_launcher_info_t));
			}
		}
		else
		{
			return;
		}
		// 这里可以直接对 parsed_data->launcher_info 进行处理
		break;

	case 0x02: // feeding 包
		if (parsed_data->packet_header.length == sizeof(tjc_rx_feeding_info_t))
		{
			CRC16 = CRC16_Modbus(buffer, (sizeof(tjc_rx_feeding_info_t) + offset));
			recieve_CRC = buffer[offset + sizeof(tjc_rx_feeding_info_t)] | buffer[offset + sizeof(tjc_rx_feeding_info_t) + 1] << 8;
			if (recieve_CRC != CRC16)
			{
				return;
			}
			else
			{
				memcpy(&parsed_data->feeding_info, buffer + offset, sizeof(tjc_rx_feeding_info_t));
			}
		}
		else
		{
			return;
		}

		// 这里可以直接对 parsed_data->feeding_info 进行处理
		break;

	case 0x03: // force_axis 包
		if (parsed_data->packet_header.length == sizeof(tjc_rx_force_axis_info_t))
		{
			CRC16 = CRC16_Modbus(buffer, (sizeof(tjc_rx_force_axis_info_t) + offset));
			recieve_CRC = buffer[offset + sizeof(tjc_rx_force_axis_info_t)] | buffer[offset + sizeof(tjc_rx_force_axis_info_t) + 1] << 8;
			if (recieve_CRC != CRC16)
			{
				return;
			}
			else
			{
				memcpy(&parsed_data->force_axis_info, buffer + offset, sizeof(tjc_rx_force_axis_info_t));
			}
		}
		else
		{
			return;
		}
		// 这里可以直接对 parsed_data->force_axis_info 进行处理

		break;

	case 0x04: // shoot 包
		if (parsed_data->packet_header.length == sizeof(tjc_rx_shoot_info_t))
		{
			CRC16 = CRC16_Modbus(buffer, (sizeof(tjc_rx_shoot_info_t) + offset));
			recieve_CRC = buffer[offset + sizeof(tjc_rx_shoot_info_t)] | buffer[offset + sizeof(tjc_rx_shoot_info_t) + 1] << 8;
			if (recieve_CRC != CRC16)
			{
				return;
			}
			else
			{
				memcpy(&parsed_data->shoot_info, buffer + offset, sizeof(tjc_rx_shoot_info_t));
			}
		}
		else
		{
			return;
		}
		// 这里可以直接对 parsed_data->shoot_info 进行处理
		break;
	default:
		// 如果包 ID 是未知的，直接返回
		return;
	}

	// 在此可以对解析后的数据进行进一步的处理
	if (car.car_ctrl_mode == CAR_SLEEP_MODE) // 关控条件下写入flash（写操作需要一定时间，建议在关控情况下进行）
	{
		Tuner_UpdateTask_Start(); // 启动更新任务
	}
	else
	{
#ifdef TEST_MODE
		tjc_temp_upodate(); // 临时调参函数。
#endif
	}
	// 比如将 `parsed_data` 存储到队列、发送到其他模块等
}

static void tjc_temp_upodate(void)
{
	switch (tjc.tjc__all_rx_info->packet_header.pack_id)
	{
	case 0:
		launcher.config.motor3508_up_position = tjc.tjc__all_rx_info->launcher_info.up_position;
		launcher.config.motor3508_down_position = tjc.tjc__all_rx_info->launcher_info.down_position;
		launcher.config.motor6020_standby_angle[0] = tjc.tjc__all_rx_info->launcher_info.standy2;
		launcher.config.motor6020_standby_angle[1] = tjc.tjc__all_rx_info->launcher_info.standy3;
		launcher.config.motor6020_standby_angle[2] = tjc.tjc__all_rx_info->launcher_info.standy4;
		launcher.config.motor6020_load_angle[0] = tjc.tjc__all_rx_info->launcher_info.load2;
		launcher.config.motor6020_load_angle[1] = tjc.tjc__all_rx_info->launcher_info.load3;
		launcher.config.motor6020_load_angle[2] = tjc.tjc__all_rx_info->launcher_info.load4;

		launcher.feedServo_info.second_servo1_target = tjc.tjc__all_rx_info->feeding_info.servo_state2;
		launcher.feedServo_info.third_servo1_target = tjc.tjc__all_rx_info->feeding_info.servo_state3;
		launcher.feedServo_info.fourth_servo1_target = tjc.tjc__all_rx_info->feeding_info.servo_state4;
		launcher.electromagnet_info.second_electromagnet_target = tjc.tjc__all_rx_info->feeding_info.electromagnet_state2;
		launcher.electromagnet_info.third_electromagnet_target = tjc.tjc__all_rx_info->feeding_info.electromagnet_state3;
		launcher.electromagnet_info.fourth_electromagnet_target = tjc.tjc__all_rx_info->feeding_info.electromagnet_state4;
		feeding_secondservo_config.sleep = tjc.tjc__all_rx_info->feeding_info.servo_pwm1;
		feeding_thirdservo_config.sleep = tjc.tjc__all_rx_info->feeding_info.servo_pwm2;
		feeding_fourthservo_config.sleep = tjc.tjc__all_rx_info->feeding_info.servo_pwm3;

		force.config.force_first_offset = tjc.tjc__all_rx_info->force_axis_info.force1_target;
		force.config.force_second_offset = tjc.tjc__all_rx_info->force_axis_info.force2_target;
		force.config.force_third_offset = tjc.tjc__all_rx_info->force_axis_info.force3_target;
		force.config.force_fourth_offset = tjc.tjc__all_rx_info->force_axis_info.force4_target;
		axis.config.axis_first_offset = tjc.tjc__all_rx_info->force_axis_info.axis1_target;
		axis.config.axis_second_offset = tjc.tjc__all_rx_info->force_axis_info.axis2_target;
		axis.config.axis_third_offset = tjc.tjc__all_rx_info->force_axis_info.axis3_target;
		axis.config.axis_fourth_offset = tjc.tjc__all_rx_info->force_axis_info.axis4_target;

		car.task_info.first_widows_total_shoots = tjc.tjc__all_rx_info->shoot_info.frist_window_shoot;
		car.target_id = tjc.tjc__all_rx_info->shoot_info.target_id;
		car.judge_info.launch_opening_status = tjc.tjc__all_rx_info->shoot_info.launch_open_flag;
		//		 = tjc.tjc__all_rx_info->shoot_info.rebuild_flag;            //待开发
		break;
	case 1:
		launcher.config.motor3508_up_position = tjc.tjc__all_rx_info->launcher_info.up_position;
		launcher.config.motor3508_down_position = tjc.tjc__all_rx_info->launcher_info.down_position;
		launcher.config.motor6020_standby_angle[0] = tjc.tjc__all_rx_info->launcher_info.standy2;
		launcher.config.motor6020_standby_angle[1] = tjc.tjc__all_rx_info->launcher_info.standy3;
		launcher.config.motor6020_standby_angle[2] = tjc.tjc__all_rx_info->launcher_info.standy4;
		launcher.config.motor6020_load_angle[0] = tjc.tjc__all_rx_info->launcher_info.load2;
		launcher.config.motor6020_load_angle[1] = tjc.tjc__all_rx_info->launcher_info.load3;
		launcher.config.motor6020_load_angle[2] = tjc.tjc__all_rx_info->launcher_info.load4;

		break;
	case 2:
		launcher.feedServo_info.second_servo1_target = tjc.tjc__all_rx_info->feeding_info.servo_state2;
		launcher.feedServo_info.third_servo1_target = tjc.tjc__all_rx_info->feeding_info.servo_state3;
		launcher.feedServo_info.fourth_servo1_target = tjc.tjc__all_rx_info->feeding_info.servo_state4;
		launcher.electromagnet_info.second_electromagnet_target = tjc.tjc__all_rx_info->feeding_info.electromagnet_state2;
		launcher.electromagnet_info.third_electromagnet_target = tjc.tjc__all_rx_info->feeding_info.electromagnet_state3;
		launcher.electromagnet_info.fourth_electromagnet_target = tjc.tjc__all_rx_info->feeding_info.electromagnet_state4;
		feeding_secondservo_config.sleep = tjc.tjc__all_rx_info->feeding_info.servo_pwm1;
		feeding_thirdservo_config.sleep = tjc.tjc__all_rx_info->feeding_info.servo_pwm2;
		feeding_fourthservo_config.sleep = tjc.tjc__all_rx_info->feeding_info.servo_pwm3;
		break;
	case 3:
		launcher.feedServo_info.second_servo1_target = tjc.tjc__all_rx_info->feeding_info.servo_state2;
		launcher.feedServo_info.third_servo1_target = tjc.tjc__all_rx_info->feeding_info.servo_state3;
		launcher.feedServo_info.fourth_servo1_target = tjc.tjc__all_rx_info->feeding_info.servo_state4;
		launcher.electromagnet_info.second_electromagnet_target = tjc.tjc__all_rx_info->feeding_info.electromagnet_state2;
		launcher.electromagnet_info.third_electromagnet_target = tjc.tjc__all_rx_info->feeding_info.electromagnet_state3;
		launcher.electromagnet_info.fourth_electromagnet_target = tjc.tjc__all_rx_info->feeding_info.electromagnet_state4;
		feeding_secondservo_config.sleep = tjc.tjc__all_rx_info->feeding_info.servo_pwm1;
		feeding_thirdservo_config.sleep = tjc.tjc__all_rx_info->feeding_info.servo_pwm2;
		feeding_fourthservo_config.sleep = tjc.tjc__all_rx_info->feeding_info.servo_pwm3;
		break;
	case 4:
		car.task_info.first_widows_total_shoots = tjc.tjc__all_rx_info->shoot_info.frist_window_shoot;
		car.target_id = tjc.tjc__all_rx_info->shoot_info.target_id;
		car.judge_info.launch_opening_status = (uint8_t)tjc.tjc__all_rx_info->shoot_info.launch_open_flag;

		break;
	}
}

/*-----------------------发送函数---------------------------------*/

/**--为了防止发送数据进发送完成中断，定义了一个HAL_UART_Transmit_DMA_TJC函数，写在stm32_f4xx_hal_uart.c/.h文件里面，--**/

static char cmd[200]; // 用于存储发送命令的缓冲区

// TJC指令结束符：3个0xFF
#define TJC_END_FF "\xFF\xFF\xFF"

// 设置文本
void TJC_SetText(const char *obj_name, const char *text)
{
	//	char cmd[64];
	sprintf(cmd, "%s.txt=\"%s\"\xff\xff\xff", obj_name, text);
	HAL_UART_Transmit_DMA_TJC(&huart1, (uint8_t *)cmd, strlen(cmd));
}

// 设置数值
void TJC_SetValue(const char *obj_name, int value)
{
	//	char cmd[32];
	sprintf(cmd, "%s.val=%d\xff\xff\xff", obj_name, value);
	HAL_UART_Transmit_DMA_TJC(&huart1, (uint8_t *)cmd, strlen(cmd));
}

// 设置颜色
void TJC_SetColor(const char *obj_name, uint16_t color)
{
	//	char cmd[32];
	sprintf(cmd, "%s.pco=%d\xff\xff\xff", obj_name, color);
	HAL_UART_Transmit_DMA_TJC(&huart1, (uint8_t *)cmd, strlen(cmd));
}

// 切换页面
void TJC_ChangePage(uint8_t page_id)
{
	//	char cmd[16];
	sprintf(cmd, "page %d\xff\xff\xff", page_id);
	HAL_UART_Transmit_DMA_TJC(&huart1, (uint8_t *)cmd, strlen(cmd));
}

// 使能/禁用控件
void TJC_SetVisible(const char *obj_name, uint8_t enable)
{
	//	char cmd[32];
	sprintf(cmd, "vis %s,%d\xff\xff\xff", obj_name, enable);
	HAL_UART_Transmit_DMA_TJC(&huart1, (uint8_t *)cmd, strlen(cmd));
}

void TJC_Get_Data(void)
{
	TJC_SetValue("mea_3508_pos", launcher.motors.motor3508R->rx_info->encoder_sum);
	TJC_SetValue("mea_6020_pos", launcher.motors.motor6020->rx_info->encoder_sum);
	TJC_SetValue("mea_force", force.force_motor->rx_info->encoder_sum);
	TJC_SetValue("mea_axis", axis.axis_motor->rx_info->encoder_sum);
}
