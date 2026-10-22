/**
 *@file uart_screen_protocol.h
 *@brief 陶晶驰串口屏通信协议
 *@Version 1.0
 *@date 2025-11-20
 */#ifndef _UARTSCREEN_PROTOCOL_H
#define _UARTSCREEN_PROTOCOL_H
#include "rp_config.h"

// #define TJC_PACKET_HEAD  0xA5
//#define TJC_PACKET_ID    0x00
//#define TJC_PROTOCOL_ID  0x00E0
#define TJC_DATA_LENGTH  (sizeof(tjc_rx_all_info_t)
#define TJC_HEAD_LENGTH  4

//#define TEST_MODE           //临时调参开关
 
 /**
  *@biref 陶晶驰数据包包头
  */
 typedef struct 
 {
	 uint8_t SOF;       //A6
	 uint8_t pack_id;   //00-04
	 uint8_t length;     //数据包长度
	 uint8_t END;        //6A
 }__attribute__((packed)) tjc_packet_header_t;
 
 /**
  *@brief launcher包
  *@id    01         
  *@length 32
  */
typedef struct
{
	int32_t up_position;
	int32_t down_position;	 
	int32_t standy2;
	int32_t standy3;
	int32_t standy4;
	int32_t load2; 
	int32_t load3; 
	int32_t load4;	
}__attribute__((packed)) tjc_rx_launcher_info_t;

 /**
  *@brief feeding包
  *@id    02
  *@length 28+8
  */
typedef struct
{
	int32_t servo_state2; 
	int32_t servo_state3;	
	int32_t servo_state4;
	int32_t electromagnet_state2;
	int32_t electromagnet_state3;
	int32_t electromagnet_state4;
	int32_t servo_pwm1;
	int32_t servo_pwm2;
	int32_t servo_pwm3;
}__attribute__((packed)) tjc_rx_feeding_info_t;

 /**
  *@brief force_axis包
  *@id    03
  *@length 32+8
  */
typedef struct
{

	int32_t force1_target; 
	int32_t force2_target;	
	int32_t force3_target;
	int32_t force4_target;
	int32_t axis1_target;
	int32_t axis2_target;
	int32_t axis3_target;
	int32_t axis4_target;
	int32_t force_all_offest;
	int32_t axis_all_offest;
}__attribute__((packed)) tjc_rx_force_axis_info_t;

 /**
  *@brief force_axis包
  *@id    04
  *@length 16
  */
typedef struct
{
	int32_t frist_window_shoot; 
	int32_t target_id;	
	int32_t launch_open_flag;
	int32_t rebuild_flag;
}__attribute__((packed)) tjc_rx_shoot_info_t;

 /**
  *@biref 接收数据包
  *@id    00
  *@length 112+16
  *
  */
 
typedef struct
{
	tjc_packet_header_t packet_header;
	tjc_rx_launcher_info_t launcher_info;
	tjc_rx_feeding_info_t feeding_info;
	tjc_rx_force_axis_info_t force_axis_info;
	tjc_rx_shoot_info_t shoot_info;
	 
}__attribute__((packed)) tjc_rx_all_info_t;

/**
 *@brief 发送数据包
 */
 
typedef struct
{
	int32_t force_offest_measure;
	int32_t axis_offest_measure;
	
}__attribute__((packed)) tjc_tx_info_t;
 
typedef struct
{
	tjc_rx_all_info_t* tjc__all_rx_info;
	tjc_tx_info_t* tjc_tx_info;
	int16_t CRC16;
}tjc_t;

extern tjc_t tjc;
void Tjc_Rx_Data(uint8_t *rxBuf);
void Tic_Tx_Data(void);
void process_received_data(uint8_t* buffer,tjc_rx_all_info_t* parsed_data); 


void TJC_SetText(const char* obj_name, const char* text);
void TJC_SetValue(const char* obj_name, int value);
void TJC_SetColor(const char* obj_name, uint16_t color);
void TJC_ChangePage(uint8_t page_id);
void TJC_SetVisible(const char* obj_name, uint8_t enable);
void TJC_Get_Data(void);
#endif
