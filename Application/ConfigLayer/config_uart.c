
#include "config_uart.h"
#include "rc_sensor.h"
#include "judge_protocol.h"
#include "uartscreen_protocol.h"

/**
  * @Name    USART1_rxDataHandler
  * @brief   数据更新
**/
void USART1_rxDataHandler(uint8_t *rxBuf)
{
	process_received_data(rxBuf,tjc.tjc__all_rx_info); 
}

/**
  * @Name    USART3_rxDataHandler
  * @brief   遥控器更新
**/
void USART3_rxDataHandler(uint8_t *rxBuf)
{
	// 更新遥控数据
	rc_sensor.update(&rc_sensor, rxBuf);//解析协议
	rc_sensor.check(&rc_sensor);
}

/**
  * @Name    USART6_rxDataHandler
  * @brief    
**/

void USART6_rxDataHandler(uint8_t *rxBuf)
{
	Judge_Update(&judge, rxBuf);
}
