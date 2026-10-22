#include "can_protocol.h"
#include "drv_can.h"
#include "launcher_motor.h"
#include "axis_motor.h"
#include "force_motor.h"
#include "launcher.h"
#include "axis.h"
#include "force.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

/**
 * @brief CAN1 发送数据
 *
 */
uint8_t CAN1_txBuf[8];
uint8_t CAN2_txBuf[8];
void CAN1_Send(void)
{
	
	CAN1_txBuf[0] = (launcher.motor3508_info.motor3508L_output_current>>8)&0xFF;
	CAN1_txBuf[1] = launcher.motor3508_info.motor3508L_output_current&0xFF;
	CAN1_txBuf[2] = (launcher.motor3508_info.motor3508R_output_current>>8)&0xFF;
	CAN1_txBuf[3] = launcher.motor3508_info.motor3508R_output_current&0xFF;
	CAN1_txBuf[4] = (force.motor_info.output_current>>8)&0xFF;
	CAN1_txBuf[5] = force.motor_info.output_current&0xFF;
	CAN1_txBuf[6] = (axis.motor_info.output_current>>8)&0xFF;
	CAN1_txBuf[7] = axis.motor_info.output_current&0xFF;	
	CAN_SendData(&hcan1,0x200,CAN1_txBuf);
	
}

void CAN2_Send(void)
{
	CAN2_txBuf[0] = (launcher.motor6020_info.output_current>>8)&0xFF;
	CAN2_txBuf[1] = launcher.motor6020_info.output_current&0xFF;
	CAN2_txBuf[2] = 0;
	CAN2_txBuf[3] = 0;
	CAN2_txBuf[4] = 0;
	CAN2_txBuf[5] = 0;
	CAN2_txBuf[6] = 0;
	CAN2_txBuf[7] = 0;
	CAN_SendData(&hcan2,0x1FF,CAN2_txBuf);
	
}


/**
 *  @brief  CAN1 接收数据
 */
void CAN1_rxDataHandler(uint32_t rxId, uint8_t *rxBuf)
{
	switch (rxId)
	{
		case ID_LAUNCHER_RM_3508L:
		{
			launcher_motor[LAUNCHER_MOTOR_RM3508L].rx(&launcher_motor[LAUNCHER_MOTOR_RM3508L], rxBuf);
			break;
		}
		case ID_AXIS_RM_2006:
		{
			axis_motor[AXIS_MOTOR].rx(&axis_motor[AXIS_MOTOR], rxBuf);
			break;
		}
		case ID_FORCE_RM_3508:
		{
			force_motor[FORCE_MOTOR].rx(&force_motor[FORCE_MOTOR],rxBuf);
			break;
		}
		case ID_LAUNCHER_RM_3508R:
			launcher_motor[LAUNCHER_MOTOR_RM3508R].rx(&launcher_motor[LAUNCHER_MOTOR_RM3508R], rxBuf);
		break;
		default:
			break;
	}
}
/**
 *  @brief  CAN2 接收数据
 */
void CAN2_rxDataHandler(uint32_t canId, uint8_t *rxBuf)
{
	
	switch (canId)
	{

		case ID_LOAD_RM_6020:
			launcher_motor[FEEDING_MOTOR_RM6020].rx(&launcher_motor[FEEDING_MOTOR_RM6020], rxBuf);
			break;
		default:
			break;
	}
}
