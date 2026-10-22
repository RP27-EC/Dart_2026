/**
 * @file  device.c
 */
 
/* Includes ------------------------------------------------------------------*/
#include "device.h"
#include "launcher.h"
#include "axis.h"
#include "force.h"
#include "car.h"


/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
device_t device [DEVICE_CNT] =
{	
	[DEVICE_01] = {
	 .motor = &force_motor[FORCE_MOTOR],	
	},
	[DEVICE_02] = {
	 .motor = &axis_motor[AXIS_MOTOR],	
	},
	[DEVICE_03] = {
	 .motor = &launcher_motor[LAUNCHER_MOTOR_RM3508L],	
	},
	[DEVICE_04] = {
	 .motor = &launcher_motor[LAUNCHER_MOTOR_RM3508R],	
	},
	[DEVICE_05]= {
	 .motor = &launcher_motor[FEEDING_MOTOR_RM6020],	
	},
};
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/


/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void DEVICE_Init(void)
{
	axis_motor_init();
	force_motor_init();
	launcher_motor_init();
	// imu_sensor.init(&imu_sensor);
	
}

/**
  * @Name    Device_HeartBeat
  * @brief   设备心跳
  * @param   
  * @retval 
  * @author  
  * @Date    
**/
void Device_HeartBeat(void)
{
	axis_motor_heart_beat();
	force_motor_heart_beat();
	launcher_motor_heart_beat();
	if(axis.axis_motor->state->status == DEV_ONLINE && force.force_motor->state->status == DEV_ONLINE && launcher.motors.motor3508L->state->status == DEV_ONLINE  &&\
		launcher.motors.motor3508R->state->status == DEV_ONLINE && launcher.motors.motor6020->state->status == DEV_ONLINE)
	{
		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
	}
}

/**
  * @Name    Device_Work
  * @brief   设备总控，任务中调用
  * @param   
  * @retval 
  * @author  
  * @Date    
**/
void Device_Work(void)
{

	launcher.work(&launcher);
	axis.work(&axis);
	force.work(&force);

#if RC_OFF_PROTECTION
	
	if(!RC_ONLINE || car.car_ctrl_mode == CAR_SLEEP_MODE)
	{
		for(uint16_t i=0;i<DEVICE_CNT;i++)
		{
			device[i].motor->tx_info->torque_current = 0;
			device[i].motor->ctrl->speed_ctrl->out = 0;
			device[i].motor->ctrl->angle_ctrl_inner->out = 0;
			device[i].motor->ctrl->angle_ctrl_outer->out = 0;
		}
		launcher.motor3508_info.motor3508L_output_current = 0;
		launcher.motor3508_info.motor3508R_output_current = 0;
		force.motor_info.output_current = 0;
		axis.motor_info.output_current = 0;
		launcher.motor6020_info.output_current = 0;
	}


#endif
	
}

