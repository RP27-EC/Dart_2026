#include "axis_motor.h"
#include "can.h"
#include "rp_math.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;


Motor_RM_Born_Info_t axis_RM_2006_Born_info={
	.rxId = 3,
	.stdId = 0x200,
	.type = _2006_Single,
	.hcan = &hcan1,
};

Motor_RM_Rx_Info_t axis_RM_2006_rx_info;
Motor_RM_Tx_Info_t axis_RM_2006_tx_info;
Motor_RM_State_t axis_RM_2006_state={
	.offline_cnt = 0,
	.offline_cnt_max = 50,
	.status = DEV_OFFLINE,
};

pid_ctrl_t Axis_2006_Ctrl_Vision={
	.target = 0,
	.measure = 0,
	.err = 0,
	.last_err = 0,
	.kp = 20,
	.ki =  0.2,
	.kd = 0,
	.pout = 0,
	.iout = 0,
	.dout = 0,
	.out = 0,
	.last_dout = 0,
	.integral = 0,
	.integral_max = 10000,
	.out_max = 16000,
};

pid_ctrl_t Axis_2006_Ctrl_Speed={
	.target = 0,
	.measure = 0,
	.err = 0,
	.last_err = 0,
	.kp = 1,
	.ki =  0.1,
	.kd = 0,
	.pout = 0,
	.iout = 0,
	.dout = 0,
	.out = 0,
	.last_dout = 0,
	.integral = 0,
	.integral_max = 10000,
	.out_max = 16000,
};
pid_ctrl_t Axis_2006_Ctrl_Position_inner={
	.target = 0,
	.measure = 0,
	.err = 0,
	.last_err = 0,
	.kp = 13,
	.ki =  0.2,
	.kd = 0,
	.pout = 0,
	.iout = 0,
	.dout = 0,
	.out = 0,
	.last_dout = 0,
	.integral = 0,
	.integral_max = 10000,
	.out_max = 16000,
};
pid_ctrl_t Axis_2006_Ctrl_Position_outer={
	.target = 0,
	.measure = 0,
	.err = 0,
	.last_err = 0,
	.kp = 0.4,
	.ki =  0,
	.kd = 0,
	.pout = 0,
	.iout = 0,
	.dout = 0,
	.out = 0,
	.last_dout = 0,
	.integral = 0,
	.integral_max = 0,
	.out_max = 8000,
};

Motor_RM_Ctrl_Info_t axis_RM_2006_pid={
	.speed_ctrl = &Axis_2006_Ctrl_Speed,
	.angle_ctrl_inner = &Axis_2006_Ctrl_Position_inner,
	.angle_ctrl_outer = &Axis_2006_Ctrl_Position_outer,
	.vision_ctrl = &Axis_2006_Ctrl_Vision,
};

Motor_RM_t axis_motor[AXIS_MOTOR_LIST]=
{
	[AXIS_MOTOR]={
		.born_info = &axis_RM_2006_Born_info,
		.rx_info = &axis_RM_2006_rx_info,
		.tx_info = &axis_RM_2006_tx_info,
		.state = &axis_RM_2006_state,
		.ctrl = &axis_RM_2006_pid,
		.single_init =RM_Motor_Init,
	},
};

void axis_motor_init(void)
{
	for(uint8_t i=0;i<AXIS_MOTOR_LIST;i++)
	{
		axis_motor[i].single_init(&axis_motor[i]);
	}	
}
	
void axis_motor_heart_beat(void)
{
	for(uint8_t i=0;i<AXIS_MOTOR_LIST;i++)
	{
		axis_motor[i].single_heart_beat(&axis_motor[i]);
	}
}
 
bool axis_motor_stick_check(Motor_RM_t *motor)
{
	static uint8_t stick_cnt = 0;
	if (abs(motor->tx_info->torque_current) >= 1000 && motor->rx_info->encoder_speed <= 50)
	{
		stick_cnt++;
		if (stick_cnt >= 100)
		{
			stick_cnt = 0;

			return true;
		}
		else
		{
			return false;
		}
	}
	else
	{
		stick_cnt = 0;
		return false;
	}
}
