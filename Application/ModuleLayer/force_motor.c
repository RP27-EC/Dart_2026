#include "force_motor.h"
#include "can.h"
#include "rp_math.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;


Motor_RM_Born_Info_t force_RM_3508L_Born_info={
	.rxId = 2,
	.stdId = 0x200,
	.type = _3508_Reduction,
	.hcan = &hcan1,
};

Motor_RM_Rx_Info_t force_RM_3508_rx_info;
Motor_RM_Tx_Info_t force_RM_3508_tx_info;
Motor_RM_State_t force_RM_3508_state={
	.offline_cnt = 0,
	.offline_cnt_max = 50,
	.status = DEV_OFFLINE,
};
pid_ctrl_t Force_3508_Ctrl_Speed={
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
pid_ctrl_t Force_3508_Ctrl_Position_inner={
	.target = 0,
	.measure = 0,
	.err = 0,
	.last_err = 0,
	.kp = 10,
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
pid_ctrl_t Force_3508_Ctrl_Position_outer={
	.target = 0,
	.measure = 0,
	.err = 0,
	.last_err = 0,
	.kp = 0.3,
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

Motor_RM_Ctrl_Info_t force_RM_3508_pid={
	.speed_ctrl = &Force_3508_Ctrl_Speed,
	.angle_ctrl_inner = &Force_3508_Ctrl_Position_inner,
	.angle_ctrl_outer = &Force_3508_Ctrl_Position_outer,
};

Motor_RM_t force_motor[FORCE_MOTOR_LIST]=
{
	[FORCE_MOTOR]={
		.born_info = &force_RM_3508L_Born_info,
		.rx_info = &force_RM_3508_rx_info,
		.tx_info = &force_RM_3508_tx_info,
		.state = &force_RM_3508_state,
		.ctrl = &force_RM_3508_pid,
		.single_init =RM_Motor_Init,		
	},
};

void force_motor_init(void)
{
	for(uint8_t i=0;i<FORCE_MOTOR_LIST;i++)
	{
		force_motor[i].single_init(&force_motor[i]);
	}	
}
	
void force_motor_heart_beat(void)
{
	for(uint8_t i=0;i<FORCE_MOTOR_LIST;i++)
	{
		force_motor[i].single_heart_beat(&force_motor[i]);
	}
}

bool force_motor_stick_check(Motor_RM_t *motor)
{
	static uint8_t stick_cnt = 0;
	if (abs(motor->tx_info->torque_current) >= 2000 && motor->rx_info->encoder_speed <= 10)
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
