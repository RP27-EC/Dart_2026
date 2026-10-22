/**
 * @file launcher_motor.c
 * @author
 * @brief
 * @version 0.1
 * @date 2025-12-22
 *
 *
 */

#include "launcher_motor.h"
#include "rp_math.h"
#include "tim.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

Motor_RM_Born_Info_t launcher_RM_3508L_Born_info = {
	.rxId = 0,
	.stdId = 0x200,
	.type = _3508_Reduction_27,
	.hcan = &hcan1,
};
Motor_RM_Born_Info_t launcher_RM_3508R_Born_info = {
	.rxId = 1,
	.stdId = 0x200,
	.type = _3508_Reduction_27,
	.hcan = &hcan1,
};
Motor_RM_Born_Info_t feeding_RM_6020_Born_info = {
	.rxId = 0,
	.stdId = 0x1FF,
	.type = _6020_Single,
	.hcan = &hcan2,
};

Motor_RM_Rx_Info_t feeding_RM_6020_rx_info;
Motor_RM_Rx_Info_t launcher_RM_3508L_rx_info;
Motor_RM_Rx_Info_t launcher_RM_3508R_rx_info;

Motor_RM_Tx_Info_t feeding_RM_6020_tx_info;
Motor_RM_Tx_Info_t launcher_RM_3508L_tx_info;
Motor_RM_Tx_Info_t launcher_RM_3508R_tx_info;

Motor_RM_State_t feeding_RM_6020_state = {
	.offline_cnt = 0,
	.offline_cnt_max = 50,
	.status = DEV_OFFLINE,
};
Motor_RM_State_t launcher_RM_3508L_state = {
	.offline_cnt = 0,
	.offline_cnt_max = 50,
	.status = DEV_OFFLINE,
};
Motor_RM_State_t launcher_RM_3508R_state = {
	.offline_cnt = 0,
	.offline_cnt_max = 50,
	.status = DEV_OFFLINE,
};

pid_ctrl_t Launcher_6020_Ctrl_Speed = {
	.target = 0,
	.measure = 0,
	.err = 0,
	.last_err = 0,
	.kp = 0,
	.ki = 0,
	.kd = 0,
	.pout = 0,
	.iout = 0,
	.dout = 0,
	.out = 0,
	.last_dout = 0,
	.integral = 0,
	.integral_max = 0,
	.out_max = 0,
};
pid_ctrl_t Launcher_6020_Ctrl_Position_inner = {
	.target = 0,
	.measure = 0,
	.err = 0,
	.last_err = 0,
	.kp = 120,
	.ki = 0.2,
	.kd = 0,
	.pout = 0,
	.iout = 0,
	.dout = 0,
	.out = 0,
	.last_dout = 0,
	.integral = 0,
	.integral_max = 22000,
	.out_max = 24000,
};
pid_ctrl_t Launcher_6020_Ctrl_Position_outer = {
	.target = 0,
	.measure = 0,
	.err = 0,
	.last_err = 0,
	.kp = 0.4,
	.ki = 0,
	.kd = 0,
	.pout = 0,
	.iout = 0,
	.dout = 0,
	.out = 0,
	.last_dout = 0,
	.integral = 0,
	.integral_max = 0,
	.out_max = 50,
};

pid_ctrl_t Launcher_3508L_Ctrl_Speed = {
	.target = 0,
	.measure = 0,
	.err = 0,
	.last_err = 0,
	.kp = 1,
	.ki = 0.1,
	.kd = 0,
	.pout = 0,
	.iout = 0,
	.dout = 0,
	.out = 0,
	.last_dout = 0,
	.integral = 0,
	.integral_max = 8000,
	.out_max = 16000,
};
pid_ctrl_t Launcher_3508L_Ctrl_Position_inner = {
	.target = 0,
	.measure = 0,
	.err = 0,
	.last_err = 0,
	.kp = 18,
	.ki = 0.45,
	.kd = 0,
	.pout = 0,
	.iout = 0,
	.dout = 0,
	.out = 0,
	.last_dout = 0,
	.integral = 0,
	.integral_max = 20000,
	.out_max = 16000,
};
pid_ctrl_t Launcher_3508L_Ctrl_Position_outer = {
	.target = 0,
	.measure = 0,
	.err = 0,
	.last_err = 0,
	.kp = 0.4,
	.ki = 0,
	.kd = 0,
	.pout = 0,
	.iout = 0,
	.dout = 0,
	.out = 0,
	.last_dout = 0,
	.integral = 0,
	.integral_max = 0,
	.out_max = 6000,
};

pid_ctrl_t Launcher_3508R_Ctrl_Speed = {
	.target = 0,
	.measure = 0,
	.err = 0,
	.last_err = 0,
	.kp = 1.0,
	.ki = 0.1,
	.kd = 0,
	.pout = 0,
	.iout = 0,
	.dout = 0,
	.out = 0,
	.last_dout = 0,
	.integral = 0,
	.integral_max = 10000   ,
	.out_max = 16000,
};
pid_ctrl_t Launcher_3508R_Ctrl_Position_inner = {
	.target = 0,
	.measure = 0,
	.err = 0,
	.last_err = 0,
	.kp = 0,
	.ki = 0,
	.kd = 0,
	.pout = 0,
	.iout = 0,
	.dout = 0,
	.out = 0,
	.last_dout = 0,
	.integral = 0,
	.integral_max = 0,
	.out_max = 0,
};
pid_ctrl_t Launcher_3508R_Ctrl_Position_outer = {
	.target = 0,
	.measure = 0,
	.err = 0,
	.last_err = 0,
	.kp = 0,
	.ki = 0,
	.kd = 0,
	.pout = 0,
	.iout = 0,
	.dout = 0,
	.out = 0,
	.last_dout = 0,
	.integral = 0,
	.integral_max = 0,
	.out_max = 0,
};

Motor_RM_Ctrl_Info_t feeding_RM_6020_pid = {
	.speed_ctrl = &Launcher_6020_Ctrl_Speed,
	.angle_ctrl_inner = &Launcher_6020_Ctrl_Position_inner,
	.angle_ctrl_outer = &Launcher_6020_Ctrl_Position_outer,
};
Motor_RM_Ctrl_Info_t launcher_RM_3508L_pid = {
	.speed_ctrl = &Launcher_3508L_Ctrl_Speed,
	.angle_ctrl_inner = &Launcher_3508L_Ctrl_Position_inner,
	.angle_ctrl_outer = &Launcher_3508L_Ctrl_Position_outer,
};
Motor_RM_Ctrl_Info_t launcher_RM_3508R_pid = {
	.speed_ctrl = &Launcher_3508R_Ctrl_Speed,
	.angle_ctrl_inner = &Launcher_3508R_Ctrl_Position_inner,
	.angle_ctrl_outer = &Launcher_3508R_Ctrl_Position_outer,
};

Motor_RM_t launcher_motor[LAUNCHER_MOTOR_LIST] =
	{
		[FEEDING_MOTOR_RM6020] = {
			.born_info = &feeding_RM_6020_Born_info,
			.rx_info = &feeding_RM_6020_rx_info,
			.tx_info = &feeding_RM_6020_tx_info,
			.state = &feeding_RM_6020_state,
			.ctrl = &feeding_RM_6020_pid,
			.single_init = RM_Motor_Init,

		},
		[LAUNCHER_MOTOR_RM3508L] = {
			.born_info = &launcher_RM_3508L_Born_info,
			.rx_info = &launcher_RM_3508L_rx_info,
			.tx_info = &launcher_RM_3508L_tx_info,
			.state = &launcher_RM_3508L_state,
			.ctrl = &launcher_RM_3508L_pid,
			.single_init = RM_Motor_Init,

		},
		[LAUNCHER_MOTOR_RM3508R] = {
			.born_info = &launcher_RM_3508R_Born_info,
			.rx_info = &launcher_RM_3508R_rx_info,
			.tx_info = &launcher_RM_3508R_tx_info,
			.state = &launcher_RM_3508R_state,
			.ctrl = &launcher_RM_3508R_pid,
			.single_init = RM_Motor_Init,
		},
};

void launcher_motor_init(void)
{
	for (uint8_t i = 0; i < LAUNCHER_MOTOR_LIST; i++)
	{
		launcher_motor[i].single_init(&launcher_motor[i]);
	}
}

void launcher_motor_heart_beat(void)
{
	for (uint8_t i = 0; i < LAUNCHER_MOTOR_LIST; i++)
	{
		launcher_motor[i].single_heart_beat(&launcher_motor[i]);
	}
}

servo_config_t launcher_shootservo_config = {
	.sleep = 500,
	.lock = 500,
	.release = 880,
};

servo_config_t feeding_secondservo_config = {
	.sleep = 1300,
	.lock = 1300,
	//	.sleep = 1850,
	//	.lock =1850,
	
	.release = 980,
	.shoot_target = 1300,
};

servo_config_t feeding_thirdservo_config = {
	.sleep = 1680,
	.lock = 1680,
	//	.sleep = 2000,
	//	.lock = 2000,
	.release = 1330,
	.shoot_target = 1680,
};

servo_config_t feeding_fourthservo_config = {
	.sleep = 1700	,
	.lock = 1700,
	//	.sleep = 1650,
	//	.lock = 1650,
	.release = 1400,
	.shoot_target = 1700,
};

// shoot_servo
void servo_shoot_sleep(void)
{
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, launcher_shootservo_config.sleep);
}

void servo_shoot_lock(void)
{
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, launcher_shootservo_config.lock);
}

void servo_shoot_release(void)
{
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, launcher_shootservo_config.release);
}

// feeding_second_servo
void servo_feeding4_1_sleep(void)
{
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, feeding_secondservo_config.sleep);
}

void servo_feeding4_1_lock(void)
{
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, feeding_secondservo_config.lock);
}

void servo_feeding4_1_release(void)
{
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, feeding_secondservo_config.release);
}

void servo_feeding4_1_shoot(void)
{
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, feeding_secondservo_config.shoot_target);
}

// void servo_feeding2_2_sleep(void)
//{
//	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,launcher_shootservo_config.sleep);
// }

// void servo_feeding2_2_lock(void)
//{
//	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,launcher_shootservo_config.lock);
// }

// void servo_feeding2_2_release(void)
//{
//	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,launcher_shootservo_config.release);
// }

// feeding_thrid_servo
void servo_feeding3_1_sleep(void)
{
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, feeding_thirdservo_config.sleep);
}

void servo_feeding3_1_lock(void)
{
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, feeding_thirdservo_config.lock);
}

void servo_feeding3_1_release(void)
{
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, feeding_thirdservo_config.release);
}

void servo_feeding3_1_shoot(void)
{
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, feeding_thirdservo_config.shoot_target);
}
// void servo_feeding3_2_sleep(void)
//{
//	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,launcher_shootservo_config.sleep);
// }

// void servo_feeding3_2_lock(void)
//{
//	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,launcher_shootservo_config.lock);
// }

// void servo_feeding3_2_release(void)
//{
//	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,launcher_shootservo_config.release);
// }

// feeding_fourth_servo
void servo_feeding2_1_sleep(void)
{
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, feeding_fourthservo_config.sleep);
}

void servo_feeding2_1_lock(void)
{
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, feeding_fourthservo_config.lock);
}

void servo_feeding2_1_release(void)
{
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, feeding_fourthservo_config.release);
}

void servo_feeding2_1_shoot(void)
{
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, feeding_fourthservo_config.shoot_target);
}
// void servo_feeding4_2_sleep(void)
//{
//	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,launcher_shootservo_config.sleep);
// }

// void servo_feeding4_2_lock(void)
//{
//	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,launcher_shootservo_config.lock);
// }

// void servo_feeding4_2_release(void)
//{
//	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,launcher_shootservo_config.release);
// }

void electromagnet4_on(void)
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
}

void electromagnet4_off(void)
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
}

void electromagnet3_on(void)
{
	HAL_GPIO_WritePin(GPIOI, GPIO_PIN_6, GPIO_PIN_SET);
}

void electromagnet3_off(void)
{
	HAL_GPIO_WritePin(GPIOI, GPIO_PIN_6, GPIO_PIN_RESET);
}

void electromagnet2_on(void)
{
	HAL_GPIO_WritePin(GPIOI, GPIO_PIN_7, GPIO_PIN_SET);
}

void electromagnet2_off(void)
{
	HAL_GPIO_WritePin(GPIOI, GPIO_PIN_7, GPIO_PIN_RESET);
}

/**
 *@brief �����ת�жϺ���
 *
 */
bool motor_stick_check(Motor_RM_t *motor)
{
	static uint8_t stick_cnt = 0;
	if (abs(motor->tx_info->torque_current) >= 1000 && motor->rx_info->encoder_speed <= 10)
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
