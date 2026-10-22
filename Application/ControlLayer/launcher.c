#include "launcher.h"
#include "rp_math.h"

TaskHandle_t xLauncherTaskHandle;

void Launcher_Work(launcher_t *launcher);

launcher_t launcher =
	{
		.microswitch_info = {
			.microswitch_last_state = LAUNCHER_MICROSWITCH_OFF,
			.microswitch_state = LAUNCHER_MICROSWITCH_OFF,
			.min_keep_cnt = 50,
		},
		.config = {
			.motor6020_standby_angle = {7480, 4720, 2020}, 
			.motor6020_load_angle = {6120, 3380, 625},
			.motor6020_angle_ramp_rate = 3,
			//			.motor6020_standby_angle = {4660, 7365, 1870},
			//			.motor6020_load_angle = {6123, 643, 3392},
			//			.motor6020_standby_angle = {4660,6120 ,7365 },
			//			.motor6020_load_angle = {660,1870 ,3390},
			.motor3508_firstmaxspeed = 5000,
			.motor3508_initmaxspeed = -1000,
			.motor3508_to_middle_maxspeed = 1200,
			.motor3508_othermaxspeed = 7500,
			.motor3508_protect_position = 0,
			.motor3508_up_position = 20000,
			.motor3508_middle_position = 1300000,
			.motor3508_down_position = 1340000, // 1343918----->1343700
		},

		.motors = {
			.motor6020 = &launcher_motor[FEEDING_MOTOR_RM6020],
			.motor3508L = &launcher_motor[LAUNCHER_MOTOR_RM3508L],
			.motor3508R = &launcher_motor[LAUNCHER_MOTOR_RM3508R],
		},
		.work = Launcher_Work,
};

/**
 * @brief 发射PID计算
 *
 * @param launcher
 */
#define HISTORY_SIZE (20)
int16_t speed_history[HISTORY_SIZE] = {0}; // 速度历史记录
float err_history[HISTORY_SIZE] = {0};
uint8_t history_index = 0; // 历史记录索引
float average_speed = 0;
float average_err = 0;
float current_err = 0;
void Launcher_6020_Pid_Calculating(launcher_t *launcher)
{
	// 6020电机PID计算
	switch (launcher->motor6020_info.ctrl_mode)
	{
	case MOTOR_UNFORCE:
		launcher->motor6020_info.output_current = 0;
		break;
	case MOTOR_PID_ANGLE:
	{
		int16_t ramp_rate = launcher->config.motor6020_angle_ramp_rate;
		if (ramp_rate > 0)
		{
			float ramp_err = launcher->motor6020_info.target_angle - launcher->motor6020_info.target_angle_smoothed;
			ramp_err = motor_half_cycle(ramp_err, 8191);
			ramp_err = constrain(ramp_err, -ramp_rate, ramp_rate);
			launcher->motor6020_info.target_angle_smoothed += ramp_err;
			if (launcher->motor6020_info.target_angle_smoothed >= 8192)
				launcher->motor6020_info.target_angle_smoothed -= 8192;
			else if (launcher->motor6020_info.target_angle_smoothed < 0)
				launcher->motor6020_info.target_angle_smoothed += 8192;
			launcher->motors.motor6020->ctrl->angle_ctrl_outer->target = launcher->motor6020_info.target_angle_smoothed;
		}
		else
		{
			launcher->motors.motor6020->ctrl->angle_ctrl_outer->target = launcher->motor6020_info.target_angle;
		}
	}
		launcher->motors.motor6020->ctrl->angle_ctrl_outer->measure = launcher->motors.motor6020->rx_info->encoder;

		launcher->motors.motor6020->ctrl->angle_ctrl_outer->err = launcher->motors.motor6020->ctrl->angle_ctrl_outer->target - launcher->motors.motor6020->ctrl->angle_ctrl_outer->measure;
		launcher->motors.motor6020->ctrl->angle_ctrl_outer->err = motor_half_cycle(launcher->motors.motor6020->ctrl->angle_ctrl_outer->err, 8191);
		current_err = launcher->motors.motor6020->ctrl->angle_ctrl_outer->err; // 当前误差
		single_pid_ctrl(launcher->motors.motor6020->ctrl->angle_ctrl_outer);

		launcher->motors.motor6020->ctrl->angle_ctrl_inner->target = launcher->motors.motor6020->ctrl->angle_ctrl_outer->out;
		launcher->motors.motor6020->ctrl->angle_ctrl_inner->measure = launcher->motors.motor6020->rx_info->encoder_speed;
		launcher->motors.motor6020->ctrl->angle_ctrl_inner->err = launcher->motors.motor6020->ctrl->angle_ctrl_inner->target - launcher->motors.motor6020->ctrl->angle_ctrl_inner->measure;
		single_pid_ctrl(launcher->motors.motor6020->ctrl->angle_ctrl_inner);
		launcher->motor6020_info.output_current = launcher->motors.motor6020->ctrl->angle_ctrl_inner->out;

		break;
	case MOTOR_PID_SPEED:
		launcher->motors.motor6020->ctrl->speed_ctrl->target = -launcher->motor6020_info.target_speed;
		launcher->motors.motor6020->ctrl->speed_ctrl->measure = launcher->motors.motor6020->rx_info->encoder_speed;
		launcher->motors.motor6020->ctrl->speed_ctrl->err = launcher->motors.motor6020->ctrl->speed_ctrl->target - launcher->motors.motor6020->ctrl->speed_ctrl->measure;
		current_err = launcher->motors.motor6020->ctrl->speed_ctrl->err; // 当前误差
		single_pid_ctrl(launcher->motors.motor6020->ctrl->speed_ctrl);
		launcher->motor6020_info.output_current = launcher->motors.motor6020->ctrl->speed_ctrl->out;

		break;
	default:
		break;
	}

	int16_t current_speed = launcher->motors.motor6020->rx_info->encoder_speed; // 当前速度
	// 更新速度历史记录
	speed_history[history_index] = abs(current_speed); // 将当前速度添加到历史记录中
	// 计算平均速度
	int32_t speed_sum = 0;
	for (uint8_t i = 0; i < HISTORY_SIZE; i++)
	{
		speed_sum += speed_history[i];
	}
	average_speed = speed_sum / HISTORY_SIZE; // 计算平均速度

	// 更新误差历史记录
	err_history[history_index] = abs(current_err);
	//
	int32_t err_sum = 0;
	for (uint8_t i = 0; i < HISTORY_SIZE; i++)
	{
		err_sum += err_history[i];
	}
	average_err = err_sum / HISTORY_SIZE; // 计算平均误差

	history_index = (history_index + 1) % HISTORY_SIZE; // 更新索引
}
/**
 * @brief 发射PID计算
 *
 * @param launcher
 */
void Launcher_3508_Pid_Calculating(launcher_t *launcher)
{
	// 3508电机PID计算
	switch (launcher->motor3508_info.ctrl_mode)
	{
	case MOTOR_UNFORCE:
		launcher->motor3508_info.motor3508L_output_current = 0;
		launcher->motor3508_info.motor3508R_output_current = 0;
		break;
	case MOTOR_PID_POSITION:
		//		launcher->motors.motor3508R->ctrl->angle_ctrl_outer->target = launcher->motor3508_info.motor3508R_target_position;
		//		launcher->motors.motor3508R->ctrl->angle_ctrl_outer->measure = launcher->motors.motor3508R->rx_info->encoder_sum;

		//		launcher->motors.motor3508R->ctrl->angle_ctrl_outer->err = launcher->motors.motor3508R->ctrl->angle_ctrl_outer->target - launcher->motors.motor3508R->ctrl->angle_ctrl_outer->measure;
		//		single_pid_ctrl(launcher->motors.motor3508R->ctrl->angle_ctrl_outer);

		//		launcher->motors.motor3508R->ctrl->angle_ctrl_inner->target = launcher->motors.motor3508R->ctrl->angle_ctrl_outer->out;
		//		launcher->motors.motor3508R->ctrl->angle_ctrl_inner->measure = launcher->motors.motor3508R->rx_info->encoder_speed;
		//		launcher->motors.motor3508R->ctrl->angle_ctrl_inner->err = launcher->motors.motor3508R->ctrl->angle_ctrl_inner->target - launcher->motors.motor3508R->ctrl->angle_ctrl_inner->measure;
		//		single_pid_ctrl(launcher->motors.motor3508R->ctrl->angle_ctrl_inner);
		//		launcher->motor3508_info.motor3508R_output_current = launcher->motors.motor3508R->ctrl->angle_ctrl_inner->out;
		//		launcher->motor3508_info.motor3508L_output_current = -launcher->motors.motor3508R->ctrl->angle_ctrl_inner->out;

		launcher->motors.motor3508L->ctrl->angle_ctrl_outer->target = launcher->motor3508_info.motor3508L_init_angle_sum + launcher->motor3508_info.motor3508L_target_position;
		launcher->motors.motor3508L->ctrl->angle_ctrl_outer->measure = launcher->motors.motor3508L->rx_info->encoder_sum;

		launcher->motors.motor3508L->ctrl->angle_ctrl_outer->err = launcher->motors.motor3508L->ctrl->angle_ctrl_outer->target - launcher->motors.motor3508L->ctrl->angle_ctrl_outer->measure;
		single_pid_ctrl(launcher->motors.motor3508L->ctrl->angle_ctrl_outer);

		launcher->motors.motor3508L->ctrl->angle_ctrl_inner->target = launcher->motors.motor3508L->ctrl->angle_ctrl_outer->out;
		launcher->motors.motor3508L->ctrl->angle_ctrl_inner->measure = launcher->motors.motor3508L->rx_info->encoder_speed;
		launcher->motors.motor3508L->ctrl->angle_ctrl_inner->err = launcher->motors.motor3508L->ctrl->angle_ctrl_inner->target - launcher->motors.motor3508L->ctrl->angle_ctrl_inner->measure;
		single_pid_ctrl(launcher->motors.motor3508L->ctrl->angle_ctrl_inner);
		launcher->motor3508_info.motor3508L_output_current = launcher->motors.motor3508L->ctrl->angle_ctrl_inner->out;
		launcher->motor3508_info.motor3508R_output_current = -launcher->motors.motor3508L->ctrl->angle_ctrl_inner->out;
		launcher->motors.motor3508L->tx_info->torque_current = launcher->motor3508_info.motor3508L_output_current;
		launcher->motors.motor3508R->tx_info->torque_current = launcher->motor3508_info.motor3508R_output_current;
		break;
	case MOTOR_PID_SPEED:
		launcher->motors.motor3508R->ctrl->speed_ctrl->target = -launcher->motor3508_info.target_speed;
		launcher->motors.motor3508R->ctrl->speed_ctrl->measure = launcher->motors.motor3508R->rx_info->encoder_speed;
		launcher->motors.motor3508R->ctrl->speed_ctrl->err = launcher->motors.motor3508R->ctrl->speed_ctrl->target - launcher->motors.motor3508R->ctrl->speed_ctrl->measure;
		single_pid_ctrl(launcher->motors.motor3508R->ctrl->speed_ctrl);
		launcher->motor3508_info.motor3508R_output_current = launcher->motors.motor3508R->ctrl->speed_ctrl->out;
		//		launcher->motor3508_info.motor3508L_output_current = - launcher->motors.motor3508R->ctrl->speed_ctrl->out;

		launcher->motors.motor3508L->ctrl->speed_ctrl->target = launcher->motor3508_info.target_speed;
		launcher->motors.motor3508L->ctrl->speed_ctrl->measure = launcher->motors.motor3508L->rx_info->encoder_speed;
		launcher->motors.motor3508L->ctrl->speed_ctrl->err = launcher->motors.motor3508L->ctrl->speed_ctrl->target - launcher->motors.motor3508L->ctrl->speed_ctrl->measure;
		single_pid_ctrl(launcher->motors.motor3508L->ctrl->speed_ctrl);
		launcher->motor3508_info.motor3508L_output_current = launcher->motors.motor3508L->ctrl->speed_ctrl->out;
		//		launcher->motor3508_info.motor3508R_output_current = - launcher->motors.motor3508R->ctrl->speed_ctrl->out;
		launcher->motors.motor3508L->tx_info->torque_current = launcher->motor3508_info.motor3508L_output_current;
		launcher->motors.motor3508R->tx_info->torque_current = launcher->motor3508_info.motor3508R_output_current;
		break;
	default:
		break;
	}
	launcher->motor3508_info.err_between_3508 = abs(launcher->motors.motor3508L->rx_info->encoder_sum) - abs(launcher->motors.motor3508R->rx_info->encoder_sum);
	//	if(abs(launcher->motor3508_info.err_between_3508) > 30000)
	//	{
	//		launcher->motors.motor3508L->ctrl->angle_ctrl_inner->out = 0;
	//		launcher->motors.motor3508R->ctrl->angle_ctrl_inner->out = 0;
	//		launcher->motor3508_info.motor3508L_output_current = 0;
	//		launcher->motor3508_info.motor3508R_output_current = 0;
	//	}
}

/**
 * @brief 计算电机位置
 *
 * @param launcher
 */
void Launcher_Motor_Position_Cal(launcher_t *launcher)
{
	// 3508电机位置计算
	launcher->motor3508_info.motor3508L_position = launcher->motors.motor3508L->rx_info->encoder_sum - launcher->motor3508_info.motor3508L_init_angle_sum;
	launcher->motor3508_info.motor3508R_position = launcher->motors.motor3508R->rx_info->encoder_sum + launcher->motor3508_info.motor3508L_init_angle_sum;
}

/**
 * @brief 计算供弹6020电机初始化角度
 *
 * @param launcher
 * @return uint16_t 返回最小误差角度对应的初始值
 */
uint16_t Launcher_6020_Init_Angle_Cal(launcher_t *launcher)
{
	int16_t err_angle[3] = {0};
	// 计算离哪个角度最近
	for (size_t i = 0; i < 3; i++)
	{
		err_angle[i] = launcher->config.motor6020_standby_angle[i] -
					   launcher->motors.motor6020->rx_info->encoder;
	}
	// 半圈处理
	for (size_t i = 0; i < 3; i++)
	{
		err_angle[i] = motor_half_cycle(err_angle[i], 8192);
	}
	// 取最小值绝对值
	uint16_t min_err_angle = abs(err_angle[0]);
	uint8_t min_index = 0;
	for (size_t i = 1; i < 3; i++)
	{
		if (abs(err_angle[i]) < abs(min_err_angle))
		{
			min_err_angle = abs(err_angle[i]);
			min_index = i;
		}
	}
	return launcher->config.motor6020_standby_angle[min_index];
}

/**
 * @brief 发射标志位复位
 *
 * @param launcher
 */
void Launcher_Reset(launcher_t *launcher)
{

	launcher->state = LAUNCHER_OFFLINE; // 发射器状态设置为离线

	launcher->motor3508_info.state = LAUNCHER_SUBMODULE_OFFLINE; // 3508电机状态设置为未初始化
	launcher->motor6020_info.state = LAUNCHER_SUBMODULE_OFFLINE; // 供弹6020电机状态设置为未初始化

	launcher->feedServo_info.second_servo1_target = LAUNCHER_SERVO_SLEEP; // 第二个舵机设置为休眠
	launcher->feedServo_info.third_servo1_target = LAUNCHER_SERVO_SLEEP;  // 第三个舵机设置为休眠
	launcher->feedServo_info.fourth_servo1_target = LAUNCHER_SERVO_SLEEP; // 第四个舵机设置为休眠
																		  //	launcher->feedServo_info.second_servo2_target = LAUNCHER_SERVO_SLEEP; // 第二个舵机设置为休眠
																		  //	launcher->feedServo_info.third_servo2_target = LAUNCHER_SERVO_SLEEP; // 第三个舵机设置为休眠
																		  //	launcher->feedServo_info.fourth_servo2_target = LAUNCHER_SERVO_SLEEP; // 第四个舵机设置为休眠

	launcher->fireServo_info.shoot_servo_target = LAUNCHER_SERVO_SLEEP; // 发射舵机设置为休眠

	launcher->motor3508_info.motor3508L_output_current = 0;
	launcher->motor3508_info.motor3508R_output_current = 0;
	launcher->motor6020_info.output_current = 0;

	launcher->motor3508_info.target_speed = 0; // 上膛3508电机目标速度设置为0

	launcher->motor3508_info.ctrl_mode = MOTOR_UNFORCE;
	launcher->motor6020_info.ctrl_mode = MOTOR_UNFORCE;

	launcher->task_info.launcher_initializing_state = LAUNCHER_INIT_START; // 初始化步骤设置为开始
	launcher->task_info.launcher_load_state = LAUNCHER_LOAD_START;		   // 装弹步骤设置为开始
	launcher->task_info.launcher_shoot_state = LAUNCHER_SHOOT_START;	   // 发射步骤设置为开始
}

/**
 * @brief 发射器3508电机状态控制器
 *
 * @param launcher
 */
void Launcher_Motor3508_State_Controler(launcher_t *launcher)
{
	if (launcher->motor3508_info.state < LAUNCHER_SUBMODULE_INITED)
	{
		return;
	}
	// launcher->motor3508_info.ctrl_mode = MOTOR_PID_SPEED;
	// launcher->motor3508_info.target_state = launcher->config.motor3508_initmaxspeed;
	switch (launcher->motor3508_info.target_state)
	{
	case LAUNCHER_3508_UP:
		launcher->motor3508_info.ctrl_mode = MOTOR_PID_POSITION;
		launcher->motor3508_info.motor3508L_target_position = launcher->config.motor3508_up_position;
		break;
	case LAUNCHER_3508_MIDDLE:
		launcher->motor3508_info.ctrl_mode = MOTOR_PID_POSITION;
		launcher->motor3508_info.motor3508L_target_position = launcher->config.motor3508_middle_position;
		break;
	case LAUNCHER_3508_DOWN:
		launcher->motor3508_info.ctrl_mode = MOTOR_PID_POSITION;
		launcher->motor3508_info.motor3508L_target_position = launcher->config.motor3508_down_position;
		break;
	case LAUNCHER_3508_UNFORCE:
		launcher->motor3508_info.ctrl_mode = MOTOR_UNFORCE;
		break;
	default:
		break;
	}

	if (launcher->motor3508_info.target_state == LAUNCHER_3508_UNFORCE)
	{															  // 如果卸力
		launcher->motor3508_info.state = LAUNCHER_SUBMODULE_DONE; // 达到目标值
		return;
	}

	int32_t target_err = launcher->motor3508_info.motor3508L_target_position -
						 launcher->motor3508_info.motor3508L_position;

	int16_t l_speed = launcher->motors.motor3508L->rx_info->speed; // 电机速度
	if (abs(target_err) < 400 && abs(l_speed) < 20)
	{
		launcher->motor3508_info.state = LAUNCHER_SUBMODULE_DONE; // 达到目标值
	}
	else
	{
		launcher->motor3508_info.state = LAUNCHER_SUBMODULE_MOVING; // 没有达到目标值
	}
}

/**
 * @brief 发射器供弹6020电机状态控制器
 *
 * @param launcher
 */
void Launcher_Motor6020_State_Controler(launcher_t *launcher)
{
	if (launcher->motor6020_info.state < LAUNCHER_SUBMODULE_INITED)
	{
		return; // 如果供弹6020电机还没有初始化完成
	}
	switch (launcher->motor6020_info.target_state)
	{
	case LAUNCHER_6020_SECOND_STANDBY:
		launcher->motor6020_info.target_angle = launcher->config.motor6020_standby_angle[0];
		break;
	case LAUNCHER_6020_THIRD_STANDBY:
		launcher->motor6020_info.target_angle = launcher->config.motor6020_standby_angle[1];
		break;
	case LAUNCHER_6020_FOURTH_STANDBY:
		launcher->motor6020_info.target_angle = launcher->config.motor6020_standby_angle[2];
		break;
	case LAUNCHER_6020_SECOND_LOAD:
		launcher->motor6020_info.target_angle = launcher->config.motor6020_load_angle[0];
		break;
	case LAUNCHER_6020_THIRD_LOAD:
		launcher->motor6020_info.target_angle = launcher->config.motor6020_load_angle[1];
		break;
	case LAUNCHER_6020_FOURTH_LOAD:
		launcher->motor6020_info.target_angle = launcher->config.motor6020_load_angle[2];
		break;
	default:
		break;
	}

	// float d_err = launcher->motors.motor6020->pid.angle.info.err -
	// 							launcher->motors.motor6020->pid.angle.info.last_err; // 计算误差变化量
	float angle_err = launcher->motors.motor6020->rx_info->encoder -
					  launcher->motor6020_info.target_angle; // 计算角度误差
	angle_err = motor_half_cycle(angle_err, 8192);
	if (launcher->motor6020_info.target_state == LAUNCHER_6020_SECOND_STANDBY)
	{
		if (abs(angle_err) < 15 && average_err < 20)
		{
			launcher->motor6020_info.state = LAUNCHER_SUBMODULE_DONE; // 达到目标值
		}
		else
		{
			launcher->motor6020_info.state = LAUNCHER_SUBMODULE_MOVING; // 没有达到目标值
		}
	}
	else
	{
		if (abs(angle_err) < 5 && average_err < 10)
		{
			launcher->motor6020_info.state = LAUNCHER_SUBMODULE_DONE; // 达到目标值
		}
		else
		{
			launcher->motor6020_info.state = LAUNCHER_SUBMODULE_MOVING; // 没有达到目标值
		}
	}
}

/**
 * @brief 发射器舵机控制器
 *
 * @param launcher
 */
void Launcher_Servo_Controler(launcher_t *launcher)
{
	// 供弹舵机
	switch (launcher->feedServo_info.second_servo1_target)
	{
	case LAUNCHER_SERVO_LOCK:
		servo_feeding2_1_lock(); // 供弹第二个舵机锁定
		break;
	case LAUNCHER_SERVO_RELEASE:
		servo_feeding2_1_release(); // 供弹第二个舵机释放
		break;
	case LAUNCHER_SERVO_SHOOT_TARGET:
		servo_feeding2_1_shoot();
		break;
	default:
		servo_feeding2_1_sleep(); // 供弹第二个舵机休眠
		break;
	}
	switch (launcher->feedServo_info.third_servo1_target)
	{
	case LAUNCHER_SERVO_LOCK:
		servo_feeding3_1_lock(); // 供弹第三个舵机锁定
		break;
	case LAUNCHER_SERVO_RELEASE:
		servo_feeding3_1_release(); // 供弹第三个舵机释放
		break;
	case LAUNCHER_SERVO_SHOOT_TARGET:
		servo_feeding3_1_shoot();
		break;
	default:
		servo_feeding3_1_sleep(); // 供弹第三个舵机休眠
		break;
	}
	switch (launcher->feedServo_info.fourth_servo1_target)
	{
	case LAUNCHER_SERVO_LOCK:
		servo_feeding4_1_lock(); // 供弹第四个舵机锁定
		break;
	case LAUNCHER_SERVO_RELEASE:
		servo_feeding4_1_release(); // 供弹第四个舵机释放
		break;
	case LAUNCHER_SERVO_SHOOT_TARGET:
		servo_feeding4_1_shoot();
		break;
	default:
		servo_feeding4_1_sleep(); // 供弹第四个舵机休眠
		break;
	}
	//		// 供弹舵机
	//	switch (launcher->feedServo_info.second_servo2_target)
	//	{
	//	case LAUNCHER_SERVO_LOCK:
	//		servo_feeding2_2_lock(); // 供弹第二个舵机锁定
	//		break;
	//	case LAUNCHER_SERVO_RELEASE:
	//		servo_feeding2_2_release(); // 供弹第二个舵机释放
	//		break;
	//	default:
	//		servo_feeding2_2_sleep(); // 供弹第二个舵机休眠
	//		break;
	//	}
	//	switch (launcher->feedServo_info.third_servo2_target)
	//	{
	//	case LAUNCHER_SERVO_LOCK:
	//		servo_feeding3_2_lock(); // 供弹第三个舵机锁定
	//		break;
	//	case LAUNCHER_SERVO_RELEASE:
	//		servo_feeding3_2_release(); // 供弹第三个舵机释放
	//		break;
	//	default:
	//		servo_feeding3_2_sleep(); // 供弹第三个舵机休眠
	//		break;
	//	}
	//	switch (launcher->feedServo_info.fourth_servo2_target)
	//	{
	//	case LAUNCHER_SERVO_LOCK:
	//		servo_feeding4_2_lock(); // 供弹第四个舵机锁定
	//		break;
	//	case LAUNCHER_SERVO_RELEASE:
	//		servo_feeding4_2_release(); // 供弹第四个舵机释放
	//		break;
	//	default:
	//		servo_feeding4_2_sleep(); // 供弹第四个舵机休眠
	//		break;
	//	}
	// 发射舵机
	switch (launcher->fireServo_info.shoot_servo_target)
	{
	case LAUNCHER_SERVO_LOCK:
		servo_shoot_lock(); // 发射舵机锁定
		break;
	case LAUNCHER_SERVO_RELEASE:
		servo_shoot_release(); // 发射舵机释放
		break;
	default:
		servo_shoot_sleep(); // 发射舵机休眠
		break;
	}
}

/**
 *@brief 电磁铁控制函数
 *
 */
void Launcher_Electromagnet_Controler(launcher_t *launcher)
{
	switch (launcher->electromagnet_info.second_electromagnet_target)
	{
	case LAUNCHER_ELECTROMAGNET_ON:
		electromagnet2_on();
		break;
	case LAUNCHER_ELECTROMAGNET_OFF:
		electromagnet2_off();
		break;
	default:
		break;
	}
	switch (launcher->electromagnet_info.third_electromagnet_target)
	{
	case LAUNCHER_ELECTROMAGNET_ON:
		electromagnet3_on();
		break;
	case LAUNCHER_ELECTROMAGNET_OFF:
		electromagnet3_off();
		break;
	default:
		break;
	}

	switch (launcher->electromagnet_info.fourth_electromagnet_target)
	{
	case LAUNCHER_ELECTROMAGNET_ON:
		electromagnet4_on();
		break;
	case LAUNCHER_ELECTROMAGNET_OFF:
		electromagnet4_off();
		break;
	default:
		break;
	}
}

/**
 *@biref 微动开关状态检测
 *
 */
void mircoswitch_check(void)
{
	launcher_microswitch_tate_e now_microswitch_tate;
	static uint16_t satate_keep_tick = 0;
	if (HAL_GPIO_ReadPin((GPIO_TypeDef *)0x40020400U, GPIO_PIN_12) == GPIO_PIN_SET)
	{
		now_microswitch_tate = LAUNCHER_MICROSWITCH_ON;
	}
	else
	{
		now_microswitch_tate = LAUNCHER_MICROSWITCH_OFF;
	}
	if (now_microswitch_tate != launcher.microswitch_info.microswitch_state && satate_keep_tick < launcher.microswitch_info.min_keep_cnt)
	{
		satate_keep_tick++;
	}
	else if (now_microswitch_tate != launcher.microswitch_info.microswitch_state && satate_keep_tick >= launcher.microswitch_info.min_keep_cnt)
	{
		launcher.microswitch_info.microswitch_state = now_microswitch_tate;
	}
	else
	{
		satate_keep_tick = 0;
	}
}

/**
 * @brief 发射器初始化任务
 *
 * @param launcher
 */
void Laucher_Init_Task(launcher_t *launcher)
{
	uint16_t wait_servo_work_tick = 500;
	static uint32_t init_start_tick = 0;
	static uint32_t shootServo_start_tick = 0;

	switch (launcher->task_info.launcher_initializing_state)
	{
	case LAUNCHER_INIT_START: // 0
		// set 3508 max speed
		launcher->motors.motor3508R->ctrl->angle_ctrl_outer->out_max = launcher->config.motor3508_firstmaxspeed; // 设置3508电机第一次最大速度
		// pid mode setting
		launcher->motor3508_info.ctrl_mode = MOTOR_PID_SPEED;
		launcher->motor6020_info.ctrl_mode = MOTOR_PID_ANGLE;
		// servo target setting
		launcher->fireServo_info.shoot_servo_target = LAUNCHER_SERVO_LOCK;	 // 撒放器舵机锁定
		launcher->feedServo_info.second_servo1_target = LAUNCHER_SERVO_LOCK; // 第二个舵机设置为锁定
		launcher->feedServo_info.third_servo1_target = LAUNCHER_SERVO_LOCK;	 // 第三个舵机设置为锁定
		launcher->feedServo_info.fourth_servo1_target = LAUNCHER_SERVO_LOCK; // 第四个舵机设置为锁定
																			 //		launcher->feedServo_info.second_servo2_target = LAUNCHER_SERVO_LOCK; // 第二个舵机设置为锁定
																			 //		launcher->feedServo_info.third_servo2_target = LAUNCHER_SERVO_LOCK; // 第三个舵机设置为锁定
																			 //		launcher->feedServo_info.fourth_servo2_target = LAUNCHER_SERVO_LOCK; // 第四个舵机设置为锁定
		// 6020 motor target angle setting
		launcher->motor6020_info.target_angle = Launcher_6020_Init_Angle_Cal(launcher);
		launcher->motor6020_info.target_angle_smoothed = launcher->motors.motor6020->rx_info->encoder;
		launcher->motor6020_info.state = LAUNCHER_SUBMODULE_INITIALIZING; // 供弹6020电机状态设置为初始化中
		launcher->state = LAUNCHER_INITIALIZING;						  // 发射器状态设置为初始化中
		init_start_tick = HAL_GetTick();								  // 记录初始化开始时间
		launcher->task_info.launcher_initializing_state++;				  // 进入下一个初始化步骤
		break;
	case LAUNCHER_INIT_STEP_WAIT_FEEDING_SERVO_TO_RELEASE:			// 1
		if (HAL_GetTick() - init_start_tick > wait_servo_work_tick) // 撒放器运行时间
		{
			launcher->task_info.launcher_initializing_state++; // 进入下一个初始化步骤
		}
		break;
	case LAUNCHER_INIT_STEP_WAIT_6020_TO_STANDBY: // 2

		if (abs(launcher->motors.motor6020->ctrl->angle_ctrl_outer->err) < 30 &&
			launcher->motors.motor6020->rx_info->speed < 10) // 6020电机到达准备位置
		{
			launcher->task_info.launcher_initializing_state++; // 进入下一个初始化步骤
		}
		break;
	case LAUNCHER_INIT_STEP_WAIT_SHOOT_SERVO_TO_RELEASE: // 3
		// 设置3508电机目标速度
		launcher->motor3508_info.target_speed = launcher->config.motor3508_initmaxspeed; // 上膛3508电机初始化速度
		// 设置3508电机状态
		launcher->motor3508_info.state = LAUNCHER_SUBMODULE_INITIALIZING;

		launcher->task_info.launcher_initializing_state++; // 进入下一个初始化步骤
		break;
	case LAUNCHER_INIT_STEP_WAIT_3508_TO_RELEASE: // 4
		// 等待3508电机堵转
		//		if (launcher->motors.motor3508R->c_stuck_flag(launcher->motors.motor3508R) == true)     //堵转判断->改成微动开关触发
		//		{
		//	if(launcher->microswitch_info.microswitch_state == LAUNCHER_MICROSWITCH_ON ||motor_stick_check(launcher->motors.motor3508L) == true)
		//	{
		if (motor_stick_check(launcher->motors.motor3508L) == true)
		{
			if (launcher->motor3508_info.state < LAUNCHER_SUBMODULE_INITED)
			{
				// 0点设置
				launcher->motor3508_info.motor3508L_init_angle_sum =
					launcher->motors.motor3508L->rx_info->encoder_sum;
				launcher->motor3508_info.motor3508R_init_angle_sum =
					launcher->motors.motor3508R->rx_info->encoder_sum;
				launcher->motor3508_info.target_state = LAUNCHER_3508_UP;	// 设置3508电机目标位置
				launcher->motor3508_info.ctrl_mode = MOTOR_PID_POSITION;	// 设置3508电机控制模式为位置控制
				launcher->motor3508_info.state = LAUNCHER_SUBMODULE_INITED; // 3508开始由状态控制器控制
			}
		}
		//	}
		//		}

		if (launcher->motor3508_info.state >= LAUNCHER_SUBMODULE_INITED)
		{
			launcher->motor6020_info.target_state = LAUNCHER_6020_SECOND_STANDBY; // 设置供弹6020电机目标状态为准备
			launcher->motor6020_info.state = LAUNCHER_SUBMODULE_INITED;			  // 供弹6020电机开始由状态控制器控制
			launcher->task_info.launcher_initializing_state++;					  // 进入下一个初始化步骤
		}
		break;
	case LAUNCHER_INIT_DONE:
		if (launcher->motor3508_info.state == LAUNCHER_SUBMODULE_DONE)
		{
			launcher->state = LAUNCHER_FREE; // 发射器状态设置为已初始化
		}
	default:
		break;
	}

	Launcher_Motor3508_State_Controler(launcher); // 3508电机状态控制
	Launcher_Motor6020_State_Controler(launcher); // 供弹6020电机状态控制
}

/**
 *@brief 发射器供弹任务
 *
 */
// void Launchert_Load_Dart_Task(launcher_t *launcher, launcher_6020_state_e load_target,
//							  launcher_6020_state_e standby_target, uint8_t target_id)
//{
//	static uint32_t load_start_tick = 0;
//	static uint32_t load_ok_tick = 0;
//	static uint32_t shoot_servo_start_to_work_tick = 0;
//	static uint32_t feed_servo_start_to_work_tick = 0;
//	static uint32_t electromagnet_start_to_work_tick = 0;
//	uint16_t wait_servo_work_tick = 1000;
//	uint16_t wait_electromagnet_work_tick = 100;

//	switch (launcher->task_info.launcher_load_state)
//	{
//	case LAUNCHER_LOAD_START: // 0
//		if (launcher->motor6020_info.target_state >= LAUNCHER_6020_SECOND_LOAD ||
//			launcher->motor6020_info.state == LAUNCHER_SUBMODULE_MOVING)
//		{
//			break; // 不在standby位置 或 供弹6020电机正在移动，则不进行加载
//		}
//		if (target_id == 1)
//		{
//			if (car.car_ctrl_mode == CAR_AT_MODE)
//			{
//				launcher->motors.motor3508L->ctrl->angle_ctrl_outer->out_max = launcher->config.motor3508_firstmaxspeed; // 设置3508电机加载速度
//				launcher->motors.motor3508R->ctrl->angle_ctrl_outer->out_max = launcher->config.motor3508_firstmaxspeed; // 设置3508电机加载速度
//			}
//			else
//			{
//				launcher->motors.motor3508L->ctrl->angle_ctrl_outer->out_max = launcher->config.motor3508_firstmaxspeed; // 设置3508电机加载速度
//				launcher->motors.motor3508R->ctrl->angle_ctrl_outer->out_max = launcher->config.motor3508_firstmaxspeed; // 设置3508电机加载速度
//			}
//		}
//		else
//		{
//			launcher->motors.motor3508L->ctrl->angle_ctrl_outer->out_max = launcher->config.motor3508_othermaxspeed; // 设置3508电机加载速度
//			launcher->motors.motor3508R->ctrl->angle_ctrl_outer->out_max = launcher->config.motor3508_othermaxspeed; // 设置3508电机加载速度
//		}
//		launcher->fireServo_info.shoot_servo_target = LAUNCHER_SERVO_RELEASE;
//		launcher->motor3508_info.target_state = LAUNCHER_3508_DOWN; // 设置3508目标位置在下面

//		load_start_tick = HAL_GetTick(); // 记录加载开始时间
//		load_ok_tick = 0;				 // 清空完成时间
//		launcher->task_info.launcher_load_state++;
//		break;
//	case LAUNCHER_WAIT_3508_TO_DOWN:								   // 1
//		if (launcher->motor3508_info.state == LAUNCHER_SUBMODULE_DONE) // 3508电机到达加载位置
//		{
//			launcher->fireServo_info.shoot_servo_target = LAUNCHER_SERVO_LOCK; // 设置发射舵机锁定
//			shoot_servo_start_to_work_tick = HAL_GetTick();
//			launcher->task_info.launcher_load_state++;					// 进入下一个加载步骤
//		}

//		break;
//	case LAUNCHER_WAIT_SHOOT_SERVO_TO_LOCK: // 2
//		if (HAL_GetTick() - shoot_servo_start_to_work_tick >= wait_servo_work_tick)
//		{
//			launcher->motor3508_info.target_state = LAUNCHER_3508_UP;
//			launcher->task_info.launcher_load_state++;
//		}
//		break;
//	case LAUNCHER_WAIT_6020_TO_LOAD: // 3
//		if (launcher->motor3508_info.state == LAUNCHER_SUBMODULE_DONE)// 后续可能将这里改成微动开关标志位为1
//		{
//			launcher->motor6020_info.target_state = load_target; // 设置供弹6020电机目标状态
//			launcher->task_info.launcher_load_state++;
//		}
//		break;
//	case LAUNCHER_WAIT_3508_TO_UP_FEEDING_SERVO_TO_TARGET:			   // 4
//		if (launcher->motor6020_info.state == LAUNCHER_SUBMODULE_DONE)
//		{
//			switch (target_id)
//			{
//			case 2:
//				launcher->feedServo_info.second_servo1_target = LAUNCHER_SERVO_RELEASE; // 第二个舵机释放
//				feed_servo_start_to_work_tick = HAL_GetTick();
//				break;
//			case 3:
//				launcher->feedServo_info.third_servo1_target = LAUNCHER_SERVO_RELEASE; // 第三个舵机释放
//				feed_servo_start_to_work_tick = HAL_GetTick();
//				break;
//			case 4:
//				launcher->feedServo_info.fourth_servo1_target = LAUNCHER_SERVO_RELEASE; // 第四个舵机释放
//				feed_servo_start_to_work_tick = HAL_GetTick();
//				break;
//			default:
//				break;
//			}
//		}
//		launcher->task_info.launcher_load_state++;
//		break;

//		// 电磁铁断电
//	case LAUNCHER_SET_ELECTROMAGNET_OFF:										   // 5
//		if (HAL_GetTick() - feed_servo_start_to_work_tick >= wait_servo_work_tick) // 供弹舵机到位
//		{
//			switch (target_id)
//			{
//			case 2:
//				launcher->electromagnet_info.second_electromagnet_target = LAUNCHER_ELECTROMAGNET_OFF;
//				electromagnet_start_to_work_tick = HAL_GetTick();
//				break;
//			case 3:
//				launcher->electromagnet_info.third_electromagnet_target = LAUNCHER_ELECTROMAGNET_OFF;
//				electromagnet_start_to_work_tick = HAL_GetTick();
//				break;
//			case 4:
//				launcher->electromagnet_info.fourth_electromagnet_target = LAUNCHER_ELECTROMAGNET_OFF;
//				electromagnet_start_to_work_tick = HAL_GetTick();
//				break;
//			default:
//				break;
//			}
//			launcher->task_info.launcher_load_state++;
//		}
//		break;
//	case LAUNCHER_SET_FEEDING_SERTVO_TO_RELEASE:
//		if (HAL_GetTick() - electromagnet_start_to_work_tick >= wait_electromagnet_work_tick)
//		{
//			switch (target_id)
//			{
//			case 2:
//				launcher->feedServo_info.second_servo1_target = LAUNCHER_SERVO_LOCK; // 连杆舵机收回
//																					 //				launcher->feedServo_info.second_servo2_target = LAUNCHER_SERVO_LOCK;  	// 第二个舵机收回
//				feed_servo_start_to_work_tick = HAL_GetTick();
//				launcher->electromagnet_info.second_electromagnet_target = LAUNCHER_ELECTROMAGNET_OFF;
//				break;
//			case 3:
//				launcher->feedServo_info.third_servo1_target = LAUNCHER_SERVO_LOCK; // 连杆舵机收回
//																					//				launcher->feedServo_info.third_servo2_target = LAUNCHER_SERVO_LOCK; 		// 第二个舵机收回
//				feed_servo_start_to_work_tick = HAL_GetTick();
//				launcher->electromagnet_info.third_electromagnet_target = LAUNCHER_ELECTROMAGNET_OFF;
//				break;
//			case 4:
//				launcher->feedServo_info.fourth_servo1_target = LAUNCHER_SERVO_LOCK; // 连杆舵机收回
//																					 //				launcher->feedServo_info.fourth_servo2_target = LAUNCHER_SERVO_LOCK; 	// 第二个舵机收回
//				feed_servo_start_to_work_tick = HAL_GetTick();
//				launcher->electromagnet_info.fourth_electromagnet_target = LAUNCHER_ELECTROMAGNET_OFF;
//				break;
//			default:
//				break;
//			}
//			launcher->task_info.launcher_load_state++;
//		}
//		break;
//	case LAUNCHER_WAIT_FEEDING_SERVO_TO_RELEASE:								   // 6
//		if (HAL_GetTick() - feed_servo_start_to_work_tick >= wait_servo_work_tick) // 供弹舵机成功收回
//		{
//			launcher->motor6020_info.target_state = standby_target; // 设置6020到下一个准备目标
//			launcher->task_info.launcher_load_state++;
//		}
//		break;
//	case LAUNCHER_WAIT_6020_TO_STANDBY:								   // 7
//		if (launcher->motor6020_info.state == LAUNCHER_SUBMODULE_DONE) // 6020电机到达准备位置
//		{
//			launcher->task_info.launcher_load_state++;
//		}
//		break;
//	case LAUNCHER_LOAD_OK: // 8
//		if (launcher->task_info.launcher_task_flag.Load_Flag == 1)
//		{
//			launcher->task_info.launcher_task_flag.Load_Flag = 0;			   // 清除装弹标志
//		}
//		if (load_ok_tick == 0)
//		{
//			load_ok_tick = HAL_GetTick();
//		}
//		break;
//	default:
//		break;
//	}
//}

void Launchert_Load_Dart_Task(launcher_t *launcher, launcher_6020_state_e load_target,
							  launcher_6020_state_e standby_target, uint8_t target_id)
{
	static uint32_t load_start_tick = 0;
	static uint32_t load_ok_tick = 0;
	static uint32_t shoot_servo_start_to_work_tick = 0;
	static uint32_t feed_servo_start_to_work_tick = 0;
	static uint32_t electromagnet_start_to_work_tick = 0;
	uint16_t wait_servo_work_tick = 800;
	uint16_t wait_electromagnet_work_tick = 100;
	switch (launcher->task_info.launcher_load_state)
	{
	case LAUNCHER_LOAD_START: // 0
		if (launcher->motor6020_info.state == LAUNCHER_SUBMODULE_MOVING ||
			launcher->motor6020_info.target_state == LAUNCHER_6020_SECOND_LOAD ||
			launcher->motor6020_info.target_state == LAUNCHER_6020_THIRD_LOAD ||
			launcher->motor6020_info.target_state == LAUNCHER_6020_FOURTH_LOAD)
		{
			break; // 供弹6020正在切换到装弹位时，不重复触发加载
		}
		if (target_id == 1)
		{
			if (car.car_ctrl_mode == CAR_AT_MODE)
			{
				launcher->motors.motor3508L->ctrl->angle_ctrl_outer->out_max = launcher->config.motor3508_firstmaxspeed; // 设置3508电机加载速度
				launcher->motors.motor3508R->ctrl->angle_ctrl_outer->out_max = launcher->config.motor3508_firstmaxspeed; // 设置3508电机加载速度
			}
			else
			{
				launcher->motors.motor3508L->ctrl->angle_ctrl_outer->out_max = launcher->config.motor3508_firstmaxspeed; // 设置3508电机加载速度
				launcher->motors.motor3508R->ctrl->angle_ctrl_outer->out_max = launcher->config.motor3508_firstmaxspeed; // 设置3508电机加载速度
			}
		}
		else
		{
			launcher->motors.motor3508L->ctrl->angle_ctrl_outer->out_max = launcher->config.motor3508_othermaxspeed; // 设置3508电机加载速度
			launcher->motors.motor3508R->ctrl->angle_ctrl_outer->out_max = launcher->config.motor3508_othermaxspeed; // 设置3508电机加载速度
		}

		launcher->fireServo_info.shoot_servo_target = LAUNCHER_SERVO_RELEASE;
		launcher->motor6020_info.target_state = load_target; // 设置供弹6020电机目标状态为第一个目标

		load_start_tick = HAL_GetTick(); // 记录加载开始时间
		load_ok_tick = 0;				 // 清空完成时间
		launcher->task_info.launcher_load_state++;
		break;
	case LAUNCHER_WAIT_6020_TO_LOAD:								   // 1
		if (launcher->motor6020_info.state == LAUNCHER_SUBMODULE_DONE) // 供弹6020电机到达加载位置
		{
			if (target_id == 1)
			{
				launcher->feedServo_info.second_servo1_target = LAUNCHER_SERVO_SHOOT_TARGET;
				launcher->feedServo_info.fourth_servo1_target = LAUNCHER_SERVO_SHOOT_TARGET;
			}
			launcher->motor3508_info.target_state = LAUNCHER_3508_MIDDLE; // 设置3508目标位置在下面
			launcher->task_info.launcher_load_state++;
		}

		break;
	case LAUNCHER_SET_3508_TO_DOWN:
		if (launcher->motor3508_info.state == LAUNCHER_SUBMODULE_DONE)
		{
			launcher->motor3508_info.target_state = LAUNCHER_3508_DOWN; // 设置3508目标位置在下面
			launcher->motors.motor3508L->ctrl->angle_ctrl_outer->out_max = launcher->config.motor3508_to_middle_maxspeed;
			launcher->motors.motor3508R->ctrl->angle_ctrl_outer->out_max = launcher->config.motor3508_to_middle_maxspeed;
			launcher->task_info.launcher_load_state++;
		}
		break;
	case LAUNCHER_WAIT_3508_TO_DOWN: // 2
		if (launcher->motor3508_info.state == LAUNCHER_SUBMODULE_DONE)
		{
			if (launcher->task_info.launcher_task_flag.Load_Flag == 1)
			{
				launcher->task_info.launcher_task_flag.Load_Flag = 0;			   // 清除装弹标志
				launcher->fireServo_info.shoot_servo_target = LAUNCHER_SERVO_LOCK; // 设置发射舵机锁定
				shoot_servo_start_to_work_tick = HAL_GetTick();
				launcher->task_info.launcher_load_state++; // 进入下一个加载步骤
			}
		}
		break;
	case LAUNCHER_WAIT_SHOOT_SERVO_TO_LOCK: // 3
		if (HAL_GetTick() - shoot_servo_start_to_work_tick >= wait_servo_work_tick)
		{
			if (target_id == 1)
			{
				launcher->motor3508_info.target_state = LAUNCHER_3508_MIDDLE;
				launcher->motors.motor3508L->ctrl->angle_ctrl_outer->out_max = launcher->config.motor3508_to_middle_maxspeed;
				launcher->motors.motor3508R->ctrl->angle_ctrl_outer->out_max = launcher->config.motor3508_to_middle_maxspeed;
				launcher->task_info.launcher_load_state++;
			}
			else
			{
				launcher->motor3508_info.target_state = LAUNCHER_3508_UP;
				launcher->motors.motor3508L->ctrl->angle_ctrl_outer->out_max = launcher->config.motor3508_othermaxspeed;
				launcher->motors.motor3508R->ctrl->angle_ctrl_outer->out_max = launcher->config.motor3508_othermaxspeed;

				switch (target_id)
				{
				case 2:
					launcher->feedServo_info.second_servo1_target = LAUNCHER_SERVO_RELEASE; // 第二个舵机释放
					feed_servo_start_to_work_tick = HAL_GetTick();
					break;
				case 3:
					launcher->feedServo_info.third_servo1_target = LAUNCHER_SERVO_RELEASE; // 第三个舵机释放
					feed_servo_start_to_work_tick = HAL_GetTick();
					break;
				case 4:
					launcher->feedServo_info.fourth_servo1_target = LAUNCHER_SERVO_RELEASE; // 第四个舵机释放
					feed_servo_start_to_work_tick = HAL_GetTick();
					break;
				default:
					break;
				}
				launcher->task_info.launcher_load_state++;
			}
		}
		break;
	case LAUNCHER_WAIT_3508_TO_MIDDLE:
		if (target_id == 1)
		{
			if (launcher->motor3508_info.state == LAUNCHER_SUBMODULE_DONE)
			{
				launcher->motor3508_info.target_state = LAUNCHER_3508_UP;
				launcher->motors.motor3508L->ctrl->angle_ctrl_outer->out_max = launcher->config.motor3508_firstmaxspeed;
				launcher->motors.motor3508R->ctrl->angle_ctrl_outer->out_max = launcher->config.motor3508_firstmaxspeed;
				launcher->task_info.launcher_load_state++;
			}
		}
		else
		{
			launcher->motors.motor3508L->ctrl->angle_ctrl_outer->out_max = launcher->config.motor3508_othermaxspeed;
			if (HAL_GetTick() - feed_servo_start_to_work_tick >= wait_servo_work_tick)
			{
				launcher->task_info.launcher_load_state++;
			}
		}
		break;
	case LAUNCHER_WAIT_3508_TO_UP_FEEDING_SERVO_TO_TARGET:						   // 4
		if (HAL_GetTick() - feed_servo_start_to_work_tick >= wait_servo_work_tick) // 供弹舵机到位
		{
			switch (target_id)
			{
			case 2:
				launcher->electromagnet_info.second_electromagnet_target = LAUNCHER_ELECTROMAGNET_OFF;
				electromagnet_start_to_work_tick = HAL_GetTick();
				break;
			case 3:
				launcher->electromagnet_info.third_electromagnet_target = LAUNCHER_ELECTROMAGNET_OFF;
				electromagnet_start_to_work_tick = HAL_GetTick();
				break;
			case 4:
				launcher->electromagnet_info.fourth_electromagnet_target = LAUNCHER_ELECTROMAGNET_OFF;
				electromagnet_start_to_work_tick = HAL_GetTick();
				break;
			default:
				break;
			}
			launcher->task_info.launcher_load_state++;
		}
		break;
		// 电磁铁断电
	case LAUNCHER_SET_ELECTROMAGNET_OFF: // 5
		launcher->task_info.launcher_load_state++;
		break;
	case LAUNCHER_SET_FEEDING_SERTVO_TO_RELEASE:
		if (HAL_GetTick() - electromagnet_start_to_work_tick >= wait_electromagnet_work_tick)
		{
			switch (target_id)
			{
			case 2:
				launcher->feedServo_info.second_servo1_target = LAUNCHER_SERVO_LOCK; // 连杆舵机收回
																					 //				launcher->feedServo_info.second_servo2_target = LAUNCHER_SERVO_LOCK;  	// 第二个舵机收回
				feed_servo_start_to_work_tick = HAL_GetTick();
				launcher->electromagnet_info.second_electromagnet_target = LAUNCHER_ELECTROMAGNET_OFF;
				break;
			case 3:
				launcher->feedServo_info.third_servo1_target = LAUNCHER_SERVO_LOCK; // 连杆舵机收回
																					//				launcher->feedServo_info.third_servo2_target = LAUNCHER_SERVO_LOCK; 		// 第二个舵机收回
				feed_servo_start_to_work_tick = HAL_GetTick();
				launcher->electromagnet_info.third_electromagnet_target = LAUNCHER_ELECTROMAGNET_OFF;
				break;
			case 4:
				launcher->feedServo_info.fourth_servo1_target = LAUNCHER_SERVO_LOCK; // 连杆舵机收回
																					 //				launcher->feedServo_info.fourth_servo2_target = LAUNCHER_SERVO_LOCK; 	// 第二个舵机收回
				feed_servo_start_to_work_tick = HAL_GetTick();
				launcher->electromagnet_info.fourth_electromagnet_target = LAUNCHER_ELECTROMAGNET_OFF;
				break;
			default:
				break;
			}
			launcher->task_info.launcher_load_state++;
		}
		break;
	case LAUNCHER_WAIT_FEEDING_SERVO_TO_RELEASE:
		if (HAL_GetTick() - feed_servo_start_to_work_tick >= wait_servo_work_tick) // 供弹舵机成功收回
		{
			launcher->motor6020_info.target_state = standby_target; // 设置6020到下一个准备目标
			launcher->task_info.launcher_load_state++;
		}
		break;
	case LAUNCHER_WAIT_6020_TO_STANDBY:																								// 7
		if (launcher->motor6020_info.state == LAUNCHER_SUBMODULE_DONE && launcher->motor3508_info.state == LAUNCHER_SUBMODULE_DONE) // 6020电机到达准备位置
		{
			switch (car.target_id)
			{
			case 1:
				launcher->feedServo_info.second_servo1_target = LAUNCHER_SERVO_SHOOT_TARGET;
				launcher->feedServo_info.fourth_servo1_target = LAUNCHER_SERVO_SHOOT_TARGET;
				break;
			case 2:
				launcher->feedServo_info.second_servo1_target = LAUNCHER_SERVO_SHOOT_TARGET;
				launcher->feedServo_info.third_servo1_target = LAUNCHER_SERVO_SHOOT_TARGET;
				break;
			case 3:
				launcher->feedServo_info.third_servo1_target = LAUNCHER_SERVO_SHOOT_TARGET;
				launcher->feedServo_info.fourth_servo1_target = LAUNCHER_SERVO_SHOOT_TARGET;
				break;
			case 4:

				launcher->feedServo_info.fourth_servo1_target = LAUNCHER_SERVO_SHOOT_TARGET;
				launcher->feedServo_info.second_servo1_target = LAUNCHER_SERVO_SHOOT_TARGET;
				break;
			default:
				break;
			}
			feed_servo_start_to_work_tick = HAL_GetTick();
			launcher->task_info.launcher_load_state++;
		}
		break;
	case LAUNCHER_WAIT_SERVO_TO_TARGET:
		if (HAL_GetTick() - feed_servo_start_to_work_tick >= 300)
		{
			launcher->task_info.launcher_load_state++;
		}
		break;
	case LAUNCHER_LOAD_OK: // 8
		if (load_ok_tick == 0)
		{
			load_ok_tick = HAL_GetTick();
		}
		break;
	default:
		break;
	}
}

/**
 * @brief 发射器发射任务
 *
 * @param launcher
 */
void Launcher_Shoot_Dart_Task(launcher_t *launcher)
{
	uint32_t wait_shoot_servo_work_tick = 300; // 发射舵机工作等待时间
	static uint32_t shoot_servo_work_tick = 0;
	switch (launcher->task_info.launcher_shoot_state)
	{
	case LAUNCHER_SHOOT_START:
		if(car.target_id == 2)
		{
			vTaskDelay(500);
		}
		launcher->task_info.launcher_shoot_state++;
		break;
	case LAUNCHER_SET_SHOOT_SERVO_RELEASE:
		launcher->fireServo_info.shoot_servo_target = LAUNCHER_SERVO_RELEASE;
		shoot_servo_work_tick = HAL_GetTick();
		launcher->task_info.launcher_shoot_state++;

		break;
	case LAUNCHER_WAIT_SHOOT_SERVO_TO_RELEASE:
		if (HAL_GetTick() - shoot_servo_work_tick > wait_shoot_servo_work_tick)
		{
			vTaskDelay(800);
			shoot_servo_work_tick = HAL_GetTick();
			launcher->task_info.launcher_shoot_state++;
		}
		break;
	case LAUNCHER_FEEDING_SERVO_BACK:
		if (HAL_GetTick() - shoot_servo_work_tick > wait_shoot_servo_work_tick)
		{
			launcher->feedServo_info.second_servo1_target = LAUNCHER_SERVO_LOCK;
			launcher->feedServo_info.third_servo1_target = LAUNCHER_SERVO_LOCK;
			launcher->feedServo_info.fourth_servo1_target = LAUNCHER_SERVO_LOCK;
			launcher->task_info.launcher_shoot_state++;
		}
		break;
	case LAUNCHER_SHOOT_OK:
		break;
	default:
		break;
	}
}

void Launcher_Protection_Check(launcher_t *launcher)
{
	if (launcher->motor3508_info.target_state == LAUNCHER_3508_UNFORCE)
	{
		if (launcher->motor3508_info.motor3508L_position >
				launcher->config.motor3508_protect_position &&
			abs(launcher->motors.motor3508L->rx_info->speed) > 100)
		{
			while (1)
			{
				// 扳机上
				launcher->fireServo_info.shoot_servo_target = LAUNCHER_SERVO_LOCK;
				// 其他电机卸力
				launcher->motor6020_info.target_angle = Launcher_6020_Init_Angle_Cal(launcher); // 供弹6020电机目标角度
				// 锁住3508
				launcher->motor3508_info.ctrl_mode = MOTOR_PID_POSITION;
				launcher->motor3508_info.motor3508L_target_position =
					launcher->motor3508_info.motor3508L_position - launcher->motor3508_info.motor3508L_init_angle_sum;

				if (car.car_ctrl_mode <= CAR_INIT_MODE)
				{
					//					Buzzer_Stop();
					break;
				}
				//				Buzzer_Error_Indicate();
			}
		}
	}
}

/**
 * @brief 任务管理器
 *
 * @param pvParameters
 */
void Launcher_TaskManager(void *pvParameters)
{
	uint8_t target_id = car.target_id;

	if (launcher.state < LAUNCHER_FREE) // 未初始化
	{
		xLauncherTaskHandle = NULL;
		launcher.task_info.LauncherTaskState = eDeleted; // 任务状态设置为已删除
		vTaskDelete(NULL);
	}

	static launcher_6020_state_e load_target;
	static launcher_6020_state_e standby_target;

	switch (target_id) // 判断6020状态
	{
	case 1:
		load_target = LAUNCHER_6020_SECOND_STANDBY;
		standby_target = LAUNCHER_6020_SECOND_STANDBY;
		break;
	case 2:
		load_target = LAUNCHER_6020_SECOND_LOAD;
		standby_target = LAUNCHER_6020_THIRD_STANDBY;
		break;
	case 3:
		load_target = LAUNCHER_6020_THIRD_LOAD;
		standby_target = LAUNCHER_6020_FOURTH_STANDBY;
		break;
	case 4:
		load_target = LAUNCHER_6020_FOURTH_LOAD;
		standby_target = LAUNCHER_6020_SECOND_STANDBY;
		break;
	default:
		xLauncherTaskHandle = NULL;
		launcher.task_info.LauncherTaskState = eDeleted; // 任务状态设置为已删除，取消任务。
		vTaskDelete(NULL);
		break;
	}

	for (;;)
	{
		//		Launcher_Protection_Check(&launcher); // 调用保护函数

		switch (launcher.state)
		{
		case LAUNCHER_FREE:
			launcher.task_info.launcher_load_state = LAUNCHER_LOAD_START;	// 重置加载步骤
			launcher.task_info.launcher_shoot_state = LAUNCHER_SHOOT_START; // 重置发射步骤
			launcher.state = LAUNCHER_LOADING;								// 进入加载步骤
			break;
		case LAUNCHER_LOADING:
			Launchert_Load_Dart_Task(&launcher,
									 load_target, standby_target, target_id);

			if (launcher.task_info.launcher_load_state == LAUNCHER_LOAD_OK)
			{
				launcher.state = LAUNCHER_LOADED; // 加载完成
			}
			break;
		case LAUNCHER_LOADED:
			if (launcher.task_info.launcher_task_flag.Shoot_Flag == 1)
			{
				launcher.task_info.launcher_task_flag.Shoot_Flag = 0; // 清除发射信号
				launcher.state = LAUNCHER_SHOOTING;					  // 进入发射步骤
			}
			//		    else
			//			{
			//				return;
			//			}
			break;
		case LAUNCHER_SHOOTING:
			Launcher_Shoot_Dart_Task(&launcher);
			if (launcher.task_info.launcher_shoot_state == LAUNCHER_SHOOT_OK)
			{
				launcher.state = LAUNCHER_FREE; // 发射完成
				xLauncherTaskHandle = NULL;
				launcher.task_info.LauncherTaskState = eDeleted; // 任务状态设置为已删除
				vTaskDelete(NULL);
			}
			break;
		default:
			break;
		}
		if (car.car_ctrl_mode < CAR_MT_MODE)
		{
			xLauncherTaskHandle = NULL;
			launcher.task_info.LauncherTaskState = eDeleted; // 任务状态设置为已删除
			vTaskDelete(NULL);
		}
		Launcher_Motor3508_State_Controler(&launcher); // 3508电机状态控制
		Launcher_Motor6020_State_Controler(&launcher); // 供弹6020电机状态控制
	}
}

HAL_StatusTypeDef Launcher_Task_Start(void)
{

	// 检查任务状态
	if (xLauncherTaskHandle != NULL)
	{
		// 任务还在
		return HAL_BUSY;
	}
	// 任务已删除
	if (xTaskCreate(Launcher_TaskManager,
					"LauncherTask", 2048,
					NULL, tskIDLE_PRIORITY + 2,
					&xLauncherTaskHandle) != pdPASS)
	{
		return HAL_ERROR;
	}
	else
	{
		launcher.task_info.LauncherTaskState = eRunning;
		return HAL_OK;
	}
}

/**
 * @brief 发射总控
 *
 * @param launcher
 */
void Launcher_Work(launcher_t *launcher)
{

	Launcher_Motor_Position_Cal(launcher); // 发射器电机位置计算

	if (car.car_ctrl_mode != CAR_SLEEP_MODE) // 开控
	{
		Launcher_6020_Pid_Calculating(launcher);
		Launcher_3508_Pid_Calculating(launcher);
		//		launcher->motors.motor3508L->base_info.motor_out = launcher->motor3508_info.motor3508L_output_current;
		//		launcher->motors.motor3508R->base_info.motor_out = launcher->motor3508_info.motor3508R_output_current;
	}
	else
	{
		Launcher_Reset(launcher); // 如果没有开控，则重置发射器
	}
	mircoswitch_check();
	Launcher_Servo_Controler(launcher);
	Launcher_Electromagnet_Controler(launcher);
}
