#include "car.h"
#include "imu_sensor.h"
#include "launcher_motor.h"
#include "launcher.h"
#include "force.h"
#include "axis.h"
#include "rc_sensor.h"
#include "buzzer.h"
#include "tuner.h"
#include "vision_protocol.h"

void Car_Ctrl_Mode_Update(car_t *car);
void Car_Rc_Wheel_Filter(car_t *car);
void Car_Work_MT(car_t *car);
void Car_Work_AMT(car_t *car);
void Car_Work_Init(car_t *car);
void Car_Work_AT(car_t *car);
void Car_Task_Status_Update(car_t *car);
void Car_External_Info_Update(car_t *car);
void Car_Reassign_Task(car_t *car);
void rebuild_check(car_t* car);

static TaskHandle_t xFourShotStaticTaskHandle;	  // 轴不动连发
static TaskHandle_t xSingleShotStaticTaskHandle;  // 单发静态
static TaskHandle_t xFourShotDynamicTaskHandle;	  // 轴动连发
static TaskHandle_t xSingleShotDynamicTaskHandle; // 单发动态任务

static car_ctrl_mode_e last_car_ctrl_mode;

static uint8_t is_in_AMT_setMode = 0; // 是否在设置中

static uint8_t single_shoot_static_start_id = 1; // 单发静态开始id
static uint8_t four_shot_static_start_id = 1;	 // 四连发静态开始id
static uint8_t four_shot_dynamic_start_id = 1;	 // 四连发动态开始id

static uint8_t shoot_start_id = 1; // 发射开始id

int32_t Test_err;
bool stick_flag = false;
uint8_t set_flag = 0;
car_t car =
	{
		.car_ctrl_mode = CAR_AMT_MODE, // CAR_MT_MODE,
		//	.wheel_status = CAR_WHEEL_KEEP,			// 波轮状态
		.target_id = 1, // 目标id
		.task_info = {
			.FourShotStatic_TaskState = eDeleted,
			.FourShotDynamic_TaskState = eDeleted,
			.SingleShotDynamic_TaskState = eDeleted,
			.SingleShotStatic_TaskState = eDeleted,
			.first_widows_total_shoots = 4,
		},
};

/**
 * @brief 控制任务 实时任务中调用
 *
 * @param car
 */
void Car_Ctrl(car_t *car)
{
	Car_Rc_Wheel_Filter(car);
	//	Car_Rc_Wheel_Filter(car); // 检测波轮状态
	Car_Task_Status_Update(car);
	/*控制模式更新*/
	Car_Ctrl_Mode_Update(car);
	Car_External_Info_Update(car);
	switch (car->car_ctrl_mode)
	{
	case CAR_SLEEP_MODE:
		//		Indicator_Task_Stop(); // 停止指示器任务（开机提示音）
		car->target_id = 1; // 重置目标id
		break;
	case CAR_INIT_MODE:
		Car_Work_Init(car);
		break;
	case CAR_MT_MODE:
		Car_Work_MT(car);
		break;
	case CAR_AMT_MODE:
		Car_Work_AMT(car);
		break;
	case CAR_AT_MODE:
		Car_Work_AT(car);
		break;
	default:
		break;
	}

	last_car_ctrl_mode = car->car_ctrl_mode; // 记录上次控制模式
}

/**
 * @brief 检测上升沿   //rc_sensor里面的跳变检测太快了，会存在识别不到的现象
 *
 * @param car
 */
void Car_Rc_Wheel_Filter(car_t *car)
{
	static uint16_t thumbwheel_keep_down_cnt = 0;
	static int16_t thumbwheel_down_cnt = 0;
	static int16_t thumbwheel_up_cnt = 0;
	int16_t thumbwheel_value = rc_sensor.info->thumbwheel.value;

	if (thumbwheel_value > 500)
	{
		thumbwheel_keep_down_cnt++;
	}
	else
	{
		thumbwheel_keep_down_cnt = 0;
		thumbwheel_down_cnt = 0;
	}
	if (thumbwheel_value > -500)
	{
		thumbwheel_up_cnt = 0;
	}

	switch (car->wheel_status)
	{
	case CAR_WHEEL_KEEP:
		if (thumbwheel_value > 500 && thumbwheel_down_cnt >= 0)
		{
			thumbwheel_down_cnt++;
		}
		if (thumbwheel_value < -500 && thumbwheel_up_cnt >= 0)
		{
			thumbwheel_up_cnt++;
		}
		if (thumbwheel_down_cnt > 50)
		{
			car->wheel_status = CAR_WHEEL_DOWN;
			thumbwheel_down_cnt = -1;
		}
		if (thumbwheel_up_cnt > 50)
		{
			car->wheel_status = CAR_WHEEL_UP;
			thumbwheel_up_cnt = -1; // 重置波轮向上计数
		}
		break;
	case CAR_WHEEL_UP:
	case CAR_WHEEL_DOWN:
		car->wheel_status = CAR_WHEEL_KEEP;
		break;
	default:
		break;
	}

	if (thumbwheel_keep_down_cnt > 1000)
	{
		car->wheel_status = CAR_WHEEL_HOLD_DOWN;
	}
	else if (car->wheel_status == CAR_WHEEL_HOLD_DOWN)
	{
		car->wheel_status = CAR_WHEEL_KEEP;
	}
}

/**
 * @brief 任务状态更新
 *
 * @param car
 */
void Car_Task_Status_Update(car_t *car)
{
	if (xFourShotStaticTaskHandle != NULL)
	{
		car->task_info.FourShotStatic_TaskState = eRunning;
	}
	else
	{
		car->task_info.FourShotStatic_TaskState = eDeleted;
	}

	if (xFourShotDynamicTaskHandle != NULL)
	{
		car->task_info.FourShotDynamic_TaskState = eRunning;
	}
	else
	{
		car->task_info.FourShotDynamic_TaskState = eDeleted;
	}

	if (xSingleShotDynamicTaskHandle != NULL)
	{
		car->task_info.SingleShotDynamic_TaskState = eRunning;
	}
	else
	{
		car->task_info.SingleShotDynamic_TaskState = eDeleted;
	}

	if (xSingleShotStaticTaskHandle != NULL)
	{
		car->task_info.SingleShotStatic_TaskState = eRunning;
	}
	else
	{
		car->task_info.SingleShotStatic_TaskState = eDeleted;
	}

	if (xFourShotDynamicTaskHandle == NULL &&
		xFourShotStaticTaskHandle == NULL &&
		xSingleShotDynamicTaskHandle == NULL &&
		xSingleShotStaticTaskHandle == NULL)
	{
		car->task_info.TotalTaskState = eDeleted; // 所有任务都已删除
	}
	else
	{
		car->task_info.TotalTaskState = eRunning; // 有任务在运行
	}
}

void Car_External_Info_Update(car_t *car)
{
	dart_launch_opening_status_e dart_launch_opening_status = (dart_launch_opening_status_e)judge.dart_client_cmd.dart_launch_opening_status;
	switch (car->judge_info.launch_opening_status)
	{
	case CAR_LAUNCH_ON:
		if (dart_launch_opening_status == DART_LAUNCH_OFF)
		{
			car->judge_info.launch_opening_status = CAR_LAUNCH_OFF;
		}
		else if (dart_launch_opening_status == DART_LAUNCH_MOVING)
		{
			car->judge_info.launch_opening_status = CAR_LAUNCH_CLOSING;
		}
		car->judge_info.my_dart_open_time = (HAL_GetTick() - car->judge_info.launch_open_tick);
		break;
	case CAR_LAUNCH_OFF:
		car->judge_info.my_dart_open_time = 0;
		if (dart_launch_opening_status == DART_LAUNCH_ON)
		{
			car->judge_info.launch_opening_status = CAR_LAUNCH_ON;
		}
		else if (dart_launch_opening_status == DART_LAUNCH_MOVING)
		{
			car->judge_info.launch_opening_status = CAR_LAUNCH_OPENING;
		}
		break;
	case CAR_LAUNCH_OPENING:
		car->judge_info.my_dart_open_time = 0;
		if (dart_launch_opening_status == DART_LAUNCH_ON)
		{
			car->judge_info.launch_open_tick = HAL_GetTick(); // 记录打开舱门的时间
			car->judge_info.launch_opening_status = CAR_LAUNCH_ON;
		}
		else if (dart_launch_opening_status == DART_LAUNCH_OFF)
		{
			car->judge_info.launch_opening_status = CAR_LAUNCH_OFF;
		}
		break;
	case CAR_LAUNCH_CLOSING:
		car->judge_info.my_dart_open_time = 0;
		if (dart_launch_opening_status == DART_LAUNCH_ON)
		{
			car->judge_info.launch_opening_status = CAR_LAUNCH_ON;
		}
		else if (dart_launch_opening_status == DART_LAUNCH_OFF)
		{
			car->judge_info.launch_opening_status = CAR_LAUNCH_OFF;
		}
		break;
	default:
		break;
	}

	car->judge_info.judge_state = judge.judge_state;
	car->judge_info.dart_remaining_time = judge.dart_info.dart_remaining_time;
	car->judge_info.dart_target_type = (dart_target_type_e)judge.dart_info.dart_current_target;
	car->judge_info.game_progress = (game_progress_e)judge.ext_game_status.game_progress;
	if (car->car_ctrl_mode == CAR_AT_MODE)
	{
		if (tuner.tuner_read_flash_info.first_windows_shoots >= 1 &&
			tuner.tuner_read_flash_info.first_windows_shoots <= 4)
		{
			car->task_info.first_widows_total_shoots = tuner.tuner_read_flash_info.first_windows_shoots; // 设置首次窗口总发射次数
		}
		else
		{
			car->task_info.first_widows_total_shoots = 4; // 默认值
		}
	}

	car->module_state.judge_state = judge.judge_state; // 更新模块状态
	car->module_state.motor3508L_state = launcher.motors.motor3508L->state->status;
	car->module_state.motor3508R_state = launcher.motors.motor3508R->state->status;
	car->module_state.motor6020_state = launcher.motors.motor6020->state->status;
	car->module_state.motorAxis_state = axis.axis_motor->state->status;
	car->module_state.motorForce_state = force.force_motor->state->status;
	car->module_state.vision_state = vision.status->rx_state;
//	car->module_state.imu_state = imu_sensor.work_state.dev_state;
	car->module_state.rc_state = rc_sensor.work_state == 0 ? DEV_ONLINE : DEV_OFFLINE;
}

/**
 *@brief 静态单发
 *
 */
void Car_Single_Shot_Static_Task(void *pvParameters)
{
	if (launcher.state != LAUNCHER_FREE)
	{
		xSingleShotStaticTaskHandle = NULL;
		vTaskDelete(NULL);
	}
	launcher.task_info.launcher_task_flag.Load_Flag = 1;  // 设置加载信号
	launcher.task_info.launcher_task_flag.Shoot_Flag = 0; // 清除发射信号
	Launcher_Task_Start();								  // 启动发射器任务
	for (;;)
	{
		if (car.car_ctrl_mode < CAR_MT_MODE)
		{
			xSingleShotStaticTaskHandle = NULL;
			vTaskDelete(NULL); // 关控删除
		}
		//				if (car.wheel_status == CAR_WHEEL_UP &&
		//					launcher.state == LAUNCHER_LOADED) // 手动发射控制
		//				{
		launcher.task_info.launcher_task_flag.Shoot_Flag = 1;
		//				}
		//		if (launcher.task_info.launcher_load_state == LAUNCHER_LOAD_OK)
		//		{
		//			launcher.task_info.launcher_task_flag.Shoot_Flag = 1;
		//		}
		if (launcher.task_info.LauncherTaskState == eDeleted)
		{
			break; // 发射完成退出
		}
		vTaskDelay(pdMS_TO_TICKS(1));
	}
	xSingleShotStaticTaskHandle = NULL;
	vTaskDelete(NULL);
}

/**
 * @brief 静态连发
 *
 * @param pvParameters
 */
void Car_Four_Shot_Static_Task(void *pvParameters)
{
	if (launcher.state != LAUNCHER_FREE)
	{
		xFourShotStaticTaskHandle = NULL;
		vTaskDelete(NULL);
	}
	uint8_t target_total_shoots = 4;
	uint8_t detect_start_flag = 0;
	uint8_t finished_shots = 0;
	Launcher_Task_Start();

	for (;;)
	{
		if (car.car_ctrl_mode < CAR_MT_MODE)
		{
			xFourShotStaticTaskHandle = NULL;
			vTaskDelete(NULL);
		}

		launcher.task_info.launcher_task_flag.Load_Flag = 1; // 设置加载信号
															 //		if (car.wheel_status == CAR_WHEEL_UP &&
															 //			launcher.state == LAUNCHER_LOADED) // 手动发射控制
															 //		{
															 //			launcher.task_info.launcher_task_flag.Shoot_Flag = 1;
															 //		}
															 //		if (launcher.state == LAUNCHER_LOADED)
															 //		{
															 //		launcher.task_info.launcher_task_flag.Shoot_Flag = 1; // 设置发射信号
															 //		}
		if (car.wheel_status == CAR_WHEEL_UP &&
			launcher.state == LAUNCHER_LOADED) // 手动发射控制
		{
			//		if (launcher.state == LAUNCHER_LOADED) // 手动发射控制
			//		{
			launcher.task_info.launcher_task_flag.Shoot_Flag = 1;
		}
		else if (launcher.state != LAUNCHER_LOADED)
		{
			launcher.task_info.launcher_task_flag.Shoot_Flag = 0;
		}

		if (launcher.task_info.LauncherTaskState != eDeleted)
		{
			detect_start_flag = 1;
		}
		if (detect_start_flag == 1 && launcher.task_info.LauncherTaskState == eDeleted)
		{
			finished_shots++;
			car.target_id++; // 目标id加1
			if (car.target_id > 4)
			{
				car.target_id = 1;
			}

			if (finished_shots >= target_total_shoots)
			{
				break;
			}
			detect_start_flag = 0;
			vTaskDelay(1000);
			Launcher_Task_Start();
			vTaskDelay(1000);
		}
		vTaskDelay(pdMS_TO_TICKS(1));
	}
	xFourShotStaticTaskHandle = NULL;
	vTaskDelete(NULL);
}

/**
 * @brief 动态单发任务，外部设置好发射id
 *
 * @param pvParameters
 */
void Car_Single_Shot_Dynamic_Task(void *pvParameters)
{
	if (launcher.state != LAUNCHER_FREE)
	{
		xSingleShotDynamicTaskHandle = NULL;
		vTaskDelete(NULL);
	}
	uint8_t detect_launcherTask_start_flag = 0;
	uint8_t detect_axisTask_start_flag = 0;
	uint8_t detect_forceTask_start_flag = 0;

	if (car.car_ctrl_mode == CAR_AT_MODE)
	{
		//		if (car.judge_info.dart_target_type == DART_TARGET_BASE_RANDOM_FIXED)
		//		{
		//			axis.task_info.axis_task_arg.is_vision_on = 1;
		//		}
		//		else
		//		{
		axis.task_info.axis_task_arg.is_vision_on = 0;
		//		}
	}
	else
	{
		axis.task_info.axis_task_arg.is_vision_on = 0;
	}
	launcher.task_info.launcher_task_flag.Load_Flag = 0;  // 清除加载信号
	launcher.task_info.launcher_task_flag.Shoot_Flag = 0; // 清除发射信号

	Launcher_Task_Start();
	Axis_Task_Start();
	Force_Task_Start();

	for (;;)
	{
		if (car.car_ctrl_mode < CAR_MT_MODE)
		{
			xSingleShotDynamicTaskHandle = NULL;
			vTaskDelete(NULL);
		}

		if (axis.task_info.AxisTaskState != eDeleted)
		{
			detect_axisTask_start_flag = 1;
		}
		if (launcher.task_info.LauncherTaskState != eDeleted)
		{
			detect_launcherTask_start_flag = 1;
		}
		if (force.task_info.ForceTaskState != eDeleted)
		{
			detect_forceTask_start_flag = 1;
		}

		if (detect_axisTask_start_flag == 1 &&
			axis.task_info.AxisTaskState == eDeleted)
		{
			launcher.task_info.launcher_task_flag.Load_Flag = 1; // 设置加载信号
		}

		if (detect_forceTask_start_flag == 1 &&
			force.task_info.ForceTaskState == eDeleted &&
			detect_axisTask_start_flag == 1 &&
			axis.task_info.AxisTaskState == eDeleted)
		{
			launcher.task_info.launcher_task_flag.Shoot_Flag = 1; // 设置发射信号
		}

		if (detect_launcherTask_start_flag == 1 &&
			launcher.task_info.LauncherTaskState == eDeleted)
		{
			break; // 发射完成退出
		}
		vTaskDelay(pdMS_TO_TICKS(1));
	}
	vTaskDelay(pdMS_TO_TICKS(1000));
	axis.motor_info.target_position = 0;  // 轴目标位置归零
	force.motor_info.target_position = 0; // 力矩目标位置归零
	xSingleShotDynamicTaskHandle = NULL;
	vTaskDelete(NULL);
}

/**
 * @brief 动态四连发
 *
 * @param pvParameters
 */
void Car_Four_Shot_Dynamic_Task(void *pvParameters)
{
	static uint32_t axis_move_tick = 0;
	if (car.car_ctrl_mode == CAR_AT_MODE)
	{
		if (car.judge_info.dart_target_type == DART_TARGET_BASE_RANDOM_FIXED)
		{
			axis.task_info.axis_task_arg.is_vision_on = 1;
		}
		else
		{
			axis.task_info.axis_task_arg.is_vision_on = 0;
		}
	}
	else
	{
		axis.task_info.axis_task_arg.is_vision_on = 0;
	}
	launcher.task_info.launcher_task_flag.Load_Flag = 0;  // 清除加载信号
	launcher.task_info.launcher_task_flag.Shoot_Flag = 0; // 清除发射信号

	uint8_t have_given_xLoadSemaphore = 0;						// 是否已经给了加载信号
	uint8_t target_total_shoots = 4;							// 一共要大多少发
	if (car.car_ctrl_mode == CAR_AT_MODE && car.target_id == 1) // 比赛模式
	{
		have_given_xLoadSemaphore = 1;								   // CAR_AT_MODE在外面加载信号
		target_total_shoots = car.task_info.first_widows_total_shoots; // 首次窗口总发射次数
	}
	else
	{
		// 把这里的Axis_Task_Start任务放到后面供弹完成之后进行，这里进行一个yaw轴的左偏供弹
		Axis_Task_Start();
	}

	uint8_t have_given_xShootSemaphore = 0; // 是否已经给了发射

	Launcher_Task_Start();
	Force_Task_Start();
	for (;;)
	{
		if (car.car_ctrl_mode < CAR_MT_MODE)
		{
			xFourShotDynamicTaskHandle = NULL;
			vTaskDelete(NULL);
		}
#ifdef OUTPOST_ALIVE_CHECK
		if(vision.status->rx_state == DEV_ONLINE && car.target_id != 1)
		{
			rebuild_check(&car);
		}
#endif
		// 发射完成则进行下一发
//		if(vision.rx_info->target_status == 0 && car.is_round_two == 0 && car.target_id != 1)
//		{
//			target_total_shoots = car.target_id;
//			car.is_round_two = 1;
//		}
		if (launcher.task_info.LauncherTaskState == eDeleted && axis.motor_info.load_flag == 0)
		{
//			if(vision.rx_info->target_status == 0 && car.is_round_two == 0)
//			{
//				target_total_shoots = car.target_id;
//				car.is_round_two = 1;
//			}
			car.target_id++;
			if (car.target_id > target_total_shoots)
			{
				if (car.target_id > 4)
				{
					car.target_id = 1; // 重置目标id
				}
				if (car.target_id == 4)
				{
					launcher.config.motor6020_angle_ramp_rate = 0; // 供弹6020电机目标角度平滑速率
				}
				else
				{
					launcher.config.motor6020_angle_ramp_rate = 3;
				}
				break; // 大于4重置退出
			}
			if (car.target_id == 2)
			{
				vTaskDelay(pdMS_TO_TICKS(800));
			}
			else
			{
				vTaskDelay(pdMS_TO_TICKS(300));
			}
			// 在供弹前将位置偏到左边进行供弹
			axis_move_tick = HAL_GetTick();
			axis.motor_info.load_flag = 1;
			Force_Task_Start();
		}
		if (axis.motor_info.load_flag == 1)
		{
			if (car.target_id == 2)
			{
				axis.motor_info.target_position = axis.motor_info.load_position;
			}
		}
		if (abs(axis.motor_info.current_position - axis.motor_info.target_position) < 2000 || HAL_GetTick() - axis_move_tick > 2000 || car.target_id != 2)
		{
			Launcher_Task_Start();
			have_given_xLoadSemaphore = 0;
			have_given_xShootSemaphore = 0;
		}
		if (launcher.task_info.launcher_load_state <= LAUNCHER_WAIT_3508_TO_DOWN &&
			axis.task_info.AxisTaskState == eDeleted && have_given_xLoadSemaphore == 0)
		{
			//			if (car.judge_info.judge_state == DEV_ONLINE &&
			//				car.judge_info.launch_opening_status == CAR_LAUNCH_ON &&
			//				car.judge_info.dart_remaining_time <= 1)  //这里的超时退出可以根据一发需要的时间进行调整
			//			{
			//				break; // 不够时间退出
			//			}
			launcher.task_info.launcher_task_flag.Load_Flag = 1;
			have_given_xLoadSemaphore = 1;
		}
		// 檢查轴任務是否完成
		if (launcher.state == LAUNCHER_LOADED)
		{
			axis.motor_info.load_flag = 0;
			Axis_Task_Start();
		}
		if (launcher.state == LAUNCHER_LOADED && axis.task_info.AxisTaskState == eDeleted &&
			force.task_info.ForceTaskState == eDeleted && have_given_xShootSemaphore == 0)
		{
			launcher.task_info.launcher_task_flag.Shoot_Flag = 1;
			have_given_xShootSemaphore = 1;
		}
		vTaskDelay(pdMS_TO_TICKS(1));
	}


	vTaskDelay(pdMS_TO_TICKS(1000));
	axis.motor_info.target_position = 0;  // 轴目标位置归零
	force.motor_info.target_position = 0; // 力矩目标位置归零
	xFourShotDynamicTaskHandle = NULL;
	vTaskDelete(NULL);
}

/**
 * @brief 控制模式更新
 *
 * @param car
 */
void Car_Ctrl_Mode_Update(car_t *car)
{
	/********* car_ctrl_mode setting *********/
	if (!RC_ONLINE)
	{
		car->car_ctrl_mode = CAR_SLEEP_MODE; // 如果遥控器不在线，设置为休眠模式
	}
	else if (launcher.state < LAUNCHER_FREE &&
			 car->car_ctrl_mode != CAR_INIT_MODE)
	{
		if (rc_sensor.info->s1.value == 1)
		{
			car->car_ctrl_mode = CAR_MT_MODE;
		}
		else if (rc_sensor.info->s1.value == 2 ||
				 rc_sensor.info->s1.value == 3)
		{
			car->car_ctrl_mode = CAR_INIT_MODE;
		}
	}

	if (car->task_info.TotalTaskState == eDeleted)
	{
		if (car->car_ctrl_mode > CAR_INIT_MODE && car->car_ctrl_mode != CAR_AT_MODE)
		{
			// 初始化完成
			switch (rc_sensor.info->s1.value)
			{
			case 1: // 左拨杆上
				car->car_ctrl_mode = CAR_MT_MODE;
				break;
			case 3: // 左拨杆中
				car->car_ctrl_mode = CAR_AMT_MODE;
				break;
			case 2: // 左拨杆下
				car->car_ctrl_mode = CAR_AT_MODE;
				break;
			default:
				break;
			}
		}
	}
}

/**
 * @brief 手动波工作函数
 *
 * @param car
 */
void Car_Work_MT(car_t *car)
{
	if (last_car_ctrl_mode != CAR_MT_MODE) // 切换模式处理
	{
		launcher.fireServo_info.shoot_servo_target = LAUNCHER_SERVO_LOCK;
		// 目标速度清零
		launcher.motor3508_info.target_speed = 0;										// 上膛3508电机目标速度清零
		axis.motor_info.target_speed = 0;												// 轴电机目标速度清零
		force.motor_info.target_speed = 0;												// 力矩电机目标速度清零
		launcher.motor6020_info.target_angle = Launcher_6020_Init_Angle_Cal(&launcher); // 计算供弹6020电机初始化角度
		is_in_AMT_setMode = 0;															// 退出设置模式
		Indicator_Task_Stop();															// 停止指示器任务
	}

	// launcher 控制
	launcher.motor6020_info.ctrl_mode = MOTOR_PID_ANGLE;
	//	launcher.motor3508_info.ctrl_mode = MOTOR_PID_SPEED;
	launcher.motor3508_info.ctrl_mode = MOTOR_PID_POSITION;
	//	if(motor_stick_check(launcher.motors.motor3508L) == true)
	//	{
	//		stick_flag = true;
	//		launcher.motor3508_info.motor3508L_init_angle_sum = launcher.motors.motor3508L->rx_info->encoder_sum;
	//		launcher.motor3508_info.motor3508L_target_position = launcher.motor3508_info.motor3508L_init_angle_sum + 20000;
	//	}
	//	if(stick_flag == false)
	//	{
	//		launcher.motor3508_info.ctrl_mode = MOTOR_PID_SPEED;
	//	}
	//	else
	//	{
	//		launcher.motor3508_info.ctrl_mode = MOTOR_PID_POSITION;
	//	}

#if !CAR_DEBUG_MODE_ENABLE
	launcher.motor6020_info.target_angle = Launcher_6020_Init_Angle_Cal(&launcher); // 计算供弹6020电机初始化角度
#endif

	// 舵机控制
	if (rc_sensor.info->s2.value == 1) // 右拨杆上
	{
		// 电机目标速度
		//		launcher.motor3508_info.target_speed = rc_sensor.info->ch1 * 10;
		launcher.motor3508_info.motor3508L_target_position += rc_sensor.info->ch1 * 2;
		//		launcher.motor3508_info.motor3508R_target_position = -launcher.motor3508_info.motor3508L_target_position;
		//		if(launcher.motor3508_info.ctrl_mode == MOTOR_PID_SPEED)
		//		{
		//
		//			if (car->wheel_status == CAR_WHEEL_DOWN)
		//			{
		//				if(set_flag == 0)
		//				{
		//					set_flag = 1;
		//				}
		//				else
		//				{
		//					set_flag = 0;
		//				}
		//				if(set_flag == 1)
		//				{
		//					launcher.motor3508_info.target_speed = -1000;
		//				}
		//				else
		//				{
		//					launcher.motor3508_info.target_speed = 0;
		//				}
		//			}
		//		}
		//		else
		//		{
		//		launcher.motor3508_info.motor3508L_target_position += rc_sensor.info->ch1 * 10;
		//		launcher.motor3508_info.motor3508R_target_position = -launcher.motor3508_info.motor3508L_target_position;
		//		}
		if (car->wheel_status == CAR_WHEEL_UP)
		{
			if (launcher.fireServo_info.shoot_servo_target == LAUNCHER_SERVO_LOCK)
			{
				launcher.fireServo_info.shoot_servo_target = LAUNCHER_SERVO_RELEASE;
			}
			else
			{
				launcher.fireServo_info.shoot_servo_target = LAUNCHER_SERVO_LOCK;
			}
		}
		if (car->wheel_status == CAR_WHEEL_DOWN)
		{
			if (car->reassign_falg == 0)
			{
				car->reassign_falg = 1;
				axis.axis_reassign_task_step = AXIS_REASSIGN_START;
				force.force_reassign_task_step = FORCE_REASSIGN_START;
			}
			else
			{
				car->reassign_falg = 0;
				axis.axis_reassign_task_step = AXIS_REASSIGN_DONE;
				force.force_reassign_task_step = FORCE_REASSIGN_DONE;
			}
		}
	}

	if (car->reassign_falg == 1)
	{
		Car_Reassign_Task(car);
	}

	if (rc_sensor.info->s2.value == 2) // 右拨杆下
	{
		if (car->wheel_status == CAR_WHEEL_UP)
		{
			switch (car->target_id)
			{
			case 2:
				if (launcher.feedServo_info.second_servo1_target == LAUNCHER_SERVO_LOCK)
				{
					launcher.feedServo_info.second_servo1_target = LAUNCHER_SERVO_RELEASE;
				}
				else
				{
					launcher.feedServo_info.second_servo1_target = LAUNCHER_SERVO_LOCK;
				}
				break;
			case 3:
				if (launcher.feedServo_info.third_servo1_target == LAUNCHER_SERVO_LOCK)
				{
					launcher.feedServo_info.third_servo1_target = LAUNCHER_SERVO_RELEASE;
				}
				else
				{
					launcher.feedServo_info.third_servo1_target = LAUNCHER_SERVO_LOCK;
				}
				break;
			case 4:
				if (launcher.feedServo_info.fourth_servo1_target == LAUNCHER_SERVO_LOCK)
				{
					launcher.feedServo_info.fourth_servo1_target = LAUNCHER_SERVO_RELEASE;
				}
				else
				{
					launcher.feedServo_info.fourth_servo1_target = LAUNCHER_SERVO_LOCK;
				}
				break;
			case 1:
				if (launcher.feedServo_info.fourth_servo1_target == LAUNCHER_SERVO_LOCK)
				{
					launcher.feedServo_info.second_servo1_target = LAUNCHER_SERVO_RELEASE;
					launcher.feedServo_info.third_servo1_target = LAUNCHER_SERVO_RELEASE;
					launcher.feedServo_info.fourth_servo1_target = LAUNCHER_SERVO_RELEASE;
				}
				else
				{
					launcher.feedServo_info.second_servo1_target = LAUNCHER_SERVO_LOCK;
					launcher.feedServo_info.third_servo1_target = LAUNCHER_SERVO_LOCK;
					launcher.feedServo_info.fourth_servo1_target = LAUNCHER_SERVO_LOCK;
				}
				break;
			default:
				break;
			}
		}
		if (car->wheel_status == CAR_WHEEL_DOWN)
		{
			switch (car->target_id)
			{
			case 2:
				if (launcher.electromagnet_info.second_electromagnet_target == LAUNCHER_ELECTROMAGNET_ON)
				{
					launcher.electromagnet_info.second_electromagnet_target = LAUNCHER_ELECTROMAGNET_OFF;
				}
				else
				{
					launcher.electromagnet_info.second_electromagnet_target = LAUNCHER_ELECTROMAGNET_ON;
				}
				break;
			case 3:
				if (launcher.electromagnet_info.third_electromagnet_target == LAUNCHER_ELECTROMAGNET_ON)
				{
					launcher.electromagnet_info.third_electromagnet_target = LAUNCHER_ELECTROMAGNET_OFF;
				}
				else
				{
					launcher.electromagnet_info.third_electromagnet_target = LAUNCHER_ELECTROMAGNET_ON;
				}
				break;
			case 4:
				if (launcher.electromagnet_info.fourth_electromagnet_target == LAUNCHER_ELECTROMAGNET_ON)
				{
					launcher.electromagnet_info.fourth_electromagnet_target = LAUNCHER_ELECTROMAGNET_OFF;
				}
				else
				{
					launcher.electromagnet_info.fourth_electromagnet_target = LAUNCHER_ELECTROMAGNET_ON;
				}
				break;
			case 1:
				if (launcher.electromagnet_info.second_electromagnet_target == LAUNCHER_ELECTROMAGNET_ON)
				{
					launcher.electromagnet_info.second_electromagnet_target = LAUNCHER_ELECTROMAGNET_OFF;
					launcher.electromagnet_info.third_electromagnet_target = LAUNCHER_ELECTROMAGNET_OFF;
					launcher.electromagnet_info.fourth_electromagnet_target = LAUNCHER_ELECTROMAGNET_OFF;
				}
				else
				{
					launcher.electromagnet_info.second_electromagnet_target = LAUNCHER_ELECTROMAGNET_ON;
					launcher.electromagnet_info.third_electromagnet_target = LAUNCHER_ELECTROMAGNET_ON;
					launcher.electromagnet_info.fourth_electromagnet_target = LAUNCHER_ELECTROMAGNET_ON;
				}
				break;
			default:
				break;
			}

			//				car->task_info.first_widows_total_shoots++;
			//				if (car->task_info.first_widows_total_shoots > 4)
			//				{
			//					car->task_info.first_widows_total_shoots = 1;
			//				}
			//				// 蜂鸣器提示
		}
	}

	// axis force 控制
	if (rc_sensor.info->s2.value == 3) // 右拨杆中
	{
#if !CAR_DEBUG_MODE_ENABLE
		if (car->wheel_status == CAR_WHEEL_UP)
		{
			// 软复位
			HAL_NVIC_SystemReset();
		}
#endif
		if (car->wheel_status == CAR_WHEEL_DOWN) // 后续可能会改静态单发或者动态单发
		{
			// 当前发射镖编号(target_id)
			car->target_id++;
			if (car->target_id > 4)
			{
				car->target_id = 1;
				// 蜂鸣器提示
			}
		}
		force.motor_info.ctrl_mode = MOTOR_PID_SPEED; // 设置力矩电机控制模式为速度控制
		axis.motor_info.ctrl_mode = MOTOR_PID_SPEED;  // 设置轴电机控制模式为速度控制

		force.motor_info.target_speed = rc_sensor.info->ch3 * 10; // 设置力电机目标速度
		axis.motor_info.target_speed = rc_sensor.info->ch0 * 10;  // 设置轴电机目标速度
	}
	else
	{
		if (car->reassign_falg != 1)
		{
			force.motor_info.ctrl_mode = MOTOR_PID_POSITION; // 设置力矩电机控制模式为位置控制
			axis.motor_info.ctrl_mode = MOTOR_PID_POSITION;	 // 设置轴电机控制模式为位置控制
			force.motor_info.target_position = 0;
			axis.motor_info.target_position = 0;
		}
	}
#if CAR_DEBUG_MODE_ENABLE
	if (rc_sensor.info->s2.value == 3) // 左拨杆中
	{
		if (car->wheel_status == CAR_WHEEL_UP)
		{
			launcher.motor6020_info.target_state++;
			if (launcher.motor6020_info.target_state > LAUNCHER_6020_FOURTH_LOAD)
			{
				launcher.motor6020_info.target_state = LAUNCHER_6020_SECOND_STANDBY;
			}
		}
		//		if (car->wheel_status == CAR_WHEEL_DOWN)
		//		{
		//			car->target_id++;
		//			if (car->target_id > 4)
		//			{
		//				car->target_id = 1;
		//				// 蜂鸣器提示
		//			}
		//		}

		Launcher_Motor6020_State_Controler(&launcher);
	}

#endif
	Test_err = abs(launcher.motors.motor3508L->rx_info->encoder_sum) - abs(launcher.motors.motor3508R->rx_info->encoder_sum);
}

/**
 * @brief 手自一体波工作函数
 *
 * @param car
 */
void Car_Work_AMT(car_t *car)
{

	if (last_car_ctrl_mode != CAR_AMT_MODE) // 切换模式处理
	{
		// 目标速度清零
		launcher.motor3508_info.target_speed = 0;		 // 上膛3508电机目标速度清零
		axis.motor_info.target_speed = 0;				 // 轴电机目标速度清零
		axis.motor_info.target_position = 0;			 // 轴电机目标位置清零
		axis.motor_info.ctrl_mode = MOTOR_PID_POSITION;	 // 设置轴电机控制模式为位置控制
		force.motor_info.target_speed = 0;				 // 力矩电机目标速度清零
		force.motor_info.target_position = 0;			 // 力矩电机目标位置清零
		force.motor_info.ctrl_mode = MOTOR_PID_POSITION; // 设置力矩电机控制模式为位置控制
		is_in_AMT_setMode = 0;							 // 退出设置模式
		Indicator_Task_Stop();			// 停止指示器任务
	}

	switch (rc_sensor.info->s2.value)
	{
	case 1: // 右拨杆上 单发测试 轴不动

		Indicator_Task_Stop(); // 停止指示器任务
		if (car->wheel_status == CAR_WHEEL_UP &&
			car->task_info.TotalTaskState == eDeleted) // 单发
		{
			//			car->target_id = single_shoot_static_start_id; // 设置目标id
			if (xTaskCreate(Car_Single_Shot_Static_Task,
							"SingleShotStaticTask",
							512,
							NULL,
							tskIDLE_PRIORITY + 2,
							&xSingleShotStaticTaskHandle) != pdPASS)
			{
			}
		}
		if (car->wheel_status == CAR_WHEEL_DOWN &&
			car->task_info.TotalTaskState == eDeleted) // 四连发
		{
			//			car->target_id = four_shot_static_start_id; // 设置目标id
			if (xTaskCreate(Car_Four_Shot_Static_Task,
							"FourShotStaticTask",
							512,
							NULL,
							tskIDLE_PRIORITY + 2,
							&xFourShotStaticTaskHandle) != pdPASS)
			{
			}
		}

		break;
	case 3: // 右拨杆中 单发调参
		if (car->wheel_status == CAR_WHEEL_HOLD_DOWN)
		{
			Indicator_Task_Start();
			is_in_AMT_setMode = 1; // 进入设置模式
		}
		if (is_in_AMT_setMode)
		{
			buzzer.indicate_num = car->target_id;
			if (car->wheel_status == CAR_WHEEL_DOWN)
			{
				if (car->target_id < 4)
				{
					car->target_id++; // 目标id加1
				}
				else
				{
					car->target_id = 1; // 重置目标id
				}
			}
			else if (car->wheel_status == CAR_WHEEL_UP)
			{
				Indicator_Task_Stop(); // 停止指示器任务
				is_in_AMT_setMode = 0; // 退出设置模式
			}
		}
		//		if (car->wheel_status == CAR_WHEEL_UP &&
		//			car->task_info.TotalTaskState == eDeleted) // 第一窗发射镖数
		//		{
		//			car->task_info.first_widows_total_shoots++;
		//			if (car->task_info.first_widows_total_shoots > 4)
		//			{
		//				car->task_info.first_widows_total_shoots = 1;
		//			}
		//		}
		break;
	case 2: // 右拨杆下 带偏置四连发

		Indicator_Task_Stop(); // 停止指示器任务
		is_in_AMT_setMode = 0; // 退出设置模式

		if (car->wheel_status == CAR_WHEEL_DOWN &&
			car->task_info.TotalTaskState == eDeleted) // 創建任務
		{

			car->target_id = four_shot_dynamic_start_id;
			if (xTaskCreate(Car_Four_Shot_Dynamic_Task,
							"FourShotDynamicTask",
							512,
							NULL,
							tskIDLE_PRIORITY + 2,
							&xFourShotDynamicTaskHandle) != pdPASS)
			{
			}
		}
		else if (car->wheel_status == CAR_WHEEL_UP &&
				 car->task_info.TotalTaskState == eDeleted)
		{
			if (xTaskCreate(Car_Single_Shot_Dynamic_Task,
							"SingleShotDynamicTask",
							512,
							NULL,
							tskIDLE_PRIORITY + 2,
							&xSingleShotDynamicTaskHandle) != pdPASS)
			{
			}
		}
		break;
	default:
		break;
	}
}

/**
 * @brief 比赛模式
 *
 * @param car
 */
void Car_Work_AT(car_t *car)
{
	if (last_car_ctrl_mode != CAR_AT_MODE) // 切换模式处理
	{
		// 目标速度清零
		launcher.motor3508_info.target_speed = 0;		 // 上膛3508电机目标速度清零
		axis.motor_info.target_speed = 0;				 // 轴电机目标速度清零
		axis.motor_info.target_position = 0;			 // 轴电机目标位置清零
		axis.motor_info.ctrl_mode = MOTOR_PID_POSITION;	 // 设置轴电机控制模式为位置控制
		force.motor_info.target_speed = 0;				 // 力矩电机目标速度清零
		force.motor_info.target_position = 0;			 // 力矩电机目标位置清零
		force.motor_info.ctrl_mode = MOTOR_PID_POSITION; // 设置力矩电机控制模式为位置控制
		is_in_AMT_setMode = 0;							 // 退出设置模式
		Indicator_Task_Stop();							 // 停止指示器任务
		car->task_info.at_step = CAR_AT_STEP_INIT_STATE_CHECK;
//		car->is_round_two = 0;
	}

	switch (car->task_info.at_step)
	{
	case CAR_AT_STEP_INIT_STATE_CHECK:
		car->target_id = 1; // 重置目标id
		axis.task_info.axis_task_arg.is_vision_on = 1;
		axis.task_info.axis_task_arg.is_axis_delay = 0;
		launcher.fireServo_info.shoot_servo_target = LAUNCHER_SERVO_LOCK;
		launcher.motor3508_info.target_state = LAUNCHER_3508_UP;
		launcher.motor6020_info.target_state = LAUNCHER_6020_SECOND_STANDBY;
		Launcher_Motor3508_State_Controler(&launcher); // 控制3508电机状态
		if (car->judge_info.game_progress >= GAME_IN_CHECKING)
		{
			Force_Task_Start();
			Axis_Task_Start();
			car->task_info.at_step = CAR_AT_STEP_GAME_IN_CHECKING;
		}
		break;
	case CAR_AT_STEP_GAME_IN_CHECKING:
		launcher.motors.motor3508R->ctrl->angle_ctrl_inner->out_max = launcher.config.motor3508_firstmaxspeed; // 设置3508电机加载速度
		launcher.motor3508_info.target_state = LAUNCHER_3508_UP;											   // 设置3508电机目标状态为上膛
		Launcher_Motor3508_State_Controler(&launcher);														   // 控制3508电机状态
		if (car->judge_info.game_progress >= GAME_IN_PROGRESS)
		{
			car->task_info.at_step = CAR_AT_STEP_WAIT_FOR_CMD;
		}
		break;
	case CAR_AT_STEP_WAIT_FOR_CMD:
		if (car->judge_info.launch_opening_status == CAR_LAUNCH_OPENING)
		{
			if (car->target_id == 1)
			{
				Launcher_Task_Start();								 // 启动发射器任务
				launcher.task_info.launcher_task_flag.Load_Flag = 1; // 设置加载信号
			}
			car->task_info.at_step = CAR_AT_STEP_LAUNCH_OPENING;
		}
		break;
	case CAR_AT_STEP_LAUNCH_OPENING:
		if (car->judge_info.launch_opening_status == CAR_LAUNCH_ON)
		{
			if (xTaskCreate(Car_Four_Shot_Dynamic_Task,
							"FourShotDynamicTask",
							512,
							NULL,
							tskIDLE_PRIORITY + 2,
							&xFourShotDynamicTaskHandle) == pdPASS)
			{
				car->task_info.FourShotDynamic_TaskState = eRunning;
			}
			car->task_info.at_step = CAR_AT_STEP_LAUNCH_ON;
		}
		break;
	case CAR_AT_STEP_LAUNCH_ON:
		if (car->task_info.FourShotDynamic_TaskState == eDeleted)
		{
			if (car->target_id == 1) // 全部发射完
			{
				car->task_info.at_step = CAR_AT_STEP_LAUNCH_DONE; // 四连发任务完成
				axis.task_info.axis_task_arg.is_axis_delay = 0;
			}
			else // 还有没射完
			{
				axis.task_info.axis_task_arg.is_axis_delay = 1;	   // 第二个窗口期打移动
				car->task_info.at_step = CAR_AT_STEP_WAIT_FOR_CMD; // 等待下一条命令
			}
		}

		break;
	case CAR_AT_STEP_LAUNCH_DONE:
		break;
	default:
		break;
	}
}

/**
 * @brief 初始化工作函数
 *
 * @param car
 */
void Car_Work_Init(car_t *car)
{
	Laucher_Init_Task(&launcher);
	if (launcher.state >= LAUNCHER_FREE)
	{
		last_car_ctrl_mode = car->car_ctrl_mode; // 记录上次控制模式
		car->car_ctrl_mode = CAR_MT_MODE;		 // 初始化完成后设置为手动波模式
	}
}
/**
 *@brief 堵转复位任务
 *
 */
void Car_Reassign_Task(car_t *car)
{
	Axis_Reassign_Task(&axis);
	Force_Reassign_Task(&force);
}

#ifdef OUTPOST_ALIVE_CHECK

void rebuild_check(car_t* car)
{
	if (vision.rx_info->target_status == 0) 
	{
		// 发射任务检测
		if (car->task_info.FourShotDynamic_TaskState == eRunning)
		{
			// 先杀死 launcher task，防止装填状态机继续改写 3508 target_state
			if (xLauncherTaskHandle != NULL)
			{
				vTaskDelete(xLauncherTaskHandle);
				xLauncherTaskHandle = NULL;
			}
			launcher.task_info.LauncherTaskState = eDeleted;
	
			// 杀完 launcher task 后没人调 Launcher_Motor3508_State_Controler，
			// 所以直接设 target_position + ctrl_mode，PID 在 ControlTask 里照常跑
			launcher.motor3508_info.target_state = LAUNCHER_3508_UP;
			launcher.motor3508_info.ctrl_mode = MOTOR_PID_POSITION;
			launcher.motor3508_info.motor3508L_target_position = launcher.config.motor3508_up_position;

			if (xFourShotDynamicTaskHandle != NULL)
			{
				vTaskDelete(xFourShotDynamicTaskHandle);
				xFourShotDynamicTaskHandle = NULL;
			}
			if (xFourShotStaticTaskHandle != NULL)
			{
				vTaskDelete(xFourShotStaticTaskHandle);
				xFourShotStaticTaskHandle = NULL;
			}
//			// 检查LauncherTaskState是否为eDeleted，如果不是，说明发射任务没有正常结束，可能需要重启发射任务
//			if (launcher.task_info.LauncherTaskState != eDeleted)
//			{
//				// 如果已经上膛，就打完这发再结束任务，如果没上膛，结束任务
//				if (launcher.task_info.launcher_load_state >= LAUNCHER_WAIT_SHOOT_SERVO_TO_LOCK ||
//					launcher.state >= LAUNCHER_LOADED)
//				{
//					return;
//					// 等待发射任务完成
//				}
//				else if (launcher.fireServo_info.shoot_servo_target != LAUNCHER_SERVO_LOCK &&
//						 launcher.state < LAUNCHER_LOADED &&
//						 launcher.task_info.launcher_load_state <= LAUNCHER_WAIT_3508_TO_DOWN)
//				{
//					// 直接结束发射任务，并复位状态
//					launcher.task_info.LauncherTaskState = eDeleted;
//					launcher.motor3508_info.target_state = LAUNCHER_3508_UP;
//					if (launcher.motor3508_info.state == LAUNCHER_SUBMODULE_DONE)
//					{
//						launcher.state = LAUNCHER_FREE;
//					}
//					if (xFourShotDynamicTaskHandle != NULL)
//					{
//						vTaskDelete(xFourShotDynamicTaskHandle);
//						xFourShotDynamicTaskHandle = NULL;
//					}
//					if (xFourShotStaticTaskHandle != NULL)
//					{
//						vTaskDelete(xFourShotStaticTaskHandle);
//						xFourShotStaticTaskHandle = NULL;
//					}
//				}
//			}
			// 及时关闭发射任务
		}
		else
		{
			return;
		}
	}

}
#endif
