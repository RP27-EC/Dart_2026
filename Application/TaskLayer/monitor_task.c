/**
 ******************************************************************************
 * @file    monitor_task.c
 * @brief   监控任务
 *          1. 各模块心跳失联检测
 *          2. 监控遥控器状态，软件复位
 ******************************************************************************
 */
#include "monitor_task.h"
#include "device.h"
#include "judge_protocol.h"
#include "vision_protocol.h"
#include "rc_sensor.h"
#include "launcher.h"
// #include "imu_sensor.h"
#include "iwdg.h"
#include "car.h"

void StartMonitorTask(void const *argument)
{

	static uint8_t vision_delay_cnt = 1;
	for (;;)
	{
		Device_HeartBeat();
		rc_sensor.heart_beat(&rc_sensor);
		Judge_HeartBeat(&judge);
		Vision_HeartBeat();
		// imu_sensor.heart_beat(&imu_sensor.work_state);
		if (vision_delay_cnt >= 5)
		{
			Vision_DataTx();
			vision_delay_cnt = 1;
		}
		else
		{
			vision_delay_cnt++;
		}
		/* 喂看门狗，确保系统在正常运行时不会被复位 */
//		HAL_IWDG_Refresh(&hiwdg);

		osDelay(1);
	}
}
