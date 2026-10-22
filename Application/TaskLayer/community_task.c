#include "community_task.h"
#include "rc_sensor.h"
#include "car.h"
// #include "imu_sensor.h"

void StartCommunityTask(void const *argument)
{


	for (;;)
	{
		// imu_sensor.update(&imu_sensor);
		Car_Ctrl(&car);
		osDelay(1);
	}
}
