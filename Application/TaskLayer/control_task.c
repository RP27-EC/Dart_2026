/**
  ******************************************************************************
  * @file    control_task.c
  * @brief   
  ******************************************************************************
  */
#include "control_task.h"
#include "can_protocol.h"
#include "launcher.h"

float t;
void StartControlTask(void const * argument)
{

	for(;;) 
	{
		Device_Work();
		CAN1_Send();
		CAN2_Send();
		osDelay(1);

	}
}



