/**
 * @file buzzer.c
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2025-06-14
 * 
 * @copyright Copyright (c) 2025
 * 
 */

/* 包含头文件 ----------------------------------------------------------------*/
#include "buzzer.h"
#include "tim.h"
#include "FreeRTOS.h"
#include "task.h"
/* 内部宏定义 ----------------------------------------------------------------*/

/* 内部自定义数据类型的变量 --------------------------------------------------*/
buzzer_t buzzer = {
  .indicate_num = 2, // 指示音数量
};

/* 内部变量 ------------------------------------------------------------------*/
uint8_t is_stop_indicator_task = 0; // 是否停止指示器任务
#if ENABLE_INIT_MUSIC
static TaskHandle_t xInitMusicTaskHandle = NULL;
#endif
static TaskHandle_t xIndicatorTaskHandle = NULL;
const tNote MaxSong[] = {
    {note_3E, 280}, {1,83},{note_3E, 280}, {1,83},{note_3E, 280},{note_E, 416},{1,200},  //dududu do
    {note_D, 280},{1,83},{note_D, 280},{1,83},{note_C, 280},{note_B, 416}, // max ver

		{0,0}//end

};

/* 内部函数原型声明 ----------------------------------------------------------*/
void Timer_Set(TIM_HandleTypeDef* htim, uint32_t channel, uint32_t frequency, float dutyCycle);

/* 函数主体部分 --------------------------------------------------------------*/
/**
  * @brief				蜂鸣器和LED任务
  * @param[out]		不同频率及相应节拍
  * @param[in]		note音符，long音长
  * @retval				
*/
float db_value = 10;
void Note(int note,float Long)
{
	Timer_Set(&BUZZER_HTIM, BUZZER_CHANNEL, note, db_value);

	vTaskDelay(pdMS_TO_TICKS((uint32_t)Long));
}

// 设置占空比和频率的函数
void Timer_Set(TIM_HandleTypeDef* htim, uint32_t channel, uint32_t frequency, float dutyCycle)
{
  // 确保频率和占空比在合理范围内
  if (frequency == 0 || dutyCycle < 0.0f || dutyCycle > 100.0f)
      return;

  // 计算定时器的时钟频率
  uint32_t timerClock = 8400000; // 定时器的时钟频率

  // 计算周期和预分频器
  uint32_t prescaler = timerClock / frequency;
  
  __HAL_TIM_SET_AUTORELOAD(htim, prescaler);//设置ARR

  // 计算比较值
  uint32_t compareValue = (uint32_t)((dutyCycle / 100.0f) * prescaler);

  // 根据通道设置比较值

  __HAL_TIM_SET_COMPARE(htim, channel, compareValue);
  HAL_TIM_PWM_Start(htim,channel); 
}

#if ENABLE_INIT_MUSIC
void InitMusic_Task(void *pvParameters)
{
	static uint8_t i=0;
	while(1)
	{
	  if(MaxSong[i].mTime==0 && MaxSong[i].mName==0)
		{
			Timer_Set(&BUZZER_HTIM, BUZZER_CHANNEL, 1, 0);
			break;
		}
		Note(MaxSong[i].mName,MaxSong[i].mTime);
		i++;
	}
  xInitMusicTaskHandle = NULL; // 任务句柄置空
  vTaskDelete(NULL); // 删除任务
}
#endif // ENABLE_INIT_MUSIC

void Indicator_Task(void *pvParameters)
{
  // 任务循环
  for(;;){
    if (is_stop_indicator_task)
    {
      // 如果停止标志被设置，则删除任务
      xIndicatorTaskHandle = NULL;
      vTaskDelete(NULL);
    }
    
    for (size_t i = 0; i < buzzer.indicate_num; i++)
    {
      Note(note_E, 200); // 播放音符
      Note(1, 100);   // 播放音符
    }
    vTaskDelay(pdMS_TO_TICKS(1000)); // 延时1000毫秒
  }
}


HAL_StatusTypeDef InitMusicTask_Start(void)
{
#if ENABLE_INIT_MUSIC

	// 检查任务状态
	if (xInitMusicTaskHandle != NULL) {
      // 任务还在
      return HAL_BUSY;
	}
	// 任务已删除
	if (xTaskCreate(InitMusic_Task, 
									"InitMusicTask", 128, 
									NULL, tskIDLE_PRIORITY + 2, 
									&xInitMusicTaskHandle) != pdPASS) {
		return HAL_ERROR;
	}	
	else
	{
		return HAL_OK;
	}
#else
  return HAL_OK;
#endif
}

HAL_StatusTypeDef Indicator_Task_Start(void)
{
  // 检查任务状态
  if (xIndicatorTaskHandle != NULL) {
    // 任务还在
    return HAL_BUSY;
  }
  is_stop_indicator_task = 0; // 重置停止标志
  // 任务已删除
  if (xTaskCreate(Indicator_Task, 
                  "IndicatorTask", 128, 
                  NULL, tskIDLE_PRIORITY, 
                  &xIndicatorTaskHandle) != pdPASS) {
    return HAL_ERROR;
  }	
  else
  {
    return HAL_OK;
  }
}

HAL_StatusTypeDef Indicator_Task_Stop(void)
{
  is_stop_indicator_task = 1; // 设置停止标志
  return HAL_OK;
}


void Buzzer_Error_Indicate(void)
{
  Note(note_G, 200); // 播放音符
  Note(1, 100);   // 播放音符
  Note(note_E, 200); // 播放音符
  Note(1, 100);   // 播放音符
  vTaskDelay(pdMS_TO_TICKS(1000)); // 延时1000毫秒
}
void Buzzer_Stop(void)
{
  Timer_Set(&BUZZER_HTIM, BUZZER_CHANNEL, 1, 0);
}
