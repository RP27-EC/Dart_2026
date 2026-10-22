#ifndef __BUZZER_TASK_H
#define __BUZZER_TASK_H

#include "tim.h"
#include "gpio.h"
#include "main.h"
#include "FreeRTOS.h"
#include "semphr.h"

#define ENABLE_INIT_MUSIC 1

/* 本模块向外部提供的宏定义 -------------------------------------------------*/
#define BUZZER_HTIM  htim4
#define BUZZER_CHANNEL TIM_CHANNEL_3

#define note_A  220
#define note_3A 110  
#define note_5A 440  
#define note_sA 233  //233.082
#define note_B  247  //246.942
#define note_3B  123  //123.471
#define note_5B  494  //493.883
#define note_C  262  //261.626
#define note_5C  523  //523.251
#define note_sC 277  //277.183
#define note_D  294  //293.665
#define note_sD 311  //311.127
#define note_5D 587  //587.33
#define note_3sD 156  //155.563
#define note_E  330  //329.629
#define note_3E  165  //164.814
#define note_F  349  //349.228
#define note_3F  175  //174.614
#define note_sF 370  //369.994
#define note_3sF 185  //184.997
#define note_G  392  //391.995
#define note_sG 415  //415.305
#define note_3G 196  //195.998
#define note_5sG 831  //830.609


typedef struct
{
	short mName;//音名：取值L1~L7、M1~M7、H1~H7分别表示低音、中音、高音的1234567，取0表示休止符
	short mTime;//时值：取值T、T/2、T/4、T/8、T/16、T/32分别表示全音符、二分音符、四音符、八音符...取0表示演奏结束
	
}tNote;


typedef struct buzzer
{
	uint8_t indicate_num;
}buzzer_t;

extern buzzer_t buzzer;

HAL_StatusTypeDef InitMusicTask_Start(void);
HAL_StatusTypeDef Indicator_Task_Start(void);
HAL_StatusTypeDef Indicator_Task_Stop(void);

void Buzzer_Error_Indicate(void);
void Buzzer_Stop(void);



#endif
