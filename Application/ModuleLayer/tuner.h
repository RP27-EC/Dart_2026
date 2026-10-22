#ifndef _TUNER_H
#define _TUNER_H

#include "rp_config.h"
#include "main.h"
#include "FreeRTOS.h"
#include "semphr.h"

typedef struct tuner_flash_info
{
  int32_t up_3508_position; 
  int32_t down_3508_position; 

  int32_t second_6020_standy;
  int32_t thrid_6020_standy; 
  int32_t fourth_6020_standy;
  int32_t second_6020_load; 
  int32_t thrid_6020_load; 
  int32_t fourth_6020_load; 

  int32_t axis_all_offset;
  int32_t axis_first_offset ;
  int32_t axis_second_offset ;
  int32_t axis_third_offset; 
  int32_t axis_fourth_offset;
  int32_t force_all_offset;
  int32_t force_first_offset; 
  int32_t force_second_offset;
  int32_t force_third_offset;
  int32_t force_fourth_offset;
//  int32_t frist_force_target; // 力量第一发位置偏移
//  int32_t second_force_target; // 力量第二发位置偏移
//  int32_t thrid_force_target; // 力量第三发位置偏移
//  int32_t fourth_force_target; // 力量第四发位置偏移

//  int32_t frist_axis_target; // yaw第一发位置偏移
//  int32_t second_axis_target; // yaw第二发位置偏移
//  int32_t thrid_axis_target; // yaw第三发位置偏移
//  int32_t fourth_axis_target; // yaw第四发位置偏移	


  uint8_t first_windows_shoots; // 第一窗射击次数
  uint8_t target_id;
  uint8_t launcher_open_flag;
  uint8_t rebuild_flag;
  
} tuner_flash_info_t;

typedef enum
{
  TUNER_OFFLINE = 0, // 调谐器离线
  TUNER_INITED, // 调谐器已初始化
} tuner_status_e;

typedef struct tuner_class_t
{
  tuner_flash_info_t tuner_init_flash_info; // 上电时闪存信息

  tuner_flash_info_t tuner_read_flash_info; // 读取的闪存信息

  tuner_flash_info_t tuner_write_info; // 要写入信息

  tuner_status_e status; // 调谐器状态

} tuner_t;

extern tuner_t tuner;

void Tuner_UpdateTask_Start(void);
void Tuner_Init(void);

#endif
