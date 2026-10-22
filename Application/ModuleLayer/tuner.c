#include "tuner.h"
#include "drv_flash.h"
#include "uartscreen_protocol.h"

#define  M3508_ENCODER_SWITCHING    78656.3422459893f
#define  GM2006_ENCODER_SWITCHING	73728.0f
#define USER_FLASH_ADDRESS  ADDR_FLASH_SECTOR_11
#define FLASH_DATA_LENGHT sizeof(tuner_flash_info_t)  

static void Tunner_data_update(void);

tuner_t tuner = 
{
  .status = TUNER_OFFLINE,
};

static TaskHandle_t xTunerTaskHandle;
static uint8_t is_update_flash = 0; // 是否更新flash

/*******************************************************/
/********************** 内部函数 ************************/
/*******************************************************/

/**
 * @brief 写入调谐器信息到Flash
 * 
 * @param tuner_flash_info 
 * @return HAL_StatusTypeDef 
 */
static HAL_StatusTypeDef Tuner_Write_Flash_Info()
{
  tuner_flash_info_t *tuner_flash_info = &tuner.tuner_write_info;
  if (tuner.status != TUNER_INITED)
  {
    return HAL_ERROR; // 如果未初始化, 则不进行写入操作
  }
  
  //擦除flash页
  flash_erase_address(USER_FLASH_ADDRESS, 1);
  
  //往flash写数据
  if (flash_write_single_address(USER_FLASH_ADDRESS, 
                                (uint32_t *)tuner_flash_info, 
                                (FLASH_DATA_LENGHT + 3) / 4) == 0)
  {
    return HAL_OK; // 写入成功
  }
  else
  {
    return HAL_ERROR; // 写入失败
  }
}

/**
 * @brief 读取调谐器信息从Flash
 * 
 * @return HAL_StatusTypeDef 
 */
static HAL_StatusTypeDef Tuner_Read_Flash_Info(void)
{
  if (tuner.status != TUNER_INITED)
  {
    return HAL_ERROR; // 如果调谐器未初始化, 则不进行读取操作
  }
  
  //从flash读取调谐器信息
  flash_read(USER_FLASH_ADDRESS, (uint32_t *)&tuner.tuner_read_flash_info, (FLASH_DATA_LENGHT + 3) / 4);
  return HAL_OK;
}

static void Tuner_UpdateTask(void *pvParameters){

  for (;;)
  {
    if (is_update_flash == 0)
    {
		vTaskDelay(500);
		continue;
    }
    
		Tunner_data_update();
//    tuner_flash_info_t *tuner_write_info = &tuner.tuner_write_info;
//    gui_rx_offset_setting_info_t *rx_offset_setting_info = gui.rx_offset_setting_info;

//    tuner_write_info->axis_all_offset = rx_offset_setting_info->set_axis_all_offset;
//    tuner_write_info->force_all_offset = rx_offset_setting_info->set_force_all_offset;

//    tuner_write_info->axis_first_offset = rx_offset_setting_info->set_axis_first_offset;
//    tuner_write_info->axis_second_offset = rx_offset_setting_info->set_axis_second_offset;
//    tuner_write_info->axis_third_offset = rx_offset_setting_info->set_axis_third_offset;
//    tuner_write_info->axis_fourth_offset = rx_offset_setting_info->set_axis_fourth_offset;
//    
//    tuner_write_info->force_first_offset = rx_offset_setting_info->set_force_first_offset;
//    tuner_write_info->force_second_offset = rx_offset_setting_info->set_force_second_offset;
//    tuner_write_info->force_third_offset = rx_offset_setting_info->set_force_third_offset;
//    tuner_write_info->force_fourth_offset = rx_offset_setting_info->set_force_fourth_offset;
//    
//    tuner_write_info->first_windows_shoots = rx_offset_setting_info->set_first_windows_shoots;
    while (Tuner_Write_Flash_Info() != HAL_OK){}
    Tuner_Read_Flash_Info();
    is_update_flash = 0; // 重置更新标志
  }
}

/*******************************************************/
/********************** 外部函数 ************************/
/*******************************************************/

/**
 * @brief 初始化函数
 * 
 */
void Tuner_Init(void)
{
  flash_read(USER_FLASH_ADDRESS, (uint32_t *)&tuner.tuner_init_flash_info, (FLASH_DATA_LENGHT + 3) / 4);
  tuner.status = TUNER_INITED;
  Tuner_Read_Flash_Info(); // 读取flash信息到tuner_read_flash_info
    // 创建任务来更新调谐器信息
  if (xTunerTaskHandle == NULL)
  {
    if (xTaskCreate(Tuner_UpdateTask, 
        "TunerUpdate", 
        512, 
        NULL, 
        tskIDLE_PRIORITY, 
        &xTunerTaskHandle) != pdPASS)
    {
    }
  }
}

void Tuner_UpdateTask_Start(void)
{
  if (tuner.status != TUNER_INITED)
  {
    return; // 如果调谐器未初始化, 则不启动更新任务
  }
  is_update_flash = 1; // 设置更新标志
  
}

static void Tunner_data_update(void)
{
	tuner_flash_info_t *tuner_write_info = &tuner.tuner_write_info;
	
	switch(tjc.tjc__all_rx_info->packet_header.pack_id)
	{
	case 0:
//		tuner_write_info->up_3508_position = tjc.tjc__all_rx_info->launcher_info.up_position;
//		tuner_write_info->down_3508_position = tjc.tjc__all_rx_info->launcher_info.down_position;
//		tuner_write_info->second_6020_standy = tjc.tjc__all_rx_info->launcher_info.standy2;
//		tuner_write_info->thrid_6020_standy = tjc.tjc__all_rx_info->launcher_info.standy3;
//		tuner_write_info->fourth_6020_standy = tjc.tjc__all_rx_info->launcher_info.standy4;
//		tuner_write_info->second_6020_load = tjc.tjc__all_rx_info->launcher_info.load2;
//		tuner_write_info->thrid_6020_load = tjc.tjc__all_rx_info->launcher_info.load3;
//		tuner_write_info->fourth_6020_load = tjc.tjc__all_rx_info->launcher_info.load4;
//				
//	    tuner_write_info->axis_all_offset = (int32_t)((float)tjc.tjc__all_rx_info->force_axis_info.force_all_offest * M3508_ENCODER_SWITCHING/100);
//	    tuner_write_info->force_first_offset = (int32_t)((float)tjc.tjc__all_rx_info->force_axis_info.force1_target * M3508_ENCODER_SWITCHING/100);
//	    tuner_write_info->force_second_offset = (int32_t)((float)tjc.tjc__all_rx_info->force_axis_info.force2_target * M3508_ENCODER_SWITCHING/100);
//	    tuner_write_info->force_third_offset = (int32_t)((float)tjc.tjc__all_rx_info->force_axis_info.force3_target * M3508_ENCODER_SWITCHING/100);
//	    tuner_write_info->force_fourth_offset = (int32_t)((float)tjc.tjc__all_rx_info->force_axis_info.force4_target * M3508_ENCODER_SWITCHING/100);
//		tuner_write_info->force_all_offset = (int32_t)((float)tjc.tjc__all_rx_info->force_axis_info.axis_all_offest * GM2006_ENCODER_SWITCHING/100);
//		tuner_write_info->axis_first_offset = (int32_t)((float)tjc.tjc__all_rx_info->force_axis_info.axis1_target * GM2006_ENCODER_SWITCHING/100);
//	    tuner_write_info->axis_second_offset = (int32_t)((float)tjc.tjc__all_rx_info->force_axis_info.axis2_target * GM2006_ENCODER_SWITCHING/100);
//	    tuner_write_info->axis_third_offset = (int32_t)((float)tjc.tjc__all_rx_info->force_axis_info.axis3_target * GM2006_ENCODER_SWITCHING/100);
//	    tuner_write_info->axis_fourth_offset = (int32_t)((float)tjc.tjc__all_rx_info->force_axis_info.axis4_target * GM2006_ENCODER_SWITCHING/100  );
//		
//		tuner_write_info->first_windows_shoots = tjc.tjc__all_rx_info->shoot_info.frist_window_shoot;
		
		break;
	case 1:
//		tuner_write_info->up_3508_position = tjc.tjc__all_rx_info->launcher_info.up_position;
//		tuner_write_info->down_3508_position = tjc.tjc__all_rx_info->launcher_info.down_position;
//		tuner_write_info->second_6020_standy = tjc.tjc__all_rx_info->launcher_info.standy2;
//		tuner_write_info->thrid_6020_standy = tjc.tjc__all_rx_info->launcher_info.standy3;
//		tuner_write_info->fourth_6020_standy = tjc.tjc__all_rx_info->launcher_info.standy4;
//		tuner_write_info->second_6020_load = tjc.tjc__all_rx_info->launcher_info.load2;
//		tuner_write_info->thrid_6020_load = tjc.tjc__all_rx_info->launcher_info.load3;
//		tuner_write_info->fourth_6020_load = tjc.tjc__all_rx_info->launcher_info.load4;		
		break;
	case 2:
		break;
	case 3:
		tuner_write_info->force_all_offset = (int32_t)((float)tjc.tjc__all_rx_info->force_axis_info.force_all_offest * M3508_ENCODER_SWITCHING/100);
	    tuner_write_info->force_first_offset = (int32_t)((float)tjc.tjc__all_rx_info->force_axis_info.force1_target * M3508_ENCODER_SWITCHING/100);
	    tuner_write_info->force_second_offset = (int32_t)((float)tjc.tjc__all_rx_info->force_axis_info.force2_target * M3508_ENCODER_SWITCHING/100);
	    tuner_write_info->force_third_offset = (int32_t)((float)tjc.tjc__all_rx_info->force_axis_info.force3_target * M3508_ENCODER_SWITCHING/100);
	    tuner_write_info->force_fourth_offset = (int32_t)((float)tjc.tjc__all_rx_info->force_axis_info.force4_target * M3508_ENCODER_SWITCHING/100);
	    tuner_write_info->axis_all_offset = (int32_t)((float)tjc.tjc__all_rx_info->force_axis_info.axis_all_offest * GM2006_ENCODER_SWITCHING/100);
		tuner_write_info->axis_first_offset = (int32_t)((float)tjc.tjc__all_rx_info->force_axis_info.axis1_target * GM2006_ENCODER_SWITCHING/100);
	    tuner_write_info->axis_second_offset = (int32_t)((float)tjc.tjc__all_rx_info->force_axis_info.axis2_target * GM2006_ENCODER_SWITCHING/100);
	    tuner_write_info->axis_third_offset = (int32_t)((float)tjc.tjc__all_rx_info->force_axis_info.axis3_target * GM2006_ENCODER_SWITCHING/100);
	    tuner_write_info->axis_fourth_offset = (int32_t)((float)tjc.tjc__all_rx_info->force_axis_info.axis4_target * GM2006_ENCODER_SWITCHING/100  );
	
	
		break;
	case 4:
		tuner_write_info->first_windows_shoots = tjc.tjc__all_rx_info->shoot_info.frist_window_shoot;
		
		break;
	}
}


