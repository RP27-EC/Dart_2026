#ifndef __JUDGE_PROTOCOL_H
#define __JUDGE_PROTOCOL_H

#include "rp_config.h"
#include "main.h"
#include "stdbool.h"
#define JUDGE_FRAME_HEADER 0xA5 // 此为帧头的SOF,帧头分为 SOF,length,Seq,CRC8
//***********裁判系统的分类信息***************/
enum
{
	ID_game_state = 0x0001,	   // 比赛状态数据，1Hz
	ID_game_result = 0x0002,   // 比赛结果数据，比赛结束发送
	ID_game_robot_HP = 0x0003, // 比赛机器人血量数据，1Hz发送
	//	ID_dart_status									= 0x0004,//飞镖发射状态，飞镖发射时发送
	ID_ICRA_buff_debuff_zone_status = 0x0005, // 人工智能挑战赛加成与惩罚区状态，1Hz
	ID_event_data = 0x0101,					  // 场地事件数据，事件 ——改变后—— 发送
	//	ID_supply_projectile_action   	= 0x0102,//场地补给站动作标识数据，动作 ——改变后—— 发送
	//	ID_supply_projectile_booking    = 0x0103,//请求补给站补弹数据 ——参赛队发送——（RM 对抗赛尚未开）10Hz
	ID_referee_warning = 0x0104,	 // 裁判警告数据，——警告后——发送
	ID_dart_remaining_time = 0x0105, // 飞镖发射口倒计时，1Hz
	ID_game_robot_state = 0x0201,	 // 机器人状态数据，10Hz
	ID_power_heat_data = 0x0202,	 // 实时功率热量数据，50Hz
	ID_game_robot_pos = 0x0203,		 // 机器人位置数据，10Hz
	ID_buff_musk = 0x0204,			 // 机器人增益数据，1Hz
	//	ID_aerial_robot_energy					= 0x0205,//空中机器人能量状态数据，10Hz，只有——空中机器人主控——发送
	ID_robot_hurt = 0x0206,		  // 伤害状态数据，伤害发生后发送
	ID_shoot_data = 0x0207,		  // 实时射击数据，子弹发射后发送
	ID_bullet_remaining = 0x0208, // 弹丸剩余发送数，仅——空中机器人，哨兵机器人——以及ICRA机器人发送，1Hz
	ID_rfid_status = 0x0209,	  // 机器人RFID状态，1Hz

	ID_dart_client_directive = 0x020A, // 飞镖机器人客户端指令书, 3Hz

	ID_robot_interactive_header_data = 0x0301,		// 机器人交互数据，——发送方触发——发送 10Hz
	ID_controller_interactive_header_data = 0x0302, // 自定义控制器交互数据接口，通过——客户端触发——发送 30Hz
	ID_map_interactive_header_data = 0x0303,		// 客户端小地图交互数据，——触发发送——
	ID_keyboard_information = 0x0304,				// 键盘、鼠标信息，通过——图传串口——发送
	//  ID_smallmap                           = 0x0305,//客户端小地图接收信息
};
// 命令码枚举CMD_ID
//**************裁判系统各个信息的长度（单位/字节）**************************/
enum judge_data_length_t
{
	/* Std */
	LEN_FRAME_HEAD = 5, // 帧头长度
	LEN_CMD_ID = 2,		// 命令码长度
	LEN_FRAME_TAIL = 2, // 帧尾CRC16
	/* Ext */
	// 0x000x
	LEN_GAME_STATUS = 11,	// 1
	LEN_GAME_RESULT = 1,	// 2
	LEN_GAME_ROBOT_HP = 16, // 3
	//	LEN_DART_STATUS					         = 3,
	LEN_ICRA_BUFF_DEBUFF_ZONE_STATUS = 11, // 0x0005
	// 0x010x
	LEN_EVENT_DATA = 4, // 1
	//	LEN_SUPPLY_PROJECTILE_ACTION	   = 4,//！！！！！！！！！！！！！！！！！
	//	LEN_SUPPLY_PROJECTILE_BOOKING	   = 2,//对抗赛未开启
	LEN_REFEREE_WARNING = 3,	 // 4
	LEN_DART_REMAINING_TIME = 3, // 0x0105
	// 0x020x
	LEN_GAME_ROBOT_STATUS = 13, // 15!!!!!!!!!!!!!!!!!!!!!!!!!!!   //1
	LEN_POWER_HEAT_DATA = 14,	// ！！！！！！！！！！				//2
	LEN_GAME_ROBOT_POS = 12,	// 3
	LEN_BUFF_MASK = 8,			// 4
	//	LEN_AERIAL_ROBOT_ENERGY 	     	 = 1,//！！！！！
	LEN_ROBOT_HURT = 1,			   // 6
	LEN_SHOOT_DATA = 7,			   // ！！！！						//7
	LEN_BULLET_REMAINING = 6,	   // ！！！！
	LEN_RFID_STATUS = 5,		   // 9
	LEN_DART_CLIENT_DIRECTIVE = 6, // 0x020A								//6

	// 0x030x
	// LEN_robot_interactive_header_data      = n,
	// LEN_controller_interactive_header_data = n,
	LEN_MAP_INTERACTIVE_HEADERDATA = 15,
	LEN_KEYBOARD_INFORMATION = 12, // 0x0304

	//		LEN_SENTRY_INFO                          =24,
}; // 表2-4
/*******************裁判系统信息内容排序**************************/

typedef __packed struct
{
	uint8_t sof;
	uint16_t data_length;
	uint8_t seq;
	uint8_t crc8;
} std_frame_header_t; // LEN_FRAME_HEAD

typedef enum
{
	GAME_NOT_START = 0, // 比赛未开始
	GAME_IN_READY,		// 比赛准备阶段
	GAME_IN_CHECKING,	// 裁判系统自检阶段
	GAME_IN_COUNTDOWN,	// 5秒倒计时
	GAME_IN_PROGRESS,	// 比赛进行中
	GAME_IN_ENDING,		// 比赛结算
} game_progress_e;

/**
 * @brief 比赛状态
 *
 */
typedef __packed struct
{
	uint8_t game_type : 4;
	uint8_t game_progress : 4;
	uint16_t stage_remain_time;
	uint64_t SyncTimeStamp;
} ext_game_status_t;

/* ID: 0X0201  Byte: 27    机器人状态数据 */
typedef __packed struct
{
	uint8_t robot_id;
	uint8_t robot_level;
	uint16_t current_HP;
	uint16_t maximum_HP;
	uint16_t shooter_barrel_cooling_value;
	uint16_t shooter_barrel_heat_limit;
	uint16_t chassis_power_limit;
	float bullet_speed_limit; 
	uint8_t power_management_gimbal_output : 1;
	uint8_t power_management_chassis_output : 1;
	uint8_t power_management_shooter_output : 1;
} robot_status_t;

typedef enum
{
	DART_TARGET_OUTPOST = 0,		 // 飞镖目标为前哨站
	DART_TARGET_BASE_FIXED,			 // 飞镖目标为基地固定目标
	DART_TARGET_BASE_RANDOM_FIXED,	 // 飞镖目标为基地随机固定目标
	DART_TARGET_BASE_RANDOM_MOVING,	 // 飞镖目标为基地随机移动目标
	DART_TARGET_BASE_ENDMOST_MOVING, // 选中基地末端移动目标
} dart_target_type_e;

typedef __packed struct
{
	uint8_t dart_remaining_time;	  // 己方飞镖发射剩余时间，单位：秒
	uint16_t dart_hit_target : 3;	  // 最近一次己方飞镖击中的目标，开局默认为 0，1 为击中前哨站，2 为击中基地固定目标，3 为击中基地随机固定目标，4 为击中基地随机移动目标 , 5为基地末端移动目标
	uint16_t dart_hit_times : 3;	  // 对方最近被击中的目标累计被击中计次数，开局默认为 0，至多为 4
	uint16_t dart_current_target : 3; // 飞镖此时选定的击打目标，开局默认或未选定/选定前哨站时为 0，选中基地固定目标为 1，选中基地随机固定目标为 2，选中基地随机移动目标为 3 ,4为基地末端移动目标
	uint16_t reserved : 7;			  // 保留位
} dart_info_t;

/**
 * @brief 飞镖发射站状态
 *
 */
typedef enum
{
	DART_LAUNCH_ON = 0, // 飞镖发射站开启
	DART_LAUNCH_OFF,	// 飞镖发射站关闭
	DART_LAUNCH_MOVING, // 飞镖发射站开启中或关闭中
} dart_launch_opening_status_e;

/**
 * @brief 飞镖选手端指令数据，固定以3Hz 频率发送 0x020A
 *
 */
typedef __packed struct
{
	uint8_t dart_launch_opening_status; // 当前飞镖发射站的状态：0:开启 1:关闭 2:开启或关闭中
	uint8_t reserved;
	uint16_t target_change_time;	 // 切换击打目标时的比赛剩余时间，单位：秒，无/未切换动作，默认为 0
	uint16_t latest_launch_cmd_time; // 最后一次操作手确定发射指令时的比赛剩余时间，单位：秒，初始值为 0
} dart_client_cmd_t;

/*******************************************************************************/
/*
	机器人 ID：
	1，英雄(红)；
	2，工程(红)；
	3/4/5，步兵(红)；
	6，空中(红)；
	7，哨兵(红)；
	9，雷达（红）
	101，英雄(蓝)；
	102，工程(蓝)；
	103/104/105，步兵(蓝)；
	106，空中(蓝)；
	107，哨兵(蓝)；
	109，雷达（蓝）

	客户端 ID：
	0x0101 为英雄操作手客户端(红) ；
	0x0102 为工程操作手客户端( 红 )；
	0x0103/0x0104/0x0105 为步兵操作手客户端(红)；
	0x0106 为空中操作手客户端((红)；

	0x0165，英雄操作手客户端(蓝)；
	0x0166，工程操作手客户端(蓝)；
	0x0167/0x0168/0x0169，步兵操作手客户端(蓝)；
	0x016A，空中操作手客户端(蓝)。
*/
enum judge_robot_ID
{
	hero_red = 1,
	engineer_red = 2,
	infantry3_red = 3,
	infantry4_red = 4,
	infantry5_red = 5,
	plane_red = 6,

	hero_blue = 101,
	engineer_blue = 102,
	infantry3_blue = 103,
	infantry4_blue = 104,
	infantry5_blue = 105,
	plane_blue = 106,
};
typedef struct
{
	uint16_t teammate_hero;
	uint16_t teammate_engineer;
	uint16_t teammate_infantry3;
	uint16_t teammate_infantry4;
	uint16_t teammate_infantry5;
	uint16_t teammate_plane;
	uint16_t teammate_sentry;

	uint16_t client_hero;
	uint16_t client_engineer;
	uint16_t client_infantry3;
	uint16_t client_infantry4;
	uint16_t client_infantry5;
	uint16_t client_plane;
} ext_interact_id_t;

typedef struct
{
	std_frame_header_t fream_header; // 帧头信息

	ext_game_status_t ext_game_status; // 0x001
	robot_status_t game_robot_status;  // 0x0201
	dart_info_t dart_info;			   // 0x0105
	dart_client_cmd_t dart_client_cmd; // 0x020A

	ext_interact_id_t ids; // 与本机交互的机器人id

	uint16_t self_client; // 本机客户端

	bool data_valid; // 数据有效性

	dev_work_state_t judge_state;

	uint8_t offline_cnt;
	uint8_t offline_max_cnt;

} judge_t;

extern judge_t judge;

void Judge_HeartBeat(judge_t *judge);
void Judge_Update(judge_t *judge, uint8_t *rxBuf);

#endif
