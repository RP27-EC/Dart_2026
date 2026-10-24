#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
简化版裁判系统数据结构定义
只包含飞镖模拟器所需的数据结构
"""

import struct
from typing import List, Union

# --------------CmdID(2-Byte)----------------
GAME_STATUS = 0x0001              # 比赛状态数据，1Hz 周期发送
GAME_RESULT = 0x0002              # 比赛结果数据，比赛结束后发送
DART_INFO = 0x0105               # 飞镖发射口倒计时，1Hz 周期发送
DART_CLIENT_CMD = 0x020A         # 飞镖机器人客户端指令数据，3Hz周期发送

# --------------DataSize----------------
GAME_STATUS_DATA_SIZE = 11              # 0x0001
GAME_RESULT_DATA_SIZE = 1               # 0x0002
DART_INFO_DATA_SIZE = 3                 # 0x0105
DART_CLIENT_CMD_DATA_SIZE = 6           # 0x020A


class JudgeDataStructBase:
    """裁判系统数据结构基类"""
    
    def pack(self) -> bytes:
        """将数据结构打包为字节序列"""
        raise NotImplementedError
    
    def unpack(self, data: bytes) -> None:
        """从字节序列解包数据结构"""
        raise NotImplementedError
    
    def size(self) -> int:
        """返回数据结构的字节大小"""
        raise NotImplementedError


class ExtGameStatus(JudgeDataStructBase):
    """1. 比赛机器人状态(0x0001) 1Hz"""
    
    def __init__(self):
        self.game_type = 0          # 0-3 bit：比赛类型
        self.game_progress = 0      # 4-7 bit：当前比赛阶段
        self.stage_remain_time = 0  # 当前阶段剩余时间，单位 s
        self.sync_time_stamp = 0    # 机器人接收到该指令的精确Unix时间
    
    def pack(self) -> bytes:
        # 将位域合并为一个字节
        game_status_byte = (self.game_type & 0x0F) | ((self.game_progress & 0x0F) << 4)
        return struct.pack('<BHQ', game_status_byte, self.stage_remain_time, self.sync_time_stamp)
    
    def unpack(self, data: bytes) -> None:
        game_status_byte, self.stage_remain_time, self.sync_time_stamp = struct.unpack('<BHQ', data)
        self.game_type = game_status_byte & 0x0F
        self.game_progress = (game_status_byte >> 4) & 0x0F
    
    def size(self) -> int:
        return 11


class ExtGameResult(JudgeDataStructBase):
    """2. 比赛结果数据：0x0002。发送频率：比赛结束后发送"""
    
    def __init__(self):
        self.winner = 0  # 0 平局 1 红方胜利 2 蓝方胜利
    
    def pack(self) -> bytes:
        return struct.pack('<B', self.winner)
    
    def unpack(self, data: bytes) -> None:
        self.winner, = struct.unpack('<B', data)
    
    def size(self) -> int:
        return 1


class ExtDartInfo(JudgeDataStructBase):
    """8. 飞镖状态信息：cmd_id (0x0105)"""
    
    def __init__(self):
        self.dart_remaining_time = 0  # 15s 倒计时
        self.dart_info = 0           # 位域信息
    
    def pack(self) -> bytes:
        return struct.pack('<BH', self.dart_remaining_time, self.dart_info)
    
    def unpack(self, data: bytes) -> None:
        self.dart_remaining_time, self.dart_info = struct.unpack('<BH', data)
    
    def size(self) -> int:
        return 3


class ExtDartClientCmd(JudgeDataStructBase):
    """18. 飞镖机器人客户端指令数据：0x020A。发送频率：3Hz"""
    
    def __init__(self):
        self.dart_launch_opening_status = 0  # 当前飞镖发射站的状态
        self.reserved = 0                    # 保留
        self.target_change_time = 0          # 切换击打目标时的比赛剩余时间
        self.latest_launch_cmd_time = 0      # 最后一次操作手确定发射指令时的比赛剩余时间
    
    def pack(self) -> bytes:
        return struct.pack('<2B2H', self.dart_launch_opening_status, self.reserved,
                          self.target_change_time, self.latest_launch_cmd_time)
    
    def unpack(self, data: bytes) -> None:
        (self.dart_launch_opening_status, self.reserved,
         self.target_change_time, self.latest_launch_cmd_time) = struct.unpack('<2B2H', data)
    
    def size(self) -> int:
        return 6


def get_data_struct_by_cmd_id(cmd_id: int) -> JudgeDataStructBase:
    """根据命令ID获取对应的数据结构实例"""
    struct_map = {
        GAME_STATUS: ExtGameStatus(),
        GAME_RESULT: ExtGameResult(),
        DART_INFO: ExtDartInfo(),
        DART_CLIENT_CMD: ExtDartClientCmd(),
    }
    return struct_map.get(cmd_id)


if __name__ == "__main__":
    # 测试代码
    print("裁判系统数据结构 Python 版本 (简化版)")
    print(f"游戏状态数据大小: {ExtGameStatus().size()} bytes")
    print(f"比赛结果数据大小: {ExtGameResult().size()} bytes")
    print(f"飞镖信息数据大小: {ExtDartInfo().size()} bytes")
    print(f"飞镖客户端指令数据大小: {ExtDartClientCmd().size()} bytes")
    
    # 测试打包和解包
    status = ExtGameStatus()
    status.game_type = 1
    status.game_progress = 4
    status.stage_remain_time = 300
    status.sync_time_stamp = 1642780800
    
    # 打包
    packed_data = status.pack()
    print(f"打包后数据: {packed_data.hex()}")
    
    # 解包
    new_status = ExtGameStatus()
    new_status.unpack(packed_data)
    print(f"解包后数据: game_type={new_status.game_type}, game_progress={new_status.game_progress}")
