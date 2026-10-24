"""
裁判系统模拟器
模拟RoboMaster比赛中的裁判系统数据发送
"""

import time
import random
import argparse
import threading
from enum import Enum
from typing import Optional
from dataclasses import dataclass, field

from judge_send import *
from serial_comm import SerialComm, list_available_ports


class GamePhase(Enum):
    """比赛阶段枚举"""
    STANDBY = 0      # 未开始
    PREPARATION = 1   # 准备阶段 180s
    SELF_CHECK = 2   # 裁判系统自检 15s  
    COUNTDOWN = 3    # 五秒倒计时 5s
    IN_GAME = 4      # 比赛中 420s
    SETTLEMENT = 5   # 比赛结算 15s


class DartPhase(Enum):
    """飞镖发射状态枚举"""
    CLOSED = 0       # 关闭状态
    OPENING = 1      # 正在开启
    OPENED = 2       # 已开启，等待发射
    COUNTDOWN = 3    # 发射倒计时
    CLOSING = 4      # 正在关闭
    NOT_MANAGED = 5  # 未管理状态


@dataclass
class GameState:
    """比赛状态数据"""
    # 比赛基本信息
    game_type: int = 1
    game_progress: int = 0
    stage_remain_time: int = 0
    
    # 时间相关
    total_game_seconds: int = 0
    last_update_time: float = field(default_factory=time.time)
    game_start_time: float = field(default_factory=time.time)
    
    # 飞镖相关
    dart_opening_status: int = 1  # 1-关闭
    dart_remaining_time: int = 0
    target_change_time: int = 0
    latest_launch_cmd_time: int = 0
    last_hit_target: int = 0
    opponent_hit_count: int = 0
    selected_target: int = 0
    
    # 事件标记
    first_dart_event_triggered: bool = False
    second_dart_event_triggered: bool = False
    sent_launch_cmd: bool = False
    # 第一次发射前的暂停标志（用于在第一次发射倒计时前等待用户继续）
    first_pause_triggered: bool = False


class JudgeSystemSimulator:
    """Judge System Simulator"""
    
    def __init__(self, port: Optional[str] = None, baudrate: int = 115200, 
                 send_interval_ms: int = 100, dart_target_type: int = 0, dart_event_count: int = 2):
        """
        Initialize simulator
        
        Args:
            port: Serial port device (auto-detect if None, Windows: COM1, COM2, COM3... Linux: /dev/ttyUSB0...)
            baudrate: Baud rate
            send_interval_ms: Send interval (milliseconds)
            dart_target_type: Dart target type (0: Outpost, 1: Base fixed target, 2: Base random target, 3: Base random moving target)
            dart_event_count: Number of dart events (1 or 2)
        """
        self.serial_comm = SerialComm(port, baudrate)
        self.send_interval_ms = send_interval_ms
        self.dart_target_type = dart_target_type
        self.dart_event_count = dart_event_count
        
        # State machine
        self.game_phase = GamePhase.STANDBY
        self.dart_phase = DartPhase.CLOSED
        self.dart_phase_start_time = time.time()
        
        # Game state
        self.state = GameState()
        
        # Control variables
        self.running = False
        self.thread: Optional[threading.Thread] = None
        self.paused = False  # 新增暂停标志
        self.waiting_for_continue = False  # 等待继续标志
        self.waiting_for_continue_reason: Optional[str] = None  # 等待继续原因
        
        # 3Hz send tracking
        self.last_3hz_send_time = 0.0
    
    def update_dart_event_count(self, dart_event_count: int):
        """
        更新飞镖事件次数
        
        Args:
            dart_event_count: 飞镖事件次数 (1 或 2)
        """
        self.dart_event_count = dart_event_count
        print(f"飞镖事件次数已更新为: {dart_event_count}")
    
    def continue_simulation(self):
        """
        继续模拟（用于两次飞镖事件的中间继续）
        """
        if self.waiting_for_continue:
            continue_reason = self.waiting_for_continue_reason
            self.waiting_for_continue = False
            self.paused = False
            if continue_reason == "before_first_dart":
                print("继续模拟第一次飞镖事件...")
            elif continue_reason in ("before_second_dart", "after_first_dart_close"):
                self.dart_phase = DartPhase.OPENING
                self.dart_phase_start_time = time.time()
                self.state.second_dart_event_triggered = True
                print("继续模拟第二次飞镖事件...")
                print("飞镖舱门开始打开...")
                self.waiting_for_continue_reason = None
                return
            else:
                print("继续模拟...")
            self.waiting_for_continue_reason = None
    
    def is_waiting_for_continue(self) -> bool:
        """
        检查是否在等待继续状态
        
        Returns:
            bool: 是否在等待继续
        """
        return self.waiting_for_continue
    
    def reset_game_state(self):
        """重置游戏状态以准备新的一轮比赛"""
        self.game_phase = GamePhase.STANDBY
        self.dart_phase = DartPhase.CLOSED
        self.dart_phase_start_time = time.time()
        
        # 重置游戏状态
        self.state = GameState()
        self.state.last_update_time = time.time()
        self.state.game_start_time = time.time()
        
        # 重置3Hz发送时间
        self.last_3hz_send_time = 0.0
        
        # 重置暂停状态
        self.paused = False
        self.waiting_for_continue = False
        self.waiting_for_continue_reason = None
        
        print("游戏状态已重置")
    
    def generate_dart_info(self) -> int:
        """
        生成飞镖信息位域
        
        Returns:
            int: 飞镖信息位域值
        """
        dart_info = 0
        dart_info |= (self.state.last_hit_target & 0x07)  # bit 0-2
        dart_info |= ((self.state.opponent_hit_count & 0x07) << 3)  # bit 3-5
        dart_info |= ((self.state.selected_target & 0x03) << 6)  # bit 6-7
        return dart_info
    
    def get_current_timestamp_us(self) -> int:
        """获取当前时间戳（微秒）"""
        return int(time.time() * 1_000_000)
    
    def print_game_status(self):
        """打印当前比赛状态"""
        status_map = {
            0: "未开始比赛",
            1: "准备阶段", 
            2: "裁判系统自检阶段",
            3: "五秒倒计时",
            4: "比赛中",
            5: "比赛结算中"
        }
        
        print("=" * 40)
        print(f"当前比赛状态: {status_map.get(self.state.game_progress, '未知状态')} | "
              f"剩余时间: {self.state.stage_remain_time}秒")
        print("=" * 40)
    
    def print_dart_status(self):
        """打印飞镖状态"""
        status_map = {
            0: "已经开启",
            1: "关闭", 
            2: "正在开启或者关闭中"
        }
        
        print(f"飞镖发射口状态: {status_map.get(self.state.dart_opening_status, '未知状态')}", end="")
        
        if self.state.dart_remaining_time > 0:
            print(f" | 倒计时: {self.state.dart_remaining_time}秒")
        else:
            print()
        
        print(f"最新发射指令时间: {self.state.latest_launch_cmd_time}秒")
        print(f"飞镖选定的击打目标: {self.state.selected_target}")
    
    def update_phase_flags(self):
        """更新比赛阶段和飞镖阶段的状态标志位"""
        # 更新比赛阶段标志位
        if self.game_phase == GamePhase.STANDBY:
            self.dart_phase = DartPhase.NOT_MANAGED
            # 开始比赛
            self.game_phase = GamePhase.PREPARATION
            print("比赛开始，进入准备阶段!")
            
        elif self.game_phase == GamePhase.PREPARATION:
            if self.state.stage_remain_time == 0:
                self.game_phase = GamePhase.SELF_CHECK
                print("准备阶段结束，进入裁判系统自检阶段!")
                
        elif self.game_phase == GamePhase.SELF_CHECK:
            if self.state.stage_remain_time == 0:
                self.game_phase = GamePhase.COUNTDOWN
                self.state.stage_remain_time = 1
                self.waiting_for_continue = True
                self.waiting_for_continue_reason = "before_first_dart"
                self.paused = True
                print("裁判系统自检完成，进入五秒倒计时!")
                print("第一次飞镖事件前等待继续... 请点击 'Continue' 按钮继续模拟")
                
        elif self.game_phase == GamePhase.COUNTDOWN:
            if self.state.stage_remain_time == 0:
                self.game_phase = GamePhase.IN_GAME
                self.state.game_start_time = time.time()
                print("比赛正式开始!")
                
                # 重置飞镖相关状态
                self.state.first_dart_event_triggered = False
                self.state.second_dart_event_triggered = False
                self.state.last_hit_target = 0
                self.state.opponent_hit_count = 0
                self.state.selected_target = 0
                self.dart_phase = DartPhase.CLOSED
                # 重置第一次暂停/等待标志，确保第一次事件能正确触发暂停
                self.state.first_pause_triggered = False
                
        elif self.game_phase == GamePhase.IN_GAME:
            # 飞镖机器人在比赛进行1s和35s时触发发射事件
            if not self.state.first_dart_event_triggered and self.state.total_game_seconds >= 1:
                self.dart_phase = DartPhase.OPENING
                self.dart_phase_start_time = time.time()
                self.state.first_dart_event_triggered = True
                print("\n飞镖事件触发! (2秒标记)")
                print("飞镖舱门开始打开...")
                
                # 如果只需要一个飞镖事件，调整比赛剩余时间到合适值（约30秒让第一个事件完成）
                if self.dart_event_count == 1 and self.state.stage_remain_time > 35:
                    self.state.stage_remain_time = 35
                    print(f"调整比赛时间到 {self.state.stage_remain_time} 秒以完成单次飞镖事件")
                
            if self.state.stage_remain_time == 0:
                self.game_phase = GamePhase.SETTLEMENT
                self.dart_phase = DartPhase.CLOSED
                print("比赛结束!")
                
        elif self.game_phase == GamePhase.SETTLEMENT:
            if self.state.stage_remain_time == 0:
                # 发送比赛结果（简化为随机胜负）
                winner = random.randint(1, 2)  # 随机选择红方或蓝方胜利
                result_data = ExtGameResult()
                result_data.winner = winner
                self.serial_comm.send_data(0x0002, result_data.pack())
                print(f"比赛结束! {'红方胜利!' if winner == 1 else '蓝方胜利!'}")
                # 重新开始比赛
                self.game_phase = GamePhase.STANDBY
                self.running = False
                return
        
        # 更新飞镖阶段标志位
        dart_elapsed = time.time() - self.dart_phase_start_time
        
        if self.dart_phase == DartPhase.OPENING:
            if dart_elapsed >= 5.5:  # 5秒后开启完成
                self.dart_phase = DartPhase.OPENED
                self.dart_phase_start_time = time.time()
                print("飞镖舱门已完全打开!")
                
        elif self.dart_phase == DartPhase.OPENED:
            self.dart_phase = DartPhase.COUNTDOWN
            self.dart_phase_start_time = time.time()
            print("飞镖发射倒计时开始: 30秒")
            self.state.sent_launch_cmd = False
            
        elif self.dart_phase == DartPhase.COUNTDOWN:
            if dart_elapsed >= 30:  # 倒计时结束
                self.dart_phase = DartPhase.CLOSING
                self.dart_phase_start_time = time.time()
                print("飞镖发射倒计时结束，舱门开始关闭")
                
        elif self.dart_phase == DartPhase.CLOSING:
            if dart_elapsed >= 2:  # 2秒后关闭完成
                self.dart_phase = DartPhase.CLOSED
                self.dart_phase_start_time = time.time()
                print("飞镖舱门已完全关闭!")
                
                # 检查是否需要在第一次飞镖事件完成后暂停
                if (self.dart_event_count == 2 and 
                    self.state.first_dart_event_triggered and 
                    not self.state.second_dart_event_triggered):
                    self.waiting_for_continue = True
                    self.waiting_for_continue_reason = "after_first_dart_close"
                    self.paused = True
                    print("\n第一次飞镖事件完成! 等待继续执行第二次事件...")
                    print("请点击 'Continue' 按钮继续模拟")

    def update_data_values(self):
        """根据阶段标志位更新发送信息的对应数值"""
        # 根据比赛阶段更新数值
        if self.game_phase == GamePhase.STANDBY:
            self.state.game_progress = 0
            self.state.stage_remain_time = 0
            
        elif self.game_phase == GamePhase.PREPARATION:
            self.state.game_progress = 1
            if self.state.stage_remain_time == 0:
                self.state.stage_remain_time = 1  # 准备阶段1秒（测试用）

        elif self.game_phase == GamePhase.SELF_CHECK:
            self.state.game_progress = 2
            if self.state.stage_remain_time == 0:
                self.state.stage_remain_time = 5  # 自检阶段5秒

        elif self.game_phase == GamePhase.COUNTDOWN:
            self.state.game_progress = 3
            if self.state.stage_remain_time == 0:
                self.state.stage_remain_time = 1  # 倒计时1秒

        elif self.game_phase == GamePhase.IN_GAME:
            self.state.game_progress = 4
            if self.state.stage_remain_time == 0:
                # 根据飞镖事件次数动态设置比赛时间
                if self.dart_event_count == 1:
                    self.state.stage_remain_time = 45  # 单次事件：更宽松的单轮时间
                else:
                    self.state.stage_remain_time = 85  # 双次事件：更宽松的双轮时间
                
            # 比赛开始第1秒前哨站被击毁（如果设置了其他目标）
            if self.state.total_game_seconds == 1 and self.dart_target_type != 0:
                self.state.selected_target = self.dart_target_type
                
        elif self.game_phase == GamePhase.SETTLEMENT:
            self.state.game_progress = 5
            if self.state.stage_remain_time == 0:
                self.state.stage_remain_time = 2
        
        # 根据飞镖阶段更新数值
        dart_elapsed = time.time() - self.dart_phase_start_time
        
        if self.dart_phase == DartPhase.CLOSED:
            self.state.dart_opening_status = 1
            self.state.dart_remaining_time = 0
            
        elif self.dart_phase == DartPhase.OPENING:
            self.state.dart_opening_status = 2
            
        elif self.dart_phase == DartPhase.OPENED:
            self.state.dart_opening_status = 0
            
        elif self.dart_phase == DartPhase.COUNTDOWN:
            self.state.dart_opening_status = 0
            self.state.dart_remaining_time = max(0, 30 - int(dart_elapsed))

            # 发射指令（保持原逻辑在倒计时开始后1秒触发）
            if self.state.dart_remaining_time == 29 and not self.state.sent_launch_cmd:
                self.state.sent_launch_cmd = True
                self.state.latest_launch_cmd_time = self.state.stage_remain_time
                
        elif self.dart_phase == DartPhase.CLOSING:
            self.state.dart_opening_status = 2
            self.state.dart_remaining_time = 0
            
        elif self.dart_phase == DartPhase.NOT_MANAGED:
            # 未管理状态的特殊处理
            if self.game_phase == GamePhase.PREPARATION:
                self.state.dart_opening_status = 1
            elif self.game_phase == GamePhase.SELF_CHECK:
                if self.state.stage_remain_time == 2:
                    self.state.dart_opening_status = 0  # 开启飞镖发射口
                elif self.state.stage_remain_time == 1:
                    self.state.dart_opening_status = 2  # 准备关闭飞镖发射口
                else:
                    self.state.dart_opening_status = 2  # 准备开启飞镖发射口
            elif self.game_phase == GamePhase.COUNTDOWN:
                if self.state.stage_remain_time == 1:
                    self.state.dart_opening_status = 1  # 关闭飞镖发射口
                    
        # 比赛结束时的特殊清理
        if self.game_phase == GamePhase.SETTLEMENT and self.state.stage_remain_time == 0:
            self.state.dart_remaining_time = 0
            self.state.latest_launch_cmd_time = 0
            self.state.selected_target = 0
            self.state.dart_opening_status = 1
    
    def send_periodic_data(self):
        """发送周期性数据"""
        current_time = time.time()
        
        # 1Hz 发送比赛状态数据
        game_status = ExtGameStatus()
        game_status.game_type = self.state.game_type
        game_status.game_progress = self.state.game_progress
        game_status.stage_remain_time = self.state.stage_remain_time
        game_status.sync_time_stamp = self.get_current_timestamp_us()
        self.serial_comm.send_data(0x0001, game_status.pack())
        
        # 1Hz 发送飞镖相关数据
        dart_info = self.generate_dart_info()
        dart_info_data = ExtDartInfo()
        dart_info_data.dart_remaining_time = self.state.dart_remaining_time
        dart_info_data.dart_info = dart_info
        self.serial_comm.send_data(0x0105, dart_info_data.pack())
        
        # 3Hz 发送飞镖客户端指令数据 (每333ms发送一次)
        if current_time - self.last_3hz_send_time >= 1.0/3.0:  # 333ms
            dart_cmd_data = ExtDartClientCmd()
            dart_cmd_data.dart_launch_opening_status = self.state.dart_opening_status
            dart_cmd_data.reserved = 0
            dart_cmd_data.target_change_time = self.state.target_change_time
            dart_cmd_data.latest_launch_cmd_time = self.state.latest_launch_cmd_time
            self.serial_comm.send_data(0x020A, dart_cmd_data.pack())
            self.last_3hz_send_time = current_time
    
    def update_loop(self):
        """主更新循环"""
        ms_accumulated = 0
        
        while self.running:
            current_time = time.time()
            elapsed_ms = int((current_time - self.state.last_update_time) * 1000)
            
            if elapsed_ms >= self.send_interval_ms:
                self.state.last_update_time = current_time
                # 暂停时冻结比赛时间和飞镖状态，避免第一次暂停期间自己跑完
                if not self.paused and self.game_phase != GamePhase.STANDBY:
                    ms_accumulated += elapsed_ms
                    while ms_accumulated >= 1000 and self.state.stage_remain_time > 0:
                        self.state.stage_remain_time -= 1
                        ms_accumulated -= 1000

                    if self.game_phase == GamePhase.IN_GAME:
                        self.state.total_game_seconds = int(current_time - self.state.game_start_time)

                # 只有在非暂停状态下才更新状态机与飞镖相关数值
                if not self.paused:
                    self.update_phase_flags()
                    self.update_data_values()
                
                # 无论是否暂停都发送数据（保持串口通信）
                self.send_periodic_data()
                
                # 打印状态
                self.print_game_status()
                self.print_dart_status()
            
            time.sleep(0.01)  # 避免CPU占用过高
    
    def start(self):
        """Start simulator"""
        # Check if serial port is already connected
        if not self.serial_comm.is_connected():
            # Try to connect if not already connected
            if not self.serial_comm.connect():
                raise RuntimeError("Cannot connect to serial port")
        
        self.running = True
        self.thread = threading.Thread(target=self.update_loop, daemon=True)
        self.thread.start()
        
        print("Judge system simulator started")
    
    def stop(self):
        """Stop simulator"""
        self.running = False
        if self.thread:
            self.thread.join()
        self.serial_comm.disconnect()
        print("裁判系统模拟器已停止")
