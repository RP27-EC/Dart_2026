"""
Judge System Simulator GUI Interface - Corrected Version
Using DearPyGui to create a graphical user interface, implementing the same functionality as the command-line main function
"""

import dearpygui.dearpygui as dpg
import threading
import time
import sys
import os
from typing import Optional

# Add src directory to Python path to ensure correct module imports
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from simulator import JudgeSystemSimulator
from serial_comm import list_available_ports


class JudgeSimulatorGUI:
    """Judge System Simulator GUI Interface"""
    
    def __init__(self):
        self.simulator: Optional[JudgeSystemSimulator] = None
        self.is_connected = False
        self.is_running = False
        self.is_waiting_for_continue = False  # 新增：等待继续状态
        self.available_ports = []
        self.port_display_list = []  # Store display texts for combo
        
        # GUI component references
        self.port_combo = None
        self.connect_button = None
        self.start_auto_button = None
        self.status_text = None
        self.dart_target_combo = None
        self.dart_event_count_combo = None  # 新增飞镖事件次数选择
        self.interval_input = None
        
        # Game status display components
        self.game_status_text = None
        self.remaining_time_text = None
        self.dart_status_text = None
        self.game_progress_bar = None
        self.game_progress_text = None  # 新增：进度文本显示
        self.dart_progress_bar = None
        self.loading_indicator = None  # 新增：加载指示器
        
        self.setup_gui()
    
    def refresh_ports(self):
        """Refresh available serial port list"""
        try:
            # Get detailed port information
            try:
                import serial.tools.list_ports
                ports = serial.tools.list_ports.comports()
            except ImportError:
                ports = []
            
            port_info_list = []
            port_name_list = []
            
            for port in ports:
                port_name = port.device
                description = port.description
                # Create display text with port name and description
                display_text = f"{port_name} - {description}"
                port_info_list.append(display_text)
                port_name_list.append(port_name)  # Keep original port name for connection
            
            self.available_ports = port_name_list  # Store original port names for connection
            self.port_display_list = port_info_list  # Store display texts
            
            if self.port_combo:
                if port_info_list:
                    dpg.configure_item(self.port_combo, items=port_info_list)
                    dpg.set_value(self.port_combo, port_info_list[0])
                else:
                    dpg.configure_item(self.port_combo, items=["No ports available"])
                    dpg.set_value(self.port_combo, "No ports available")
                    
        except Exception as e:
            self.log_message(f"Failed to refresh ports: {e}")
            # Fallback to basic port detection
            try:
                ports = list_available_ports(return_list=True)
                self.available_ports = ports if ports else []
                self.port_display_list = self.available_ports
                
                if self.port_combo:
                    if self.available_ports:
                        dpg.configure_item(self.port_combo, items=self.available_ports)
                        dpg.set_value(self.port_combo, self.available_ports[0])
                    else:
                        dpg.configure_item(self.port_combo, items=["No ports available"])
                        dpg.set_value(self.port_combo, "No ports available")
            except Exception as fallback_error:
                self.log_message(f"Fallback port detection also failed: {fallback_error}")
    
    def log_message(self, message: str):
        """Add log message - only to console"""
        timestamp = time.strftime("%H:%M:%S")
        log_text = f"[{timestamp}] {message}"
        print(log_text)  # Only output to console
    
    def update_connection_status(self, connected: bool):
        """Update connection status text and color"""
        if self.status_text:
            if connected:
                # Green color for connected
                dpg.set_value(self.status_text, "Connected")
                dpg.configure_item(self.status_text, color=(0, 255, 0))  # Green
            else:
                # Red color for disconnected
                dpg.set_value(self.status_text, "Disconnected")
                dpg.configure_item(self.status_text, color=(255, 0, 0))  # Red
    
    def update_game_status_display(self):
        """Update game status display in GUI"""
        if not self.simulator or not self.game_status_text or not self.remaining_time_text:
            return
            
        try:
            status_map = {
                0: "Game Not Started",
                1: "Preparation Phase", 
                2: "Judge System Self-Check Phase",
                3: "Five Second Countdown",
                4: "In Game",
                5: "Game Settlement"
            }
            
            dart_status_map = {
                0: "Already Open",
                1: "Closed", 
                2: "Opening or Closing"
            }
            
            # Update game status text (separated)
            game_status = status_map.get(self.simulator.state.game_progress, 'Unknown Status')
            game_text = f"Current Game Status: {game_status}"
            dpg.set_value(self.game_status_text, game_text)
            
            # Update remaining time text (separated)
            time_text = f"Remaining Time: {self.simulator.state.stage_remain_time}s"
            dpg.set_value(self.remaining_time_text, time_text)
            
            # Update dart status text
            dart_status = dart_status_map.get(self.simulator.state.dart_opening_status, 'Unknown Status')
            dart_text = f"Dart Launch Port Status: {dart_status}"
            if self.simulator.state.dart_remaining_time > 0:
                dart_text += f" | Countdown: {self.simulator.state.dart_remaining_time}s"
            dart_text += f"\nLatest Launch Command Time: {self.simulator.state.latest_launch_cmd_time}s"
            dart_text += f"\nDart Selected Target: {self.simulator.state.selected_target}"
            dpg.set_value(self.dart_status_text, dart_text)
            
            # Update progress bars
            if self.game_progress_bar:
                # Game progress bar based on stage remaining time with dynamic max values
                max_time = 1  # 默认最大时间
                
                if self.simulator.state.game_progress == 0:  # 未开始
                    max_time = 1
                elif self.simulator.state.game_progress == 1:  # 准备阶段
                    max_time = 1
                elif self.simulator.state.game_progress == 2:  # 自检阶段
                    max_time = 1
                elif self.simulator.state.game_progress == 3:  # 倒计时阶段
                    max_time = 3
                elif self.simulator.state.game_progress == 4:  # 比赛中
                    # 根据飞镖事件次数动态设置最大时间
                    max_time = 35 if self.simulator.dart_event_count == 1 else 60
                elif self.simulator.state.game_progress == 5:  # 比赛结算
                    max_time = 2
                
                if max_time > 0:
                    progress = max(0, min(1.0, self.simulator.state.stage_remain_time / max_time))
                    dpg.set_value(self.game_progress_bar, progress)
                    
                    # 更新进度条的显示信息，显示当前时间/最大时间
                    if self.game_progress_text:
                        progress_text = f"Game Progress: {self.simulator.state.stage_remain_time}s / {max_time}s"
                        dpg.set_value(self.game_progress_text, progress_text)
                else:
                    # 当没有时间信息时，设置为0
                    dpg.set_value(self.game_progress_bar, 0.0)
                    if self.game_progress_text:
                        dpg.set_value(self.game_progress_text, "Game Progress: 0s / 1s")
            
            if self.dart_progress_bar:
                # Dart progress bar based on dart remaining time
                if self.simulator.state.dart_remaining_time > 0:
                    progress = max(0, min(1.0, self.simulator.state.dart_remaining_time / 20.0))  # 20 seconds max
                    dpg.set_value(self.dart_progress_bar, progress)
                else:
                    dpg.set_value(self.dart_progress_bar, 0.0)
                    
        except Exception as e:
            print(f"Error updating status display: {e}")
    
    def connect_serial(self):
        """Connect/disconnect serial port"""
        try:
            if not self.is_connected:
                # Connect serial port
                selected_display = dpg.get_value(self.port_combo)
                dart_target_name = dpg.get_value(self.dart_target_combo)
                interval = dpg.get_value(self.interval_input)
                
                # Extract actual port name from display text
                if selected_display == "No ports available":
                    self.log_message("Error: No available ports found")
                    return
                
                # Find the actual port name corresponding to the display text
                selected_port = None
                if hasattr(self, 'port_display_list') and self.port_display_list:
                    try:
                        display_index = self.port_display_list.index(selected_display)
                        selected_port = self.available_ports[display_index]
                    except (ValueError, IndexError):
                        # Fallback: extract port name from display text (assuming format "PORT - Description")
                        selected_port = selected_display.split(' - ')[0] if ' - ' in selected_display else selected_display
                else:
                    # Fallback: use display text directly
                    selected_port = selected_display
                
                # Map dart target types
                dart_target_map = {
                    "Outpost": 0,
                    "Base Fixed Target": 1, 
                    "Base Random Target": 2,
                    "Base Random Moving Target": 3
                }
                dart_target = dart_target_map.get(dart_target_name, 0)
                
                # Create simulator instance with fixed baudrate
                self.simulator = JudgeSystemSimulator(
                    port=selected_port,
                    baudrate=115200,  # Fixed baudrate
                    send_interval_ms=int(interval),
                    dart_target_type=dart_target,
                    dart_event_count=2  # 默认值，会在开始时动态更新
                )
                
                # Try to connect
                connection_success = False
                try:
                    connection_success = self.simulator.serial_comm.connect()
                except Exception as connect_error:
                    self.log_message(f"Serial connection error: {connect_error}")
                    connection_success = False
                
                if connection_success and self.simulator.serial_comm.is_connected():
                    self.is_connected = True
                    dpg.set_value(self.connect_button, "Disconnect Port")
                    self.update_connection_status(True)  # Update status text to green "Connected"
                    dpg.configure_item(self.start_auto_button, enabled=True)
                    
                    # 设置连接成功的loading指示器状态
                    dpg.set_value("simulation_status_text", "Status: Ready")
                    dpg.configure_item("simulation_status_text", color=(128, 128, 128))  # 灰色表示就绪
                    
                    # Show both display text and actual port for better information
                    port_info = f"{selected_display} (Port: {selected_port})" if selected_display != selected_port else selected_port
                    self.log_message(f"Serial connection successful: {port_info} (Baudrate: 115200)")
                    self.log_message(f"Configuration - Dart Target: {dart_target_name}, Send Interval: {interval}ms")
                else:
                    self.log_message(f"Serial connection failed: {selected_port}")
                    self.log_message("Please check if the port is already in use or try another port")
                    # Clean up failed simulator
                    if self.simulator:
                        try:
                            self.simulator.serial_comm.disconnect()
                        except:
                            pass
                        self.simulator = None
                    
            else:
                # Disconnect serial port
                if self.simulator:
                    if self.is_running:
                        self.stop_auto_send()
                    
                    self.simulator.serial_comm.disconnect()
                    self.simulator = None
                
                self.is_connected = False
                dpg.set_value(self.connect_button, "Connect Port")
                self.update_connection_status(False)  # Update status text to red "Disconnected"
                dpg.configure_item(self.start_auto_button, enabled=False)
                
                # 重置loading指示器状态
                dpg.hide_item(self.loading_indicator)
                dpg.set_value("simulation_status_text", "Status: Disconnected")
                dpg.configure_item("simulation_status_text", color=(255, 0, 0))  # 红色表示断开连接
                
                self.log_message("Serial port disconnected")
                
        except Exception as e:
            error_msg = str(e)
            self.log_message(f"Serial operation failed: {error_msg}")
            
            # Provide helpful error messages based on the error type
            if "PermissionError" in error_msg or "Access is denied" in error_msg or "拒绝访问" in error_msg:
                self.log_message("SOLUTION: Port access denied. Try:")
                self.log_message("1. Close other applications using this port (Arduino IDE, PuTTY, etc.)")
                self.log_message("2. Run this program as Administrator")
                self.log_message("3. Check if the device is properly connected")
            elif "FileNotFoundError" in error_msg or "does not exist" in error_msg:
                self.log_message("SOLUTION: Port not found. Try:")
                self.log_message("1. Check device connection")
                self.log_message("2. Click 'Refresh' to update port list")
            
            # Clean up any partially created simulator
            if hasattr(self, 'simulator') and self.simulator:
                try:
                    self.simulator.serial_comm.disconnect()
                except:
                    pass
                self.simulator = None
            
            # Ensure status shows disconnected state
            self.update_connection_status(False)
            
            import traceback
            traceback.print_exc()
    
    def start_auto_send(self):
        """Start automatic sending/Continue simulation"""
        if not self.simulator or not self.is_connected:
            self.log_message("Error: Serial port not connected")
            return
        
        try:
            # 检查是否是继续模拟（等待继续状态）
            if self.is_waiting_for_continue:
                continue_reason = getattr(self.simulator, "waiting_for_continue_reason", None)
                # 继续模拟
                self.simulator.continue_simulation()
                self.is_waiting_for_continue = False
                dpg.configure_item(self.start_auto_button, label="Start One Round", enabled=False)
                
                # 恢复模拟运行状态显示
                dpg.set_value("simulation_status_text", "Status: Simulation Running...")
                dpg.configure_item("simulation_status_text", color=(0, 255, 0))  # 绿色表示运行中
                
                if continue_reason == "before_first_dart":
                    self.log_message("Continuing first dart event...")
                elif continue_reason in ("before_second_dart", "after_first_dart_close"):
                    self.log_message("Continuing second dart event...")
                else:
                    self.log_message("Continuing simulation...")
                return
            
            # 每次开始前更新飞镖事件次数
            dart_event_count_name = dpg.get_value(self.dart_event_count_combo)
            dart_event_count_map = {
                "1 Event": 1,
                "2 Events": 2
            }
            dart_event_count = dart_event_count_map.get(dart_event_count_name, 2)
            
            # 更新模拟器的飞镖事件次数
            self.simulator.update_dart_event_count(dart_event_count)
            
            # 重置游戏状态以准备新的一轮
            self.simulator.reset_game_state()
            
            self.simulator.start()
            self.is_running = True
            dpg.configure_item(self.start_auto_button, enabled=False)
            
            # 显示loading指示器
            dpg.show_item(self.loading_indicator)
            dpg.set_value("simulation_status_text", "Status: Simulation Running...")
            dpg.configure_item("simulation_status_text", color=(0, 255, 0))  # 绿色表示运行中
            
            self.log_message(f"Automatic sending started with {dart_event_count} dart event(s) (will stop after one round)")
            
            # Start monitoring thread for one round completion
            monitor_thread = threading.Thread(target=self.monitor_round_completion, daemon=True)
            monitor_thread.start()
            
            # Start status update thread
            update_thread = threading.Thread(target=self.status_update_loop, daemon=True)
            update_thread.start()
            
        except Exception as e:
            self.log_message(f"Failed to start automatic sending: {e}")
            import traceback
            traceback.print_exc()
    
    def monitor_round_completion(self):
        """Monitor game round completion"""
        if not self.simulator:
            return
            
        # Wait for the game to actually start
        while (self.is_running and 
               self.simulator and 
               self.simulator.game_phase.value <= 3):  # Wait to enter IN_GAME phase
            time.sleep(0.5)
        
        if not self.is_running or not self.simulator:
            return
        
        self.log_message("Game start detected, monitoring round completion...")
        
        # Wait for game to end (return to STANDBY phase or stop running)
        # 但不在等待继续状态时停止
        while (self.is_running and 
               self.simulator and 
               self.simulator.running and
               self.simulator.game_phase.value != 0):  # Wait to return to STANDBY
            
            # 如果进入等待继续状态，继续监控但不结束
            if self.simulator.is_waiting_for_continue():
                # 等待继续被触发
                while (self.is_running and 
                       self.simulator and 
                       self.simulator.running and
                       self.simulator.is_waiting_for_continue()):
                    time.sleep(0.5)
                
                # 如果不再等待继续，说明已经继续了，继续监控游戏结束
                if not self.is_running or not self.simulator:
                    return
            
            time.sleep(0.5)
        
        if self.is_running and self.simulator:
            self.log_message("One round completed, auto-stopping...")
            self.stop_auto_send()
    
    def status_update_loop(self):
        """Status update loop for GUI"""
        while self.is_running and self.simulator:
            try:
                self.update_game_status_display()
                
                # 检查是否进入等待继续状态
                if (self.simulator.is_waiting_for_continue() and 
                    not self.is_waiting_for_continue):
                    self.is_waiting_for_continue = True
                    dpg.configure_item(self.start_auto_button, label="Continue", enabled=True)
                    
                    # 更新loading指示器状态为等待继续
                    dpg.set_value("simulation_status_text", "Status: Waiting for Continue...")
                    dpg.configure_item("simulation_status_text", color=(255, 255, 0))  # 黄色表示等待
                    
                    continue_reason = getattr(self.simulator, "waiting_for_continue_reason", None)
                    if continue_reason == "before_first_dart":
                        self.log_message("Waiting for continue... Click 'Continue' to start first dart event")
                    elif continue_reason in ("before_second_dart", "after_first_dart_close"):
                        self.log_message("Waiting for continue... Click 'Continue' to start second dart event")
                    else:
                        self.log_message("Waiting for continue... Click 'Continue' to resume simulation")
                
                time.sleep(0.5)  # Update every 500ms
            except Exception as e:
                print(f"Status update error: {e}")
                break
    
    
    def stop_auto_send(self):
        """Stop automatic sending"""
        if self.simulator and self.is_running:
            try:
                self.simulator.stop()
                self.is_running = False
                self.is_waiting_for_continue = False  # 重置等待继续状态
                dpg.configure_item(self.start_auto_button, label="Start One Round", enabled=True)
                
                # 隐藏loading指示器
                dpg.hide_item(self.loading_indicator)
                dpg.set_value("simulation_status_text", "Status: Ready")
                dpg.configure_item("simulation_status_text", color=(128, 128, 128))  # 灰色表示就绪
                
                self.log_message("Automatic sending stopped")
                
            except Exception as e:
                self.log_message(f"Failed to stop automatic sending: {e}")
                import traceback
                traceback.print_exc()
    
    def handle_spacebar(self):
        """处理空格键按下事件"""
        # 只有在按钮启用时才触发
        if dpg.is_item_enabled(self.start_auto_button):
            self.start_auto_send()
    
    def handle_enter(self):
        """处理回车键按下事件"""
        # 只有在按钮启用时才触发
        if dpg.is_item_enabled(self.start_auto_button):
            self.start_auto_send()
    
    def setup_gui(self):
        """Setup GUI interface"""
        try:
            dpg.create_context()
            
            # Set font (optional, if Chinese font files are available)
            # dpg.set_global_font_scale(1.0)
            
            with dpg.window(label="RoboMaster Judge System Simulator", tag="primary_window"):
                dpg.add_separator()
                
                # Serial port configuration area
                with dpg.collapsing_header(label="Serial Configuration", default_open=True):
                    with dpg.group(horizontal=True):
                        dpg.add_text("Port:")
                        self.port_combo = dpg.add_combo(
                            items=["Searching..."], 
                            default_value="Searching...", 
                            width=150
                        )
                        dpg.add_button(label="Refresh", callback=self.refresh_ports)
                    
                    with dpg.group(horizontal=True):
                        self.connect_button = dpg.add_button(
                            label="Connect Port", 
                            callback=self.connect_serial,
                            width=100
                        )
                        # Connection status text with color indication
                        dpg.add_text("Status:")
                        self.status_text = dpg.add_text("Disconnected", color=(255, 0, 0))  # Red for disconnected
                dpg.add_separator()
                
                # Connection status display area
                with dpg.group(horizontal=True):
                    dpg.add_text("Send Interval(ms):")
                    self.interval_input = dpg.add_input_int(
                        default_value=100, 
                        width=80,
                        min_value=10,
                        max_value=10000
                    )
                    
                with dpg.group(horizontal=True):
                    dpg.add_text("Dart Target Type:")
                    self.dart_target_combo = dpg.add_combo(
                        items=["Outpost", "Base Fixed Target", "Base Random Target", "Base Random Moving Target"],
                        default_value="Outpost",
                        width=150
                    )
                
                with dpg.group(horizontal=True):
                    dpg.add_text("Dart Event Count:")
                    self.dart_event_count_combo = dpg.add_combo(
                        items=["1 Event", "2 Events"],
                        default_value="1 Event",
                        width=150
                    )
                
                dpg.add_separator()
                
                # Game status display area
                with dpg.collapsing_header(label="Game Status", default_open=True):
                    self.game_status_text = dpg.add_text("Current Game Status: Game Not Started", color=(255, 255, 255))
                    self.remaining_time_text = dpg.add_text("Remaining Time: 0s", color=(255, 255, 255))

                    
                    # Game progress bar with text
                    self.game_progress_text = dpg.add_text("Game Progress: 0s / 1s", color=(255, 255, 255))
                    self.game_progress_bar = dpg.add_progress_bar(default_value=0.0, width=400)
                
                dpg.add_separator()
                
                # Dart status display area
                with dpg.collapsing_header(label="Dart Status", default_open=True):
                    self.dart_status_text = dpg.add_text("Dart Launch Port Status: Closed\nLatest Launch Command Time: 0s\nDart Selected Target: 0", color=(255, 255, 255))
                    
                    # Dart progress bar
                    dpg.add_text("Dart Countdown Progress:")
                    self.dart_progress_bar = dpg.add_progress_bar(default_value=0.0, width=400)
                # Control button area
                with dpg.collapsing_header(label="Control Operations", default_open=True):
                    
                    # 使用表格布局让按钮适配窗口宽度
                    with dpg.table(header_row=False, borders_innerH=False, borders_outerH=False, 
                                   borders_innerV=False, borders_outerV=False):
                        # 添加单列表格
                        dpg.add_table_column()
                        
                        with dpg.table_row():
                            with dpg.table_cell():
                                self.start_auto_button = dpg.add_button(
                                    label="Start One Round", 
                                    callback=self.start_auto_send,
                                    enabled=False,
                                    width=-1,  # -1 表示填充可用宽度
                                    height=40
                                )
                    
                    # 添加快捷键提示
                    dpg.add_text("Shortcuts: Press Space or Enter to trigger the button", color=(128, 128, 128))
                
                dpg.add_separator()
                
                # Loading indicator area (always at bottom)

                self.loading_indicator = dpg.add_loading_indicator(circle_count=8, speed=1.0, radius=15.0)
                dpg.hide_item(self.loading_indicator)  # 初始时隐藏
                
                # 添加状态文本
                dpg.add_text("Status: Ready", tag="simulation_status_text", color=(128, 128, 128))
                
                dpg.add_separator()
                
                
            
            # 注册键盘处理器
            with dpg.handler_registry():
                dpg.add_key_press_handler(dpg.mvKey_Spacebar, callback=self.handle_spacebar)
                dpg.add_key_press_handler(dpg.mvKey_Return, callback=self.handle_enter)
            
            # Setup viewport
            dpg.create_viewport(title="RoboMaster Judge System Simulator", width=500, height=750, resizable=True)
            dpg.setup_dearpygui()
            dpg.show_viewport()
            dpg.set_primary_window("primary_window", True)
            
            # Initialize port list
            self.refresh_ports()
            self.log_message("GUI initialization completed")
            
        except Exception as e:
            print(f"Failed to setup GUI: {e}")
            import traceback
            traceback.print_exc()
            raise
    
    def run(self):
        """Run graphical user interface"""
        try:
            print("RoboMaster Judge System Simulator - Graphical User Interface")
            print("Full functionality identical to command-line version")
            print("Graphical user interface window has started, please check the GUI for operations")
            print("Press Ctrl+C to exit the program")
            print("-" * 50)
            
            while dpg.is_dearpygui_running():
                dpg.render_dearpygui_frame()
                
        except KeyboardInterrupt:
            print("\nReceived interrupt signal, exiting...")
        except Exception as e:
            print(f"Graphical user interface runtime error: {e}")
            import traceback
            traceback.print_exc()
        finally:
            # Clean up resources
            print("Cleaning up resources...")
            if self.simulator and self.is_running:
                self.stop_auto_send()
            if self.simulator and self.is_connected:
                self.connect_serial()  # Disconnect
            
            # Update status to show disconnected state
            self.update_connection_status(False)
            
            try:
                dpg.destroy_context()
                print("Graphical user interface program exited")
            except:
                pass


def main():
    """Graphical user interface program entry function"""
    print("Starting RoboMaster Judge System Simulator - Graphical User Interface")
    try:
        app = JudgeSimulatorGUI()
        app.run()
    except Exception as e:
        print(f"Program startup failed: {e}")
        import traceback
        traceback.print_exc()
        input("Press Enter to exit...")


if __name__ == "__main__":
    main()
