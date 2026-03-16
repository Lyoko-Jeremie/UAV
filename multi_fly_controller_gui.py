"""
无人机手动控制测试GUI工具
支持 FH0A、FH0C 和 OWL02 类型的无人机
通过点击按钮发送指令来测试无人机的各项功能
使用 tkinter 实现
"""
import sys
import os
import threading
import time
from datetime import datetime
from typing import Optional
import tkinter as tk
from tkinter import ttk, messagebox, filedialog, colorchooser
from PIL import Image, ImageTk
import io

# 导入无人机控制模块
from uav import UAVAirplaneManager, get_airplane_manager


class ImageCaptureThread(threading.Thread):
    """FH0C拍照线程"""

    def __init__(self, airplane_controller, callbacks):
        super().__init__(daemon=True)
        self.airplane = airplane_controller
        self.callbacks = callbacks  # dict: image_ready, progress_update, status_update, error_occurred, finished
        self.running = True

    def run(self):
        """执行拍照并等待图片传输完成"""
        try:
            # 发送拍照指令
            self._call_callback('status_update', "正在发送拍照指令...")
            order_count = self.airplane.image_receiver.send_cap_image()
            
            if order_count is None:
                self._call_callback('error_occurred', "拍照指令发送失败：可能有正在进行的传输")
                return

            self._call_callback('status_update', f"拍照指令已发送，等待图片传输... (ID: {order_count})")

            # 等待图片传输完成
            max_wait_time = 30  # 最大等待时间（秒）
            start_time = time.time()
            
            while self.running and (time.time() - start_time) < max_wait_time:
                # 检查传输进度
                progress = self.airplane.image_receiver.get_transfer_progress()
                if progress:
                    current, total = progress
                    self._call_callback('progress_update', current, total)
                    self._call_callback('status_update', f"正在接收图片数据: {current}/{total} 包")
                
                # 检查是否传输完成
                if not self.airplane.image_receiver.is_transfer_in_progress():
                    # 传输已完成，获取图片
                    image_data = self.airplane.image_receiver.get_image(order_count)
                    if image_data:
                        self._call_callback('status_update', f"图片接收完成！大小: {len(image_data)} 字节")
                        self._call_callback('image_ready', image_data)
                        return
                    else:
                        # 尝试获取最新图片
                        image_data = self.airplane.image_receiver.get_latest_image()
                        if image_data:
                            self._call_callback('status_update', f"图片接收完成！大小: {len(image_data)} 字节")
                            self._call_callback('image_ready', image_data)
                            return
                
                time.sleep(0.1)
            
            if self.running:
                self._call_callback('error_occurred', "图片传输超时")
        except Exception as e:
            self._call_callback('error_occurred', f"拍照过程出错: {str(e)}")
        finally:
            self._call_callback('finished')

    def _call_callback(self, name, *args):
        if name in self.callbacks and self.callbacks[name]:
            self.callbacks[name](*args)

    def stop(self):
        self.running = False


class LogWidget(tk.Frame):
    """日志显示组件"""
    
    def __init__(self, parent):
        super().__init__(parent)
        
        # 创建文本框和滚动条
        self.text = tk.Text(self, height=8, wrap=tk.WORD, state=tk.DISABLED,
                           font=('Consolas', 9))
        scrollbar = ttk.Scrollbar(self, orient=tk.VERTICAL, command=self.text.yview)
        self.text.configure(yscrollcommand=scrollbar.set)
        
        # 配置标签颜色
        self.text.tag_configure('INFO', foreground='black')
        self.text.tag_configure('SUCCESS', foreground='green')
        self.text.tag_configure('WARNING', foreground='orange')
        self.text.tag_configure('ERROR', foreground='red')
        self.text.tag_configure('TIME', foreground='gray')
        
        # 布局
        self.text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
    
    def log(self, message: str, level: str = "INFO"):
        """添加日志"""
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self.text.configure(state=tk.NORMAL)
        self.text.insert(tk.END, f"[{timestamp}]", 'TIME')
        self.text.insert(tk.END, f" [{level}] ", level)
        self.text.insert(tk.END, f"{message}\n")
        self.text.configure(state=tk.DISABLED)
        self.text.see(tk.END)
    
    def clear(self):
        """清空日志"""
        self.text.configure(state=tk.NORMAL)
        self.text.delete(1.0, tk.END)
        self.text.configure(state=tk.DISABLED)


class UAVControlPanel(ttk.Frame):
    """单个无人机控制面板"""
    
    def __init__(self, parent, uav_id: str, manager: UAVAirplaneManager, log_widget: LogWidget):
        super().__init__(parent)
        self.uav_id = uav_id
        self.manager = manager
        self.log_widget = log_widget
        self.airplane = None
        self.capture_thread = None
        self.captured_image_data = None
        self.photo_image = None  # 保持图片引用
        
        self.init_ui()
    
    def init_ui(self):
        # 连接状态框架
        conn_frame = ttk.LabelFrame(self, text="连接状态")
        conn_frame.pack(fill=tk.X, padx=5, pady=5)
        
        ttk.Label(conn_frame, text="状态:").pack(side=tk.LEFT, padx=5)
        self.status_label = ttk.Label(conn_frame, text="未连接", foreground='gray')
        self.status_label.pack(side=tk.LEFT, padx=5)
        
        self.disconnect_btn = ttk.Button(conn_frame, text="断开", command=self.disconnect_uav, state=tk.DISABLED)
        self.disconnect_btn.pack(side=tk.RIGHT, padx=5, pady=5)
        self.connect_btn = ttk.Button(conn_frame, text="连接", command=self.connect_uav)
        self.connect_btn.pack(side=tk.RIGHT, padx=5, pady=5)
        
        # 创建标签页
        self.notebook = ttk.Notebook(self)
        self.notebook.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # 基础飞行控制
        flight_tab = self.create_flight_tab()
        self.notebook.add(flight_tab, text="飞行控制")
        
        # 移动控制
        move_tab = self.create_move_tab()
        self.notebook.add(move_tab, text="移动控制")
        
        # LED控制
        led_tab = self.create_led_tab()
        self.notebook.add(led_tab, text="LED控制")
        
        # 高级功能
        advanced_tab = self.create_advanced_tab()
        self.notebook.add(advanced_tab, text="高级功能")
        
        # FH0C拍照功能（只对FH0C类型显示）
        if self.uav_id.upper().startswith("FH0C"):
            camera_tab = self.create_camera_tab()
            self.notebook.add(camera_tab, text="📷 拍照")
    
    def create_flight_tab(self) -> ttk.Frame:
        """创建飞行控制标签页"""
        frame = ttk.Frame(self.notebook)
        
        # 起飞/降落
        takeoff_frame = ttk.LabelFrame(frame, text="起飞/降落")
        takeoff_frame.pack(fill=tk.X, padx=5, pady=5)
        
        row1 = ttk.Frame(takeoff_frame)
        row1.pack(fill=tk.X, padx=5, pady=5)
        
        ttk.Label(row1, text="起飞高度(cm):").pack(side=tk.LEFT)
        self.takeoff_height = tk.IntVar(value=80)
        ttk.Spinbox(row1, from_=30, to=200, textvariable=self.takeoff_height, width=8).pack(side=tk.LEFT, padx=5)
        
        takeoff_btn = tk.Button(row1, text="🚀 起飞", command=self.do_takeoff,
                               bg='#4CAF50', fg='white', font=('', 9, 'bold'))
        takeoff_btn.pack(side=tk.LEFT, padx=5)
        
        land_btn = tk.Button(row1, text="🛬 降落", command=self.do_land,
                            bg='#FF9800', fg='white', font=('', 9, 'bold'))
        land_btn.pack(side=tk.LEFT, padx=5)
        
        hover_btn = tk.Button(row1, text="⏸ 悬停", command=self.do_hover,
                             bg='#2196F3', fg='white', font=('', 9, 'bold'))
        hover_btn.pack(side=tk.LEFT, padx=5)
        
        # 速度设置
        speed_frame = ttk.LabelFrame(frame, text="速度设置")
        speed_frame.pack(fill=tk.X, padx=5, pady=5)
        
        row2 = ttk.Frame(speed_frame)
        row2.pack(fill=tk.X, padx=5, pady=5)
        
        ttk.Label(row2, text="飞行速度:").pack(side=tk.LEFT)
        self.speed_var = tk.IntVar(value=50)
        ttk.Spinbox(row2, from_=10, to=100, textvariable=self.speed_var, width=8).pack(side=tk.LEFT, padx=5)
        ttk.Button(row2, text="设置速度", command=self.do_set_speed).pack(side=tk.LEFT, padx=5)
        
        # 高度控制
        height_frame = ttk.LabelFrame(frame, text="高度控制")
        height_frame.pack(fill=tk.X, padx=5, pady=5)
        
        row3 = ttk.Frame(height_frame)
        row3.pack(fill=tk.X, padx=5, pady=5)
        
        ttk.Label(row3, text="目标高度(cm):").pack(side=tk.LEFT)
        self.height_var = tk.IntVar(value=80)
        ttk.Spinbox(row3, from_=30, to=200, textvariable=self.height_var, width=8).pack(side=tk.LEFT, padx=5)
        ttk.Button(row3, text="移动到高度", command=self.do_set_height).pack(side=tk.LEFT, padx=5)
        
        # 飞行模式
        mode_frame = ttk.LabelFrame(frame, text="飞行模式")
        mode_frame.pack(fill=tk.X, padx=5, pady=5)
        
        row4 = ttk.Frame(mode_frame)
        row4.pack(fill=tk.X, padx=5, pady=5)
        
        ttk.Label(row4, text="模式:").pack(side=tk.LEFT)
        self.mode_var = tk.StringVar()
        mode_combo = ttk.Combobox(row4, textvariable=self.mode_var, state='readonly', width=15)
        if self.uav_id.upper().startswith("FH0C"):
            mode_combo['values'] = ["0 - 光流", "1 - 标签", "2 - 巡线"]
            mode_combo.current(0)
        else:
            mode_combo['values'] = ["1 - 常规", "2 - 巡线", "3 - 跟随", "4 - 单机编队"]
            mode_combo.current(3)
        mode_combo.pack(side=tk.LEFT, padx=5)
        ttk.Button(row4, text="设置模式", command=self.do_set_mode).pack(side=tk.LEFT, padx=5)
        
        return frame
    
    def create_move_tab(self) -> ttk.Frame:
        """创建移动控制标签页"""
        frame = ttk.Frame(self.notebook)
        
        # 距离设置
        dist_frame = ttk.Frame(frame)
        dist_frame.pack(fill=tk.X, padx=5, pady=5)
        
        ttk.Label(dist_frame, text="移动距离(cm):").pack(side=tk.LEFT)
        self.move_distance = tk.IntVar(value=50)
        ttk.Spinbox(dist_frame, from_=10, to=500, textvariable=self.move_distance, width=8).pack(side=tk.LEFT, padx=5)
        
        # 方向控制
        direction_frame = ttk.LabelFrame(frame, text="方向移动")
        direction_frame.pack(fill=tk.X, padx=5, pady=5)
        
        # 使用网格布局
        grid_frame = ttk.Frame(direction_frame)
        grid_frame.pack(padx=10, pady=10)
        
        # 上升/下降
        ttk.Button(grid_frame, text="⬆ 上升", command=lambda: self.do_move("up")).grid(row=0, column=0, padx=5, pady=2)
        ttk.Button(grid_frame, text="⬇ 下降", command=lambda: self.do_move("down")).grid(row=2, column=0, padx=5, pady=2)
        
        # 前/后/左/右
        ttk.Button(grid_frame, text="↑ 前进", command=lambda: self.do_move("forward")).grid(row=0, column=2, padx=5, pady=2)
        ttk.Button(grid_frame, text="↓ 后退", command=lambda: self.do_move("back")).grid(row=2, column=2, padx=5, pady=2)
        ttk.Button(grid_frame, text="← 左移", command=lambda: self.do_move("left")).grid(row=1, column=1, padx=5, pady=2)
        ttk.Button(grid_frame, text="→ 右移", command=lambda: self.do_move("right")).grid(row=1, column=3, padx=5, pady=2)
        
        # 旋转控制
        rotate_frame = ttk.LabelFrame(frame, text="旋转控制")
        rotate_frame.pack(fill=tk.X, padx=5, pady=5)
        
        rotate_row = ttk.Frame(rotate_frame)
        rotate_row.pack(fill=tk.X, padx=5, pady=5)
        
        ttk.Label(rotate_row, text="角度(°):").pack(side=tk.LEFT)
        self.rotate_angle = tk.IntVar(value=90)
        ttk.Spinbox(rotate_row, from_=1, to=360, textvariable=self.rotate_angle, width=8).pack(side=tk.LEFT, padx=5)
        ttk.Button(rotate_row, text="↻ 顺时针", command=self.do_cw).pack(side=tk.LEFT, padx=5)
        ttk.Button(rotate_row, text="↺ 逆时针", command=self.do_ccw).pack(side=tk.LEFT, padx=5)
        
        # 翻转动作
        flip_frame = ttk.LabelFrame(frame, text="翻转动作")
        flip_frame.pack(fill=tk.X, padx=5, pady=5)
        
        flip_row = ttk.Frame(flip_frame)
        flip_row.pack(fill=tk.X, padx=5, pady=5)
        
        ttk.Button(flip_row, text="前翻", command=lambda: self.do_flip("forward")).pack(side=tk.LEFT, padx=5)
        ttk.Button(flip_row, text="后翻", command=lambda: self.do_flip("back")).pack(side=tk.LEFT, padx=5)
        ttk.Button(flip_row, text="左翻", command=lambda: self.do_flip("left")).pack(side=tk.LEFT, padx=5)
        ttk.Button(flip_row, text="右翻", command=lambda: self.do_flip("right")).pack(side=tk.LEFT, padx=5)
        
        # 坐标移动
        goto_frame = ttk.LabelFrame(frame, text="坐标移动 (Goto)")
        goto_frame.pack(fill=tk.X, padx=5, pady=5)
        
        goto_row = ttk.Frame(goto_frame)
        goto_row.pack(fill=tk.X, padx=5, pady=5)
        
        ttk.Label(goto_row, text="X:").pack(side=tk.LEFT)
        self.goto_x = tk.IntVar(value=0)
        ttk.Spinbox(goto_row, from_=-500, to=500, textvariable=self.goto_x, width=6).pack(side=tk.LEFT, padx=2)
        
        ttk.Label(goto_row, text="Y:").pack(side=tk.LEFT)
        self.goto_y = tk.IntVar(value=0)
        ttk.Spinbox(goto_row, from_=-500, to=500, textvariable=self.goto_y, width=6).pack(side=tk.LEFT, padx=2)
        
        ttk.Label(goto_row, text="H:").pack(side=tk.LEFT)
        self.goto_h = tk.IntVar(value=80)
        ttk.Spinbox(goto_row, from_=30, to=200, textvariable=self.goto_h, width=6).pack(side=tk.LEFT, padx=2)
        
        ttk.Button(goto_row, text="移动到坐标", command=self.do_goto).pack(side=tk.LEFT, padx=10)
        
        return frame
    
    def create_led_tab(self) -> ttk.Frame:
        """创建LED控制标签页"""
        frame = ttk.Frame(self.notebook)
        
        # RGB颜色选择
        color_frame = ttk.LabelFrame(frame, text="LED颜色")
        color_frame.pack(fill=tk.X, padx=5, pady=5)
        
        rgb_row = ttk.Frame(color_frame)
        rgb_row.pack(fill=tk.X, padx=5, pady=5)
        
        ttk.Label(rgb_row, text="R:").pack(side=tk.LEFT)
        self.led_r = tk.IntVar(value=255)
        ttk.Spinbox(rgb_row, from_=0, to=255, textvariable=self.led_r, width=5,
                   command=self.update_color_preview).pack(side=tk.LEFT, padx=2)
        
        ttk.Label(rgb_row, text="G:").pack(side=tk.LEFT)
        self.led_g = tk.IntVar(value=0)
        ttk.Spinbox(rgb_row, from_=0, to=255, textvariable=self.led_g, width=5,
                   command=self.update_color_preview).pack(side=tk.LEFT, padx=2)
        
        ttk.Label(rgb_row, text="B:").pack(side=tk.LEFT)
        self.led_b = tk.IntVar(value=0)
        ttk.Spinbox(rgb_row, from_=0, to=255, textvariable=self.led_b, width=5,
                   command=self.update_color_preview).pack(side=tk.LEFT, padx=2)
        
        # 颜色预览
        self.color_preview = tk.Label(rgb_row, text="   ", bg='#ff0000', width=5, relief=tk.SUNKEN)
        self.color_preview.pack(side=tk.LEFT, padx=10)
        
        ttk.Button(rgb_row, text="选择颜色", command=self.pick_color).pack(side=tk.LEFT, padx=5)
        
        # LED模式按钮
        mode_frame = ttk.LabelFrame(frame, text="LED模式")
        mode_frame.pack(fill=tk.X, padx=5, pady=5)
        
        mode_row = ttk.Frame(mode_frame)
        mode_row.pack(fill=tk.X, padx=5, pady=5)
        
        ttk.Button(mode_row, text="💡 设置颜色", command=self.do_led).pack(side=tk.LEFT, padx=5)
        ttk.Button(mode_row, text="💫 呼吸灯", command=self.do_bln).pack(side=tk.LEFT, padx=5)
        ttk.Button(mode_row, text="🌈 彩虹", command=self.do_rainbow).pack(side=tk.LEFT, padx=5)
        
        # 预设颜色
        preset_frame = ttk.LabelFrame(frame, text="预设颜色")
        preset_frame.pack(fill=tk.X, padx=5, pady=5)
        
        preset_colors = [
            ("红", 255, 0, 0), ("绿", 0, 255, 0), ("蓝", 0, 0, 255),
            ("黄", 255, 255, 0), ("青", 0, 255, 255), ("紫", 255, 0, 255),
            ("白", 255, 255, 255), ("橙", 255, 165, 0), ("关闭", 0, 0, 0)
        ]
        
        preset_grid = ttk.Frame(preset_frame)
        preset_grid.pack(padx=5, pady=5)
        
        for i, (name, r, g, b) in enumerate(preset_colors):
            bg_color = f'#{r:02x}{g:02x}{b:02x}'
            fg_color = 'black' if r + g + b > 380 else 'white'
            btn = tk.Button(preset_grid, text=name, bg=bg_color, fg=fg_color, width=6,
                           command=lambda r=r, g=g, b=b: self.set_preset_color(r, g, b))
            btn.grid(row=i // 3, column=i % 3, padx=2, pady=2)
        
        return frame
    
    def create_advanced_tab(self) -> ttk.Frame:
        """创建高级功能标签页"""
        frame = ttk.Frame(self.notebook)
        
        # 视觉模式
        vision_frame = ttk.LabelFrame(frame, text="视觉模式")
        vision_frame.pack(fill=tk.X, padx=5, pady=5)
        
        vision_row = ttk.Frame(vision_frame)
        vision_row.pack(fill=tk.X, padx=5, pady=5)
        
        ttk.Label(vision_row, text="视觉模式:").pack(side=tk.LEFT)
        self.vision_var = tk.StringVar()
        vision_combo = ttk.Combobox(vision_row, textvariable=self.vision_var, state='readonly', width=15)
        vision_combo['values'] = [
            "1 - 点检测", "2 - 线检测", "3 - 标签检测",
            "4 - 二维码扫描", "5 - 条形码扫描"
        ]
        vision_combo.current(0)
        vision_combo.pack(side=tk.LEFT, padx=5)
        ttk.Button(vision_row, text="设置", command=self.do_set_vision_mode).pack(side=tk.LEFT, padx=5)
        
        # 颜色检测
        color_detect_frame = ttk.LabelFrame(frame, text="颜色检测 (Lab色彩空间)")
        color_detect_frame.pack(fill=tk.X, padx=5, pady=5)
        
        # L通道
        l_row = ttk.Frame(color_detect_frame)
        l_row.pack(fill=tk.X, padx=5, pady=2)
        ttk.Label(l_row, text="L_L:").pack(side=tk.LEFT)
        self.l_l = tk.IntVar(value=0)
        ttk.Spinbox(l_row, from_=0, to=255, textvariable=self.l_l, width=5).pack(side=tk.LEFT, padx=2)
        ttk.Label(l_row, text="L_H:").pack(side=tk.LEFT, padx=(10, 0))
        self.l_h = tk.IntVar(value=255)
        ttk.Spinbox(l_row, from_=0, to=255, textvariable=self.l_h, width=5).pack(side=tk.LEFT, padx=2)
        
        # A通道
        a_row = ttk.Frame(color_detect_frame)
        a_row.pack(fill=tk.X, padx=5, pady=2)
        ttk.Label(a_row, text="A_L:").pack(side=tk.LEFT)
        self.a_l = tk.IntVar(value=0)
        ttk.Spinbox(a_row, from_=0, to=255, textvariable=self.a_l, width=5).pack(side=tk.LEFT, padx=2)
        ttk.Label(a_row, text="A_H:").pack(side=tk.LEFT, padx=(10, 0))
        self.a_h = tk.IntVar(value=255)
        ttk.Spinbox(a_row, from_=0, to=255, textvariable=self.a_h, width=5).pack(side=tk.LEFT, padx=2)
        
        # B通道
        b_row = ttk.Frame(color_detect_frame)
        b_row.pack(fill=tk.X, padx=5, pady=2)
        ttk.Label(b_row, text="B_L:").pack(side=tk.LEFT)
        self.b_l = tk.IntVar(value=0)
        ttk.Spinbox(b_row, from_=0, to=255, textvariable=self.b_l, width=5).pack(side=tk.LEFT, padx=2)
        ttk.Label(b_row, text="B_H:").pack(side=tk.LEFT, padx=(10, 0))
        self.b_h = tk.IntVar(value=255)
        ttk.Spinbox(b_row, from_=0, to=255, textvariable=self.b_h, width=5).pack(side=tk.LEFT, padx=2)
        
        ttk.Button(color_detect_frame, text="设置颜色检测", command=self.do_color_detect).pack(pady=5)
        
        return frame
    
    def create_camera_tab(self) -> ttk.Frame:
        """创建FH0C拍照功能标签页"""
        frame = ttk.Frame(self.notebook)
        
        # 拍照控制
        capture_frame = ttk.LabelFrame(frame, text="拍照控制")
        capture_frame.pack(fill=tk.X, padx=5, pady=5)
        
        btn_row = ttk.Frame(capture_frame)
        btn_row.pack(fill=tk.X, padx=5, pady=5)
        
        self.capture_btn = tk.Button(btn_row, text="📷 拍照", command=self.do_capture_image,
                                    bg='#E91E63', fg='white', font=('', 11, 'bold'), padx=15, pady=5)
        self.capture_btn.pack(side=tk.LEFT, padx=5)
        
        self.save_btn = ttk.Button(btn_row, text="💾 保存图片", command=self.do_save_image, state=tk.DISABLED)
        self.save_btn.pack(side=tk.LEFT, padx=5)
        
        # 进度条
        self.capture_progress = ttk.Progressbar(capture_frame, mode='determinate')
        self.capture_progress.pack(fill=tk.X, padx=5, pady=5)
        self.capture_progress.pack_forget()  # 初始隐藏
        
        # 状态标签
        self.capture_status = ttk.Label(capture_frame, text="点击'拍照'按钮开始拍照")
        self.capture_status.pack(padx=5, pady=5)
        
        # 图片预览
        preview_frame = ttk.LabelFrame(frame, text="图片预览")
        preview_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        self.image_preview = ttk.Label(preview_frame, text="暂无图片", anchor=tk.CENTER,
                                      background='#f0f0f0', relief=tk.SUNKEN)
        self.image_preview.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        return frame
    
    # ============ 控制方法 ============
    
    def connect_uav(self):
        """连接无人机"""
        try:
            self.airplane = self.manager.get_airplane_extended(self.uav_id)
            self.status_label.config(text="已连接", foreground='green')
            self.connect_btn.config(state=tk.DISABLED)
            self.disconnect_btn.config(state=tk.NORMAL)
            self.log_widget.log(f"无人机 {self.uav_id} 连接成功", "SUCCESS")
        except Exception as e:
            self.log_widget.log(f"连接失败: {str(e)}", "ERROR")
            messagebox.showwarning("连接失败", str(e))
    
    def disconnect_uav(self):
        """断开无人机"""
        try:
            if self.airplane:
                self.airplane.shutdown()
                self.airplane = None
            self.status_label.config(text="已断开", foreground='gray')
            self.connect_btn.config(state=tk.NORMAL)
            self.disconnect_btn.config(state=tk.DISABLED)
            self.log_widget.log(f"无人机 {self.uav_id} 已断开", "INFO")
        except Exception as e:
            self.log_widget.log(f"断开失败: {str(e)}", "ERROR")
    
    def check_connection(self) -> bool:
        """检查连接状态"""
        if self.airplane is None:
            messagebox.showwarning("未连接", "请先连接无人机")
            return False
        return True
    
    def do_takeoff(self):
        """起飞"""
        if not self.check_connection():
            return
        height = self.takeoff_height.get()
        try:
            self.airplane.takeoff(height)
            self.log_widget.log(f"起飞到 {height}cm", "SUCCESS")
        except Exception as e:
            self.log_widget.log(f"起飞失败: {str(e)}", "ERROR")
    
    def do_land(self):
        """降落"""
        if not self.check_connection():
            return
        try:
            self.airplane.land()
            self.log_widget.log("降落", "SUCCESS")
        except Exception as e:
            self.log_widget.log(f"降落失败: {str(e)}", "ERROR")
    
    def do_hover(self):
        """悬停"""
        if not self.check_connection():
            return
        try:
            self.airplane.hover()
            self.log_widget.log("悬停", "SUCCESS")
        except Exception as e:
            self.log_widget.log(f"悬停失败: {str(e)}", "ERROR")
    
    def do_set_speed(self):
        """设置速度"""
        if not self.check_connection():
            return
        speed = self.speed_var.get()
        try:
            self.airplane.speed(speed)
            self.log_widget.log(f"设置速度: {speed}", "SUCCESS")
        except Exception as e:
            self.log_widget.log(f"设置速度失败: {str(e)}", "ERROR")
    
    def do_set_height(self):
        """设置高度"""
        if not self.check_connection():
            return
        height = self.height_var.get()
        try:
            self.airplane.high(height)
            self.log_widget.log(f"移动到高度: {height}cm", "SUCCESS")
        except Exception as e:
            self.log_widget.log(f"设置高度失败: {str(e)}", "ERROR")
    
    def do_set_mode(self):
        """设置飞行模式"""
        if not self.check_connection():
            return
        mode_text = self.mode_var.get()
        mode = int(mode_text.split(" - ")[0])
        try:
            self.airplane.airplane_mode(mode)
            self.log_widget.log(f"设置飞行模式: {mode_text}", "SUCCESS")
        except Exception as e:
            self.log_widget.log(f"设置模式失败: {str(e)}", "ERROR")
    
    def do_move(self, direction: str):
        """移动"""
        if not self.check_connection():
            return
        distance = self.move_distance.get()
        try:
            method = getattr(self.airplane, direction)
            method(distance)
            self.log_widget.log(f"{direction} {distance}cm", "SUCCESS")
        except Exception as e:
            self.log_widget.log(f"移动失败: {str(e)}", "ERROR")
    
    def do_cw(self):
        """顺时针旋转"""
        if not self.check_connection():
            return
        angle = self.rotate_angle.get()
        try:
            self.airplane.cw(angle)
            self.log_widget.log(f"顺时针旋转 {angle}°", "SUCCESS")
        except Exception as e:
            self.log_widget.log(f"旋转失败: {str(e)}", "ERROR")
    
    def do_ccw(self):
        """逆时针旋转"""
        if not self.check_connection():
            return
        angle = self.rotate_angle.get()
        try:
            self.airplane.ccw(angle)
            self.log_widget.log(f"逆时针旋转 {angle}°", "SUCCESS")
        except Exception as e:
            self.log_widget.log(f"旋转失败: {str(e)}", "ERROR")
    
    def do_flip(self, direction: str):
        """翻转"""
        if not self.check_connection():
            return
        try:
            method_name = f"flip_{direction}"
            method = getattr(self.airplane, method_name)
            method()
            self.log_widget.log(f"翻转: {direction}", "SUCCESS")
        except Exception as e:
            self.log_widget.log(f"翻转失败: {str(e)}", "ERROR")
    
    def do_goto(self):
        """移动到坐标"""
        if not self.check_connection():
            return
        x = self.goto_x.get()
        y = self.goto_y.get()
        h = self.goto_h.get()
        try:
            self.airplane.goto(x, y, h)
            self.log_widget.log(f"移动到坐标 ({x}, {y}, {h})", "SUCCESS")
        except Exception as e:
            self.log_widget.log(f"移动失败: {str(e)}", "ERROR")
    
    def update_color_preview(self):
        """更新颜色预览"""
        r = self.led_r.get()
        g = self.led_g.get()
        b = self.led_b.get()
        color = f'#{r:02x}{g:02x}{b:02x}'
        self.color_preview.config(bg=color)
    
    def pick_color(self):
        """选择颜色"""
        color = colorchooser.askcolor(title="选择颜色")
        if color[0]:
            r, g, b = [int(c) for c in color[0]]
            self.led_r.set(r)
            self.led_g.set(g)
            self.led_b.set(b)
            self.update_color_preview()
    
    def set_preset_color(self, r: int, g: int, b: int):
        """设置预设颜色"""
        self.led_r.set(r)
        self.led_g.set(g)
        self.led_b.set(b)
        self.update_color_preview()
        self.do_led()
    
    def do_led(self):
        """设置LED颜色"""
        if not self.check_connection():
            return
        r = self.led_r.get()
        g = self.led_g.get()
        b = self.led_b.get()
        try:
            self.airplane.led(r, g, b)
            self.log_widget.log(f"设置LED颜色: RGB({r}, {g}, {b})", "SUCCESS")
        except Exception as e:
            self.log_widget.log(f"设置LED失败: {str(e)}", "ERROR")
    
    def do_bln(self):
        """设置呼吸灯"""
        if not self.check_connection():
            return
        r = self.led_r.get()
        g = self.led_g.get()
        b = self.led_b.get()
        try:
            self.airplane.bln(r, g, b)
            self.log_widget.log(f"设置呼吸灯: RGB({r}, {g}, {b})", "SUCCESS")
        except Exception as e:
            self.log_widget.log(f"设置呼吸灯失败: {str(e)}", "ERROR")
    
    def do_rainbow(self):
        """设置彩虹灯"""
        if not self.check_connection():
            return
        r = self.led_r.get()
        g = self.led_g.get()
        b = self.led_b.get()
        try:
            self.airplane.rainbow(r, g, b)
            self.log_widget.log(f"设置彩虹灯: RGB({r}, {g}, {b})", "SUCCESS")
        except Exception as e:
            self.log_widget.log(f"设置彩虹灯失败: {str(e)}", "ERROR")
    
    def do_set_vision_mode(self):
        """设置视觉模式"""
        if not self.check_connection():
            return
        mode_text = self.vision_var.get()
        mode = int(mode_text.split(" - ")[0])
        try:
            self.airplane.vision_mode(mode)
            self.log_widget.log(f"设置视觉模式: {mode_text}", "SUCCESS")
        except Exception as e:
            self.log_widget.log(f"设置视觉模式失败: {str(e)}", "ERROR")
    
    def do_color_detect(self):
        """设置颜色检测"""
        if not self.check_connection():
            return
        try:
            self.airplane.vision_color(
                self.l_l.get(), self.l_h.get(),
                self.a_l.get(), self.a_h.get(),
                self.b_l.get(), self.b_h.get()
            )
            self.log_widget.log("设置颜色检测参数", "SUCCESS")
        except Exception as e:
            self.log_widget.log(f"设置颜色检测失败: {str(e)}", "ERROR")
    
    # ============ FH0C 拍照功能 ============
    
    def do_capture_image(self):
        """拍照"""
        if not self.check_connection():
            return
        
        # 检查是否是FH0C类型
        if not self.uav_id.upper().startswith("FH0C"):
            messagebox.showwarning("不支持", "拍照功能仅支持FH0C类型无人机")
            return
        
        # 禁用拍照按钮
        self.capture_btn.config(state=tk.DISABLED)
        self.capture_progress.pack(fill=tk.X, padx=5, pady=5)
        self.capture_progress['value'] = 0
        self.capture_status.config(text="正在拍照...")
        
        # 定义回调函数
        def on_image_ready(image_data):
            self.winfo_toplevel().after(0, lambda: self._on_image_ready(image_data))
        
        def on_progress_update(current, total):
            self.winfo_toplevel().after(0, lambda: self._on_capture_progress(current, total))
        
        def on_status_update(status):
            self.winfo_toplevel().after(0, lambda: self._on_capture_status(status))
        
        def on_error(error):
            self.winfo_toplevel().after(0, lambda: self._on_capture_error(error))
        
        def on_finished():
            self.winfo_toplevel().after(0, self._on_capture_finished)
        
        callbacks = {
            'image_ready': on_image_ready,
            'progress_update': on_progress_update,
            'status_update': on_status_update,
            'error_occurred': on_error,
            'finished': on_finished
        }
        
        # 启动拍照线程
        self.capture_thread = ImageCaptureThread(self.airplane, callbacks)
        self.capture_thread.start()
        
        self.log_widget.log("开始拍照...", "INFO")
    
    def _on_image_ready(self, image_data: bytes):
        """图片就绪"""
        self.captured_image_data = image_data
        self.save_btn.config(state=tk.NORMAL)
        
        # 显示图片预览
        try:
            image = Image.open(io.BytesIO(image_data))
            # 缩放以适应预览区域
            max_size = (320, 240)
            image.thumbnail(max_size, Image.Resampling.LANCZOS)
            self.photo_image = ImageTk.PhotoImage(image)
            self.image_preview.config(image=self.photo_image, text='')
            self.log_widget.log(f"图片接收完成，大小: {len(image_data)} 字节", "SUCCESS")
        except Exception as e:
            self.image_preview.config(text=f"显示失败: {str(e)}")
            self.log_widget.log(f"图片显示失败: {str(e)}", "ERROR")
    
    def _on_capture_progress(self, current: int, total: int):
        """更新拍照进度"""
        if total > 0:
            progress = int((current / total) * 100)
            self.capture_progress['value'] = progress
    
    def _on_capture_status(self, status: str):
        """更新拍照状态"""
        self.capture_status.config(text=status)
    
    def _on_capture_error(self, error: str):
        """拍照错误"""
        self.log_widget.log(f"拍照错误: {error}", "ERROR")
        messagebox.showwarning("拍照失败", error)
    
    def _on_capture_finished(self):
        """拍照完成"""
        self.capture_btn.config(state=tk.NORMAL)
        self.capture_progress.pack_forget()
    
    def do_save_image(self):
        """保存图片"""
        if self.captured_image_data is None:
            messagebox.showwarning("无图片", "没有可保存的图片")
            return
        
        # 选择保存路径
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        default_name = f"uav_capture_{timestamp}.jpg"
        file_path = filedialog.asksaveasfilename(
            defaultextension=".jpg",
            initialfile=default_name,
            filetypes=[("JPEG图片", "*.jpg"), ("PNG图片", "*.png"), ("所有文件", "*.*")]
        )
        
        if file_path:
            try:
                with open(file_path, 'wb') as f:
                    f.write(self.captured_image_data)
                self.log_widget.log(f"图片已保存: {file_path}", "SUCCESS")
                messagebox.showinfo("保存成功", f"图片已保存到:\n{file_path}")
            except Exception as e:
                self.log_widget.log(f"保存失败: {str(e)}", "ERROR")
                messagebox.showwarning("保存失败", str(e))


class MultiUAVControllerGUI:
    """多无人机控制器主窗口"""
    
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("无人机手动控制测试工具")
        self.root.geometry("900x750")
        self.root.minsize(800, 600)
        
        self.manager = get_airplane_manager()
        self.uav_panels = {}
        
        self.init_ui()
        
        # 绑定关闭事件
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
    
    def init_ui(self):
        # 顶部：添加无人机
        add_frame = ttk.LabelFrame(self.root, text="添加无人机")
        add_frame.pack(fill=tk.X, padx=5, pady=5)
        
        add_row = ttk.Frame(add_frame)
        add_row.pack(fill=tk.X, padx=5, pady=5)
        
        ttk.Label(add_row, text="无人机类型:").pack(side=tk.LEFT)
        self.type_var = tk.StringVar(value="FH0A")
        type_combo = ttk.Combobox(add_row, textvariable=self.type_var, state='readonly', width=8)
        type_combo['values'] = ["FH0A", "FH0C", "OWL02"]
        type_combo.pack(side=tk.LEFT, padx=5)
        type_combo.bind('<<ComboboxSelected>>', self.on_type_changed)
        
        ttk.Label(add_row, text="串口:").pack(side=tk.LEFT, padx=(10, 0))
        self.port_var = tk.StringVar(value="COM3")
        ttk.Entry(add_row, textvariable=self.port_var, width=10).pack(side=tk.LEFT, padx=5)
        
        ttk.Label(add_row, text="OWL02 ID:").pack(side=tk.LEFT, padx=(10, 0))
        self.owl_id_var = tk.IntVar(value=0)
        self.owl_id_spinbox = ttk.Spinbox(add_row, from_=0, to=15, textvariable=self.owl_id_var, width=5, state=tk.DISABLED)
        self.owl_id_spinbox.pack(side=tk.LEFT, padx=5)
        
        ttk.Button(add_row, text="➕ 添加无人机", command=self.add_uav_panel).pack(side=tk.LEFT, padx=10)
        
        # 主区域使用 PanedWindow
        paned = ttk.PanedWindow(self.root, orient=tk.VERTICAL)
        paned.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # 无人机控制面板区域（使用Notebook）
        self.notebook = ttk.Notebook(paned)
        paned.add(self.notebook, weight=3)
        
        # 日志区域
        log_frame = ttk.LabelFrame(paned, text="操作日志")
        paned.add(log_frame, weight=1)
        
        self.log_widget = LogWidget(log_frame)
        self.log_widget.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # 日志控制按钮
        log_btn_frame = ttk.Frame(log_frame)
        log_btn_frame.pack(fill=tk.X, padx=5, pady=2)
        ttk.Button(log_btn_frame, text="清空日志", command=self.log_widget.clear).pack(side=tk.LEFT)
        
        # 初始日志
        self.log_widget.log("无人机控制测试工具已启动", "INFO")
        self.log_widget.log("支持的无人机类型: FH0A, FH0C, OWL02", "INFO")
        self.log_widget.log("FH0C 支持拍照功能", "INFO")
    
    def on_type_changed(self, event=None):
        """无人机类型改变"""
        if self.type_var.get() == "OWL02":
            self.owl_id_spinbox.config(state=tk.NORMAL)
        else:
            self.owl_id_spinbox.config(state=tk.DISABLED)
    
    def add_uav_panel(self):
        """添加无人机控制面板"""
        uav_type = self.type_var.get()
        port = self.port_var.get().strip().upper()
        
        if not port:
            messagebox.showwarning("输入错误", "请输入串口号")
            return
        
        # 构建无人机ID
        if uav_type == "OWL02":
            owl_id = self.owl_id_var.get()
            uav_id = f"{uav_type}:{port}:{owl_id}"
        else:
            uav_id = f"{uav_type}:{port}"
        
        # 检查是否已存在
        if uav_id in self.uav_panels:
            messagebox.showwarning("已存在", f"无人机 {uav_id} 已经添加")
            # 切换到该标签页
            for i in range(self.notebook.index('end')):
                if self.notebook.tab(i, 'text') == uav_id:
                    self.notebook.select(i)
                    break
            return
        
        # 创建控制面板
        panel = UAVControlPanel(self.notebook, uav_id, self.manager, self.log_widget)
        self.uav_panels[uav_id] = panel
        self.notebook.add(panel, text=uav_id)
        self.notebook.select(panel)
        
        self.log_widget.log(f"已添加无人机: {uav_id}", "SUCCESS")
    
    def on_closing(self):
        """关闭窗口事件"""
        # 断开所有连接
        for uav_id, panel in self.uav_panels.items():
            if panel.airplane:
                try:
                    panel.airplane.shutdown()
                except:
                    pass
        
        # 销毁管理器
        try:
            self.manager.destroy()
        except:
            pass
        
        self.root.destroy()
    
    def run(self):
        """运行主循环"""
        self.root.mainloop()


def main():
    """主函数"""
    app = MultiUAVControllerGUI()
    app.run()


if __name__ == "__main__":
    main()

