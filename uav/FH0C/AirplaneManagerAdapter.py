"""
此文件是使用 FH0C 库对PhantasyIslandPythonRemoteControl库的模拟，
目的是尽可能使得使用PhantasyIslandPythonRemoteControl库编写的代码可以在只修改import导入表的情况下直接调用FH0A库对应的功能
"""
from typing import Dict, Optional
from time import sleep

from .SerialThread import SerialThread


class AirplaneController(object):
    """
    此类是到PhantasyIslandPythonRemoteControl库中AirplaneController的适配器，是对SerialThread的wrapper
    """
    s: SerialThread

    def __init__(self, port):
        self.s = SerialThread(port=port)
        pass

    def takeoff(self, high: int, ):
        """起飞到指定高度 单位cm"""
        self.s.send().takeoff(high)
        pass

    def land(self, ):
        """降落"""
        self.s.send().land()
        pass

    def up(self, distance: int):
        """上升指定距离 单位cm"""
        self.s.send().up(distance)
        pass

    def down(self, distance: int):
        """下降指定距离 单位cm"""
        self.s.send().down(distance)
        pass

    def forward(self, distance: int):
        """前进指定距离 单位cm"""
        self.s.send().forward(distance)
        pass

    def back(self, distance: int):
        """后退指定距离 单位cm"""
        self.s.send().back(distance)
        pass

    def left(self, distance: int):
        """左移指定距离 单位cm"""
        self.s.send().left(distance)
        pass

    def right(self, distance: int):
        """右移指定距离 单位cm"""
        self.s.send().right(distance)
        pass

    def goto(self, x: int, y: int, h: int):
        """移动到指定坐标处"""
        self.s.send().arrive(x, y, h)
        pass

    def rotate(self, degree: int):
        """顺时旋转指定角度"""
        self.s.send().rotate(degree)
        pass

    def cw(self, degree: int):
        """顺时针旋转指定角度"""
        self.s.send().cw(degree)
        pass

    def ccw(self, degree: int):
        """逆时针旋转指定角度"""
        self.s.send().ccw(degree)
        pass

    def speed(self, speed: int):
        """设置飞行速度"""
        self.s.send().speed(speed)
        pass

    def high(self, high: int):
        """移动到指定高度处"""
        self.s.send().high(high)
        pass

    def led(self, r: int, g: int, b: int):
        """设置无人机led色彩"""
        self.s.send().led(r, g, b)
        pass

    def bln(self, r: int, g: int, b: int):
        """设置无人机led呼吸灯色彩"""
        self.s.send().bln(r, g, b)
        pass

    def rainbow(self, r: int, g: int, b: int):
        """设置无人机led彩虹色彩"""
        self.s.send().rainbow(r, g, b)
        pass

    def stop(self):
        """悬停"""
        self.s.send().hovering()
        pass

    def hover(self):
        """悬停"""
        self.s.send().hovering()
        pass

    pass


class AirplaneControllerExtended(AirplaneController):
    """
    此类在AirplaneController的基础上添加了FH0A特有的功能及函数
    """

    def airplane_mode(self, mode: int):
        """设置无人机飞行模式
        :param mode: 1常规2巡线3跟随4单机编队 通常情况下使用模式4
        """
        self.s.send().airplane_mode(mode)
        pass

    def move(self, direction: int, distance: int):
        """移动到指定坐标处"""
        self.s.send().move(direction, distance)
        pass

    def flip(self, direction: int, circle: int = 1):
        """做翻转（翻跟头）动作
        :param port:
        :param direction: 翻转方向：f(向前)/b(向后)/l(向左)/r(向右)
        :param circle: 翻转的圈数
        """
        self.s.send().flip(direction, circle)
        pass

    def flip_forward(self, circle: int = 1):
        """向前做翻转（翻跟头）动作
        :param circle: 翻转的圈数
        """
        self.s.send().flip_forward(circle)
        pass

    def flip_back(self, circle: int = 1):
        """向后做翻转（翻跟头）动作
        :param circle: 翻转的圈数
        """
        self.s.send().flip_back(circle)
        pass

    def flip_left(self, circle: int = 1):
        """向左做翻转（翻跟头）动作
        :param circle: 翻转的圈数
        """
        self.s.send().flip_left(circle)
        pass

    def flip_right(self, circle: int = 1):
        """向右做翻转（翻跟头）动作
        :param circle: 翻转的圈数
        """
        self.s.send().flip_right(circle)
        pass

    def vision_mode(self, mode: int):
        """设置视觉工作模式
        :param mode: 1点检测2线检测3标签检测4二维码扫描5条形码扫描
        """
        self.s.send().vision_mode(mode)
        pass

    def vision_color(self, L_L: int, L_H: int, A_L: int, A_H: int, B_L: int, B_H: int, ):
        """颜色检测 检测指定颜色
        L_*/A_*/B_* 为色彩在 Lab 色彩空间上的L/a/b三个色彩通道
        *_L/*_H 为色彩在 Lab 色彩空间上各个的色彩通道的上下限范围
        """
        self.s.send().vision_color(L_L, L_H, A_L, A_H, B_L, B_H)
        pass

    def mode(self, mode: int):
        """设置无人机飞行模式
        :param mode: 1常规2巡线3跟随4单机编队 通常情况下使用模式4
        """
        self.s.send().airplane_mode(mode)

    # def request_read_multi_setting_info(self):
    #     """发送获取多机编队设置的请求"""
    #     self.s.send().read_multi_setting()
    #     pass
    #
    # def multi_setting_info(self):
    #     """读取到的多机编队信息 MultiSettingInfo
    #     必须先调用 request_read_multi_setting_info
    #     :return: MultiSettingInfo
    #     """
    #     self.s.multi_setting_info()
    #     pass
    #
    # def request_read_single_setting_info(self):
    #     """发送获取单机设置的请求"""
    #     self.s.send().read_single_setting()
    #     pass
    #
    # def single_setting_info(self):
    #     """读取到的单机设置信息 SingleSettingInfo
    #     必须先调用 request_read_single_setting_info
    #     :return: SingleSettingInfo
    #     """
    #     self.s.single_setting_info()
    #     pass
    #
    # def request_read_hardware_info(self):
    #     """发送获取硬件信息的请求"""
    #     self.s.send().read_hardware_setting()
    #     pass
    #
    # def hardware_setting_info(self):
    #     """读取到的硬件信息 HardwareInfo
    #     必须先调用 request_read_hardware_info
    #     :return: HardwareInfo
    #     """
    #     self.s.hardware_info()
    #     pass
    #
    # def vision_sensor_info(self):
    #     """视觉传感器信息 VisionSensorInfo
    #     :return: VisionSensorInfo
    #     """
    #     self.s.vision_sensor_info()
    #     pass
    #
    # def sensor_info(self):
    #     """传感器信息 SensorInfo
    #     :return SensorInfo
    #     """
    #     self.s.sensor_info()
    #     pass
    #
    # def base_info(self):
    #     """基础信息 BaseInfo
    #     :return BaseInfo
    #     """
    #     self.s.base_info()
    #     pass

    def shutdown(self):
        self.s.shutdown()
        pass

    pass


class AirplaneManager(object):
    """
    此类是到PhantasyIslandPythonRemoteControl库中AirplaneManager的适配器
    完全适配PhantasyIslandPythonRemoteControl的API
    """
    airplanes_table: Dict[str, AirplaneControllerExtended] = {}

    def ping(self):
        """
        这个函数为了完全适配PhantasyIslandPythonRemoteControl的API而存在
        """
        return {'ok': True, 'r': ''}

    def ping_volatile(self):
        """
        这个函数为了完全适配PhantasyIslandPythonRemoteControl的API而存在
        """
        return {'ok': True, 'r': ''}

    def start(self):
        """
        这个函数为了完全适配PhantasyIslandPythonRemoteControl的API而存在
        """
        return {'ok': True, 'r': ''}

    def get_airplane(self, id: str) -> Optional[AirplaneController]:
        """获取飞机对象
        这个函数获取的API完全适配PhantasyIslandPythonRemoteControl的API
        """
        return self.get_airplane_extended(id)
        pass

    def get_airplane_extended(self, id: str) -> Optional[AirplaneControllerExtended]:
        """获取扩展飞机对象
        这个函数获取的API在完全适配PhantasyIslandPythonRemoteControl的API基础上，添加了FH0C无人机特有功能API
        """
        a = self.airplanes_table.get(id)
        if a is not None:
            return a
        else:
            self.airplanes_table[id] = AirplaneControllerExtended(id)
            return self.airplanes_table.get(id)
        pass

    def sleep(self, time):
        sleep(time)
        pass

    def flush(self):
        """刷新飞机数据
        这个函数为了完全适配PhantasyIslandPythonRemoteControl的API而存在
        """
        return None
        pass

    def destroy(self):
        """反注册所有无人机"""
        for i in self.airplanes_table.values():
            i.shutdown()
            pass
        self.airplanes_table = {}


airplane_manager_singleton = AirplaneManager()


def get_airplane_manager():
    """获取AirplaneManager单例"""
    return airplane_manager_singleton
