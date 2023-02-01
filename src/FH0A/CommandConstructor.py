from queue import Queue
from struct import pack, unpack, pack_into, unpack_from
from enum import Enum

from .QueueSignal import QueueSignal


class CmdType(Enum):
    MULTI_SETTINGS = 1
    SINGLE_SETTINGS = 2
    SINGLE_CONTROL = 3
    READ_SETTINGS = 4
    pass


class CommandConstructorCore:
    """
    this class impl core read functions
    """

    """
    last order was generate
    """
    order_last = 0
    """
    
    """
    q_write: Queue = None

    def _order_count(self):
        """
        the order_count generator, it generate new order number
        """
        self.order_last = (self.order_last + 1) % 127
        return self.order_last
        pass

    def _check_sum(self, header: bytearray, params: bytearray):
        """
        impl checksum algorithm
        """
        return bytearray([sum(header + params) & 0xFF])
        pass

    def sendCommand(self, data: bytearray):
        """
        The sendCommand function accepts a cmd bytearray as input and adds it to the queue of commands
        to be sent to the drone.  The function does not return any values.

        :param self: Access the class attributes
        :param data:bytearray: Send the command to the device
        :return: The number of bytes written to the serial port
        :doc-author: Jeremie
        """
        """
        direct do the command bytearray send task
        """
        self.q_write.put((QueueSignal.CMD, data), block=True)
        pass

    def __init__(self, q_write: Queue):
        self.q_write = q_write
        pass

    def join_cmd(self, type: CmdType, params: bytearray):
        """
        The join_cmd function concatenates the header, params and checksum to a cmd bytearray to let it become a valid serial cmd package.
        The function is called in the send_cmd function.


        :param self: Access the attributes and methods of the class in python
        :param type:CmdType: Distinguish the different commands
        :param params:bytearray: Pass the parameters of a command
        :return: A bytearray
        :doc-author: Jeremie
        """
        """
        this function concat header+params+checksum to a bytearray for ready to send
        """
        header = bytearray(b'\xBB\x00\x00')
        if type == CmdType.MULTI_SETTINGS:  # 编队设置 0xBB, 0x08, 0x04
            pack_into("!B", header, 1, 0x08)
            pack_into("!B", header, 2, 0x04)
            # header[1] = 0x08
            # header[2] = 0x04
            checksum = self._check_sum(header, params)
            return header + params + checksum
            pass
        elif type == CmdType.SINGLE_SETTINGS:  # 单机设置 0xBB, 0x07, 0x05
            pack_into("!B", header, 1, 0x07)
            pack_into("!B", header, 2, 0x05)
            # header[1] = 0x07
            # header[2] = 0x05
            checksum = self._check_sum(header, params)
            return header + params + checksum
            pass
        elif type == CmdType.READ_SETTINGS:  # 读设置 0xBB, 0x02, 0x02
            pack_into("!B", header, 1, 0x02)
            pack_into("!B", header, 2, 0x02)
            checksum = self._check_sum(header, params)
            return header + params + checksum
            pass
        elif type == CmdType.SINGLE_CONTROL:  # 串口控制 0xBB, 0x0B, 0xF3
            pack_into("!B", header, 1, 0x0B)
            pack_into("!B", header, 2, 0xF3)
            # header[1] = 0x0B
            # header[2] = 0xF3
            pack_into("!B", params, 9, self._order_count())
            # params.append(self._order_count())
            checksum = self._check_sum(header, params)
            return header + params + checksum
            pass
        pass

    pass


class CommandConstructor(CommandConstructorCore):
    """
    this class extends CommandConstructorCore,
    it impl all functions that direct construct command .
    TODO Implement All Command Methods On This Class
    """

    def __init__(self, q_write: Queue):
        super().__init__(q_write)
        pass

    def led(self, r: int, g: int, b: int):
        """设置无人机led色彩"""
        self._led(0x00, r, g, b)

    def bln(self, r: int, g: int, b: int):
        """设置无人机led呼吸灯色彩"""
        self._led(0x01, r, g, b)

    def rainbow(self, r: int, g: int, b: int):
        """设置无人机led彩虹色彩"""
        self._led(0x02, r, g, b)

    def _led(self, mode: int, r: int, g: int, b: int):
        if mode < 0 or mode > 2:
            raise ValueError("mode illegal", mode)

        params = bytearray(10)
        pack_into("!B", params, 0, 0x08)
        pack_into("!B", params, 1, mode)
        pack_into("!B", params, 2, r)
        pack_into("!B", params, 3, g)
        pack_into("!B", params, 4, b)
        cmd = self.join_cmd(CmdType.SINGLE_CONTROL, params)
        print("led", cmd.hex(' '))
        self.sendCommand(cmd)

        pass

    def takeoff(self, high: int, ):
        """起飞到指定高度 单位cm"""
        params = bytearray(10)
        pack_into("!B", params, 0, 0x01)
        pack_into("!h", params, 1, high)
        cmd = self.join_cmd(CmdType.SINGLE_CONTROL, params)
        print("takeoff", cmd.hex(' '))
        self.sendCommand(cmd)

        pass

    def land(self, ):
        """降落"""
        params = bytearray(10)
        pack_into("!B", params, 0, 0x00)
        cmd = self.join_cmd(CmdType.SINGLE_CONTROL, params)
        print("land", cmd.hex(' '))
        self.sendCommand(cmd)

        pass

    def move(self, direction: int, distance: int):
        """向某个方向移动多少距离
        :param direction: 移动方向（1上2下3前4后5左6右）
        :param distance: 移动距离 单位cm
        """
        if direction < 0 or direction > 6:
            raise ValueError("direction illegal", direction)

        params = bytearray(10)
        pack_into("!B", params, 0, 0x02)
        pack_into("!B", params, 1, direction)
        pack_into("!h", params, 2, distance)
        cmd = self.join_cmd(CmdType.SINGLE_CONTROL, params)
        print("move", cmd.hex(' '))
        self.sendCommand(cmd)

        pass

    def up(self, distance: int):
        """上升指定距离 单位cm"""
        self.move(0x01, distance)
        pass

    def down(self, distance: int):
        """下降指定距离 单位cm"""
        self.move(0x02, distance)
        pass

    def forward(self, distance: int):
        """前进指定距离 单位cm"""
        self.move(0x03, distance)
        pass

    def back(self, distance: int):
        """后退指定距离 单位cm"""
        self.move(0x04, distance)
        pass

    def left(self, distance: int):
        """左移指定距离 单位cm"""
        self.move(0x05, distance)
        pass

    def right(self, distance: int):
        """右移指定距离 单位cm"""
        self.move(0x06, distance)
        pass

    def flip(self, direction: int, circle: int):
        """做翻转（翻跟头）动作
        :param port:
        :param direction: 翻转方向：f(向前)/b(向后)/l(向左)/r(向右)
        :param circle: 翻转的圈数
        """
        if direction < 0 or direction > 4:
            raise ValueError("direction illegal", direction)
        if circle != 1 and circle != 2:
            raise ValueError("circle illegal", circle)

        params = bytearray(10)
        pack_into("!B", params, 0, 0x04)
        pack_into("!B", params, 1, direction)
        pack_into("!B", params, 2, circle)
        cmd = self.join_cmd(CmdType.SINGLE_CONTROL, params)
        print("flip", cmd.hex(' '))
        self.sendCommand(cmd)

        pass

    def flip_forward(self, circle: int):
        """向前做翻转（翻跟头）动作
        :param circle: 翻转的圈数
        """
        self.flip(0x01, circle)
        pass

    def flip_back(self, circle: int):
        """向后做翻转（翻跟头）动作
        :param circle: 翻转的圈数
        """
        self.flip(0x02, circle)
        pass

    def flip_left(self, circle: int):
        """向左做翻转（翻跟头）动作
        :param circle: 翻转的圈数
        """
        self.flip(0x03, circle)
        pass

    def flip_right(self, circle: int):
        """向右做翻转（翻跟头）动作
        :param circle: 翻转的圈数
        """
        self.flip(0x04, circle)
        pass

    def arrive(self, x: int, y: int, z: int):
        """移动到指定坐标处"""

        params = bytearray(10)
        pack_into("!B", params, 0, 0x03)
        pack_into("!h", params, 1, x)
        pack_into("!h", params, 3, y)
        pack_into("!h", params, 5, z)
        cmd = self.join_cmd(CmdType.SINGLE_CONTROL, params)
        print("arrive", cmd.hex(' '))
        self.sendCommand(cmd)

        pass

    def rotate(self, degree: int):
        """顺时旋转指定角度"""

        params = bytearray(10)
        pack_into("!B", params, 0, 0x05)
        pack_into("!h", params, 1, degree)
        cmd = self.join_cmd(CmdType.SINGLE_CONTROL, params)
        print("rotate", cmd.hex(' '))
        self.sendCommand(cmd)

        pass

    def cw(self, degree: int):
        """顺时针旋转指定角度"""
        if degree < 0:
            raise ValueError("degree illegal", degree)
        self.rotate(degree)
        pass

    def ccw(self, degree: int):
        """逆时针旋转指定角度"""
        if degree < 0:
            raise ValueError("degree illegal", degree)
        self.rotate(-degree)
        pass

    def speed(self, speed: int):
        """设置飞行速度"""
        if speed < 0 or speed > 200:
            raise ValueError("speed illegal", speed)

        params = bytearray(10)
        pack_into("!B", params, 0, 0x06)
        pack_into("!h", params, 1, speed)
        cmd = self.join_cmd(CmdType.SINGLE_CONTROL, params)
        print("speed", cmd.hex(' '))
        self.sendCommand(cmd)

        pass

    def high(self, high: int):
        """移动到指定高度处"""

        params = bytearray(10)
        pack_into("!B", params, 0, 0x07)
        pack_into("!h", params, 1, high)
        cmd = self.join_cmd(CmdType.SINGLE_CONTROL, params)
        print("high", cmd.hex(' '))
        self.sendCommand(cmd)

        pass

    def airplane_mode(self, mode: int):
        """
        The airplane_mode function is used to set the airplane mode on the airplane.
        The function takes one parameter, which is an integer that represents a specific state for airplane mode.
        1 - Normal Mode (default)
        2 - Line follow Mode
        3 - Follow Mode
        4 - Single Mode

        设置无人机飞行模式
        :param mode: 1常规2巡线3跟随4单机编队 通常情况下使用模式4

        :param self: Access attributes of the class
        :param mode:int: Specify which mode the airplane should be set to
        :doc-author: Trelent
        """
        if mode < 0 or mode > 5:
            raise ValueError("mode illegal", mode)

        params = bytearray(10)
        pack_into("!B", params, 0, 0x09)
        pack_into("!B", params, 1, mode)
        cmd = self.join_cmd(CmdType.SINGLE_CONTROL, params)
        print("airplane_mode", cmd.hex(' '))
        self.sendCommand(cmd)

        pass

    def vision_mode(self, mode: int):
        """
        The vision_mode function is used to set the vision detect mode on the airplane.
        The function takes one parameter, which is an integer that represents a specific state:
        1 - Point detect Mode
        2 - Line detect Mode
        3 - Tag detect Mode
        4 - Qrcode detect Mode
        5 - LineCode detect Mode

        设置视觉工作模式
        :param mode: 1点检测2线检测3标签检测4二维码扫描5条形码扫描

        :param self: Access the object's attributes and methods
        :param mode:int: Set the airplane mode to 1, 2, 3, 4 or 5
        :doc-author: Jeremie
        """
        if mode < 0 or mode > 5:
            raise ValueError("mode illegal", mode)

        params = bytearray(10)
        pack_into("!B", params, 0, 0x0A)
        pack_into("!B", params, 1, mode)
        cmd = self.join_cmd(CmdType.SINGLE_CONTROL, params)
        print("vision_mode", cmd.hex(' '))
        self.sendCommand(cmd)

        pass

    def hovering(self, ):

        params = bytearray(10)
        pack_into("!B", params, 0, 0xFE)
        cmd = self.join_cmd(CmdType.SINGLE_CONTROL, params)
        print("hovering", cmd.hex(' '))
        self.sendCommand(cmd)

        pass

    def vision_color(self, L_L: int, L_H: int, A_L: int, A_H: int, B_L: int, B_H: int, ):
        """
        The vision_color function set the airplane to color detect mode, set which color need to detect.
        The function takes in 6 integers, L_L, L_H, A_L, A_H, B_L and B_H.
        the params is Lab color space.
        The first two integers are for Low values and High values of lightness (0-255).
        The next two integers are for Low values and High values of greeness (0-255).
        The last two integers are for Low values and High Values of blueness (0-255).

        颜色检测 检测指定颜色
        L_*/A_*/B_* 为色彩在 Lab 色彩空间上的L/a/b三个色彩通道
        *_L/*_H 为色彩在 Lab 色彩空间上各个的色彩通道的上下限范围

        :param self: Access the variables and methods in the class
        :param L_L:int: Set the lower bound of the lightness value range for a channel
        :param L_H:int: Set the upper bound of the lightness value range for a channel
        :param A_L:int: Set the lower bound of the color range for a channel
        :param A_H:int: Set the upper bound of the color range for a channel
        :param B_L:int: Set the lower bound of the color range for b channel
        :param B_H:int: Set the upper bound of the color range for b channel
        :doc-author: Jeremie
        """

        params = bytearray(10)
        pack_into("!B", params, 0, 0x0A)
        pack_into("!B", params, 1, 0x06)
        pack_into("!B", params, 2, L_L)
        pack_into("!B", params, 3, L_H)
        pack_into("!B", params, 4, A_L)
        pack_into("!B", params, 5, A_H)
        pack_into("!B", params, 6, B_L)
        pack_into("!B", params, 7, B_H)
        cmd = self.join_cmd(CmdType.SINGLE_CONTROL, params)
        print("vision_color", cmd.hex(' '))
        self.sendCommand(cmd)

        pass

    def read_multi_setting(self):
        """
        The read_multi_setting function send a read multi setting request to the drone.
        After call this function, you can read the multi setting.

        :doc-author: Jeremie
        """
        return self.read_setting(0x02)

    def read_single_setting(self):
        """
        The read_single_setting function send a read single setting request to the drone.
        After call this function, you can read the single setting.

        :doc-author: Jeremie
        """
        return self.read_setting(0x04)

    def read_hardware_setting(self):
        """
        The read_hardware_setting function send a read hardware setting request to the device.
        After call this function, you can read the hardware setting.

        :doc-author: Jeremie
        """
        return self.read_setting(0xA0)

    def read_setting(self, mode: int):
        """
        The read_setting function send a read setting request to the device.
        The mode parameter is an integer that specifies which setting to read.


        :param self: Reference the class instance
        :param mode:int: Specify which setting to read
        :doc-author: Jeremie
        """
        params = bytearray(1)
        pack_into("!B", params, 0, mode)
        cmd = self.join_cmd(CmdType.READ_SETTINGS, params)
        print("cmd", cmd.hex(' '))
        self.sendCommand(cmd)

        pass

    pass
