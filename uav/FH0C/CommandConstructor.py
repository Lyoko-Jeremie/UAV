from queue import Queue
from struct import pack, unpack, pack_into, unpack_from
from enum import Enum

from .QueueSignal import QueueSignal

# 发送数据的协议包定义
# 波特率 500000
# 总包长固定为 32 Byte
#
# HEAD  LEN        FUN      order[13]   order[13]   volume      reserved    SUM
# 0xBB  0x1D(29)   0xF3     cmd 1       cmd 2       0x64(100)   0x00        sum() & 0xFF
#
# cmd 1 和 cmd 2 内容相同
# 每个 cmd 为以下结构，多余内容填0
#
# ID        CMD[1]      COUNT[1]    PARAM
# 0x00      cmd id      cmd order   cmd param
#


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

        New Protocol (Fixed 32 Byte):
        HEAD  LEN        FUN      order[13]   order[13]   volume      reserved    SUM
        0xBB  0x1D(29)   0xF3     cmd 1       cmd 2       0x64(100)   0x00        sum() & 0xFF

        Each cmd structure (13 bytes):
        ID[1] + CMD[1] + COUNT[1] + PARAM[variable] + padding zeros

        :param self: Access the attributes and methods of the class in python
        :param type:CmdType: Distinguish the different commands
        :param params:bytearray: Pass the parameters of a command
        :return: A bytearray
        :doc-author: Jeremie
        """
        # Fixed header for new protocol
        header = bytearray(b'\xBB\x1D')  # HEAD=0xBB, LEN=0x1D(29)

        # Fixed 29-byte data body
        data_body = bytearray(29)
        data_body[0] = 0xF3  # FUN byte

        # Check param length (should not exceed 13 bytes)
        if len(params) > 13:
            raise ValueError(f"params length {len(params)} exceeds maximum 13 bytes")

        # Copy params to cmd1 position (offset 1)
        data_body[1:1+len(params)] = params

        # Copy params to cmd2 position (offset 1+13=14), cmd1 and cmd2 are identical
        data_body[14:14+len(params)] = params

        # Set volume byte at position 27 (1 + 13 + 13)
        data_body[27] = 0x64  # volume = 100

        # Set reserved byte at position 28
        data_body[28] = 0x00  # reserved = 0

        # Calculate checksum
        checksum = self._check_sum(header, data_body)

        # Construct final command: header + data_body + checksum = 2 + 29 + 1 = 32 bytes
        return header + data_body + checksum

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

    def _build_cmd_params(self, cmd: int, *param_bytes) -> bytearray:
        """
        Build command parameters following the new protocol format:
        ID[1] + CMD[1] + COUNT[1] + PARAM[variable]

        :param cmd: Command ID
        :param param_bytes: Variable parameter bytes to append
        :return: bytearray ready for join_cmd
        """
        params = bytearray()
        params.append(0x00)  # ID byte (always 0x00)
        params.append(cmd & 0xFF)  # CMD byte
        params.append(self._order_count() & 0xFF)  # COUNT byte (order count)

        # Append parameter bytes
        for byte_val in param_bytes:
            if isinstance(byte_val, int):
                params.append(byte_val & 0xFF)
            elif isinstance(byte_val, bytearray) or isinstance(byte_val, bytes):
                params.extend(byte_val)
            else:
                raise TypeError(f"Unsupported parameter type: {type(byte_val)}")

        return params

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

        # Build params: ID + CMD + COUNT + mode + r + g + b
        params = self._build_cmd_params(13, mode, r, g, b)
        cmd = self.join_cmd(CmdType.SINGLE_CONTROL, params)
        print("led", cmd.hex(' '))
        self.sendCommand(cmd)

        pass

    def takeoff(self, high: int, ):
        """起飞到指定高度 单位cm"""
        # Build params: ID + CMD + COUNT + high(Int16LE) + tagDis(50) + isGoTo(0)
        high_bytes = pack("<h", high)  # Int16LE (little-endian signed short)
        params = self._build_cmd_params(0, high_bytes, 50, 0)
        cmd = self.join_cmd(CmdType.SINGLE_CONTROL, params)
        print("takeoff", cmd.hex(' '))
        self.sendCommand(cmd)

        pass

    def land(self, ):
        """降落"""
        # Build params: ID + CMD + COUNT + 0
        params = self._build_cmd_params(254, 0)
        cmd = self.join_cmd(CmdType.SINGLE_CONTROL, params)
        print("land", cmd.hex(' '))
        self.sendCommand(cmd)

        pass

    def move(self, direction: int, distance: int):
        """向某个方向移动多少距离
        :param direction: 移动方向（1前2后3左4右5上6下7↖8↗9↙10↘）
        :param distance: 移动距离 单位cm
        """
        if direction < 1 or direction > 10:
            raise ValueError("direction illegal", direction)

        # Build params: ID + CMD + COUNT + direction(UInt8) + distance(Int16LE)
        distance_bytes = pack("<h", distance)  # Int16LE
        params = self._build_cmd_params(5, direction, distance_bytes)
        cmd = self.join_cmd(CmdType.SINGLE_CONTROL, params)
        print("move", cmd.hex(' '))
        self.sendCommand(cmd)

        pass

    def up(self, distance: int):
        """上升指定距离 单位cm"""
        self.move(0x05, distance)
        pass

    def down(self, distance: int):
        """下降指定距离 单位cm"""
        self.move(0x06, distance)
        pass

    def forward(self, distance: int):
        """前进指定距离 单位cm"""
        self.move(0x01, distance)
        pass

    def back(self, distance: int):
        """后退指定距离 单位cm"""
        self.move(0x02, distance)
        pass

    def left(self, distance: int):
        """左移指定距离 单位cm"""
        self.move(0x03, distance)
        pass

    def right(self, distance: int):
        """右移指定距离 单位cm"""
        self.move(0x04, distance)
        pass

    def flip(self, direction: int, circle: int):
        """做翻转（翻跟头）动作
        :param direction: 翻转方向：1(向前)/2(向后)/3(向左)/4(向右)
        :param circle: 翻转的圈数
        """
        if direction < 1 or direction > 4:
            raise ValueError("direction illegal", direction)
        if circle != 1 and circle != 2:
            raise ValueError("circle illegal", circle)

        # Build params: ID + CMD + COUNT + direction + circle
        params = self._build_cmd_params(12, direction, circle)
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

        # Build params: ID + CMD + COUNT + x(Int16LE) + y(Int16LE) + z(Int16LE)
        x_bytes = pack("<h", x)
        y_bytes = pack("<h", y)
        z_bytes = pack("<h", z)
        params = self._build_cmd_params(9, x_bytes, y_bytes, z_bytes)
        cmd = self.join_cmd(CmdType.SINGLE_CONTROL, params)
        print("arrive", cmd.hex(' '))
        self.sendCommand(cmd)

        pass

    def rotate(self, degree: int):
        """旋转指定角度（正数顺时针，负数逆时针）"""

        # Build params: ID + CMD + COUNT + degree(Int16LE)
        degree_bytes = pack("<h", degree)
        params = self._build_cmd_params(10, degree_bytes)
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

        # Build params: ID + CMD + COUNT + speed(Int16LE)
        speed_bytes = pack("<h", speed)
        params = self._build_cmd_params(2, speed_bytes)
        cmd = self.join_cmd(CmdType.SINGLE_CONTROL, params)
        print("speed", cmd.hex(' '))
        self.sendCommand(cmd)

        pass

    def high(self, high: int):
        """移动到指定高度处"""

        # Build params: ID + CMD + COUNT + high(Int16LE)
        high_bytes = pack("<h", high)
        params = self._build_cmd_params(11, high_bytes)
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
        if mode < 1 or mode > 4:
            raise ValueError("mode illegal", mode)

        # Build params: ID + CMD + COUNT + mode
        params = self._build_cmd_params(1, mode)
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
        if mode < 1 or mode > 5:
            raise ValueError("mode illegal", mode)

        # Build params: ID + CMD + COUNT + mode
        # Note: This command may not be supported in new protocol (commented out in TypeScript)
        params = self._build_cmd_params(0x0A, mode)
        cmd = self.join_cmd(CmdType.SINGLE_CONTROL, params)
        print("vision_mode", cmd.hex(' '))
        self.sendCommand(cmd)

        pass

    def hovering(self, ):

        # Build params: ID + CMD + COUNT + 4 (celibate mode)
        params = self._build_cmd_params(254, 4)
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

        # Build params: ID + CMD + COUNT + 0x06 + L_L + L_H + A_L + A_H + B_L + B_H
        # Note: This command may not be supported in new protocol
        params = self._build_cmd_params(0x0A, 0x06, L_L, L_H, A_L, A_H, B_L, B_H)
        cmd = self.join_cmd(CmdType.SINGLE_CONTROL, params)
        print("vision_color", cmd.hex(' '))
        self.sendCommand(cmd)

        pass

    # def read_multi_setting(self):
    #     """
    #     The read_multi_setting function send a read multi setting request to the drone.
    #     After call this function, you can read the multi setting.
    #
    #     :doc-author: Jeremie
    #     """
    #     return self.read_setting(0x02)
    #
    # def read_single_setting(self):
    #     """
    #     The read_single_setting function send a read single setting request to the drone.
    #     After call this function, you can read the single setting.
    #
    #     :doc-author: Jeremie
    #     """
    #     return self.read_setting(0x04)
    #
    # def read_hardware_setting(self):
    #     """
    #     The read_hardware_setting function send a read hardware setting request to the device.
    #     After call this function, you can read the hardware setting.
    #
    #     :doc-author: Jeremie
    #     """
    #     return self.read_setting(0xA0)
    #
    # def read_setting(self, mode: int):
    #     """
    #     The read_setting function send a read setting request to the device.
    #     The mode parameter is an integer that specifies which setting to read.
    #
    #
    #     :param self: Reference the class instance
    #     :param mode:int: Specify which setting to read
    #     :doc-author: Jeremie
    #     """
    #     params = bytearray(1)
    #     pack_into("!B", params, 0, mode)
    #     cmd = self.join_cmd(CmdType.READ_SETTINGS, params)
    #     print("cmd", cmd.hex(' '))
    #     self.sendCommand(cmd)
    #
    #     pass

    pass
