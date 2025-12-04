"""
这个文件负责解析从飞机串口发来的二进制数据
"""
from queue import Queue
from struct import pack, unpack, pack_into, unpack_from
from typing import List, Dict, Any, Tuple, Union, Literal
from dataclasses import dataclass
from threading import Lock

Header_Base_Info = b'\xAA\x0D\x07'
Header_Vision_Sensor_info = b'\xAA\x19\x30'
Header_Sensor_info = b'\xAA\x14\x01'
Header_Others = b'\xAA\x09\xf1'
Header_Others_Hardware_Info = b'\xAA\x00\x00'
Header_Others_MultiSetting_Info = b'\xAA\x00\x04'
Header_Others_SingleSetting_Info = b'\xAA\x00\x05'

Header_Fh0cBase = b'\xAA\x1b\x01'


#  typedef struct
#  {
#      u8 id;           //编号
#      u8 vol;          //电压(例如：37表示3.7V)
#      u8 ssi;          //信号强度
#      u16 state;       //传感器状态
#      u16 setting;     //设置状态
#
#      //7
#      struct {
#          u8 flag;
#          u16 tagId;   // 检测到的标签编号
#          s8 x0,y0;
#          s8 x1,y1;
#      }mv;
#      //14
#      struct {
#          s8 qual;     // 光流数据可靠性指数
#          s8 x,y;      // 光流x/y轴数据
#      }flow;
#      //17
#      struct {
#          s16 x,y,z;   // 横滚/俯仰/航向角  度
#      }imu;
#      //23
#      s16 high;
#  }
#  basicSensor_t;
#
@dataclass
class Fh0cBase:
    id: int  # u8       编号
    vol: int  # u8      电压*10 ， 37表示3.7V
    ssi: int  # u8      信号强度
    state: int  # u16   传感器状态
    setting: int  # u16  设置状态

    # mv_flag :
    # if(line.up_ok)    basicSensor.mv.flag |= BIT0;
    # if(line.down_ok)  basicSensor.mv.flag |= BIT1;
    # if(line.left_ok)  basicSensor.mv.flag |= BIT2;
    # if(line.right_ok) basicSensor.mv.flag |= BIT3;
    # if(tag.raw.flag)  basicSensor.mv.flag |= BIT4;
    # if(dot.ok)        basicSensor.mv.flag |= BIT5;
    # if(qr.ok)         basicSensor.mv.flag |= BIT6;
    # if(br.ok)         basicSensor.mv.flag |= BIT7;
    mv_flag: int  # u8
    mv_tagId: int  # u16
    mv_x0: int  # s8
    mv_y0: int  # s8
    mv_x1: int  # s8
    mv_y1: int  # s8

    flow_qual: int  # s8
    flow_x: int  # s8
    flow_y: int  # s8

    imu: Tuple[int, int, int]  # s16 x,y,z
    high: int  # s16 高度


@dataclass
class BaseInfo:
    """
    飞机以及遥控器基本数据信息

    ...
    Attributes
    ----------
    fly_id: int
        # 编队模式下的飞机编号，如果是单机模式则固定是0xFF。
    hardwareType: int
        # 硬件类型，固定是1，代表小四轴无人机
    fly_voltage: int
        # 飞机电池电压
    rmt_voltages16: int
        # 遥控电池电压
    nrf_ssi: int
        # 无人机收到遥控器数据的帧数
    self_test_flag: int
        # 传感器自检状态
    setting_flag: int
        # 当前设置状态
    """
    fly_id: int  # 编队模式下的飞机编号，如果是单机模式则固定是0xFF。
    hardwareType: int  # 硬件类型，固定是1，代表小四轴无人机
    fly_voltage: int  # 飞机电池电压
    rmt_voltages16: int  # 遥控电池电压
    nrf_ssi: int  # 无人机收到遥控器数据的帧数
    self_test_flag: int  # 传感器自检状态
    setting_flag: int  # 当前设置状态


@dataclass
class SensorInfo:
    """
    飞机传感器信息

    ...
    Attributes
    ----------
    lock_flag: int
        # 0上锁状态1解锁状态
    reserved0: int
        # 预留

    rol: int
        # 横滚角，单位：度
    pit: int
        # 俯仰角，单位：度
    yaw: int
        # 航向角，单位：度
    high: int
        # 高度，单位：厘米

    id: int
        # 固定是0，目前没什么意义

    loc_x: int
        # 当前x坐标，单位：厘米
    loc_y: int
        # 当前y坐标，单位：厘米
    reserved1: int
        # 预留
    """
    lock_flag: int  # 0上锁状态1解锁状态
    reserved0: int  # 预留

    rol: int  # 横滚角，单位：度
    pit: int  # 俯仰角，单位：度
    yaw: int  # 航向角，单位：度
    high: int  # 高度，单位：厘米

    id: int  # 固定是0，目前没什么意义

    loc_x: int  # 当前x坐标，单位：厘米
    loc_y: int  # 当前y坐标，单位：厘米
    reserved1: int  # 预留


@dataclass
class VisionSensorInfo:
    """
    飞机视觉传感器信息

    ...
    Attributes
    ----------
    flow_x: int
        # 光流x轴数据
    flow_y: int
        # 光流y轴数据
    flow_qualt: int
        # 光流数据可靠性指数

    # 以下内容仅在 点检测 模式下才有数据
    dot_x: int
        # 检测到点的x轴坐标
    dot_y: int
        # 检测到点的y轴坐标
    is_dot_ok: int
        # 是否检测到点，非零表示检测到了。
        # those are only contain valid values in line detect mode, you need call `CommandConstructor.vision_mode(1)` to enable it.

    # 以下内容仅在 线检测 模式下才有数据
    line_x: int
        # 检测到竖线的坐标
    line_y: int
        # 检测到横线的坐标
    line_x_angle: int
        # 检测到竖线的倾角
    line_y_angle: int
        # 检测到横线的倾角
    line_flag: int
        # 是否检测到线的标志
        # （bit0=1表示前方检测到线，bit1=1表示后方检测到线，bit2=1表示左方检测到线，bit3=1表示右方检测到线）
        # those are only contain valid values in line detect mode, you need call `CommandConstructor.vision_mode(2)` to enable it.

    # 以下内容仅在 标签检测 模式下才有数据
    tag_id: int
        # 检测到的标签编号
    is_tag_ok: int
        # 是否检测到标签，非零表示检测到了。

    mv_mode: int
        # 当前视觉模块的工作模式

    obsDir: int
        # 非零表示检测到了障碍物
    """
    flow_x: int  # 光流x轴数据
    flow_y: int  # 光流y轴数据
    flow_qualt: int  # 光流数据可靠性指数

    dot_x: int  # 检测到点的x轴坐标
    dot_y: int  # 检测到点的y轴坐标
    is_dot_ok: int  # 是否检测到点，非零表示检测到了。

    line_x: int  # 检测到竖线的坐标
    line_y: int  # 检测到横线的坐标
    line_x_angle: int  # 检测到竖线的倾角
    line_y_angle: int  # 检测到横线的倾角
    line_flag: int  # 是否检测到线的标志
    # （bit0=1表示前方检测到线，bit1=1表示后方检测到线，bit2=1表示左方检测到线，bit3=1表示右方检测到线）

    tag_id: int  # 检测到的标签编号
    is_tag_ok: int  # 是否检测到标签，非零表示检测到了。

    mv_mode: int  # 当前视觉模块的工作模式

    obsDir: int  # 非零表示检测到了障碍物


@dataclass
class HardwareInfo:
    """
    飞机硬件信息

    ...
    Attributes
    ----------
    hardtype: int
        # 硬件类型(0遥控1飞机)
    hardware: int
        # 硬件版本(例如：software=400，表示V4.0.0)
    software: int
        # 软件版本(固件更新日期，例如：20200101)
    iap_ware: int
        # IAP版本(iap_ware=100，表示V1.0.0)
    """
    hardtype: int  # 硬件类型(0遥控1飞机)
    hardware: int  # 硬件版本(例如：software=400，表示V4.0.0)
    software: int  # 软件版本(固件更新日期，例如：20200101)
    iap_ware: int  # IAP版本(iap_ware=100，表示V1.0.0)


@dataclass
class MultiSettingInfo:
    """
    多机编队模式设置

    ...
    Attributes
    ----------
    mode: int
        # 0单机模式1编队模式
    id: int
        # 飞机编号
    channel: int
        # 通信信道0~125
    addr: int
        # 通信地址
    """
    mode: int  # 0单机模式1编队模式
    id: int  # 飞机编号
    channel: int  # 通信信道0~125
    addr: int  # 通信地址


@dataclass
class SingleSettingInfo:
    """
    单机模式设置

    ...
    Attributes
    ----------
    mode: int
        # 0低速1中速2高速
    channel: int
        # 通信信道0~125
    addr: int
        # 通信地址（多台机子同时在单机模式下工作必须要把信道和地址设置为不一样）
    """
    mode: int  # 0低速1中速2高速
    channel: int  # 通信信道0~125
    addr: int  # 通信地址（多台机子同时在单机模式下工作必须要把信道和地址设置为不一样）


class ReadDataParser:
    """
    this class Parse data that comes from serial port.
    这个类负责解析从串口发来的数据
    """

    read_buffer: bytearray = bytearray()
    q: Queue = None
    m_base_info: BaseInfo = None
    m_sensor_info: SensorInfo = None
    m_fh0c_base: Fh0cBase = None
    m_vision_sensor_info: VisionSensorInfo = None
    m_hardware_info: HardwareInfo = None
    m_multi_setting_info: MultiSettingInfo = None
    m_single_setting_info: SingleSettingInfo = None
    m_info_lock: Lock = Lock()

    def __init__(self, q_read):
        self.q = q_read
        pass

    def push(self, data: Union[bytearray, bytes]):
        """
        this method are used by `task_read` thread, it add new bytearray comes from serial port.
        """
        self.read_buffer = self.read_buffer + data
        self.try_parse()
        pass

    def try_parse(self):
        """
        this method do the core task that split bytearray buffer stream into packages.
        """
        print(self.read_buffer)
        while len(self.read_buffer) > 3:
            header = self.read_buffer[0:3]
            size = header[1]
            if len(self.read_buffer) <= size + 3:
                break

            # print("header", header, size, header[0], header[1], header[2])
            if header == Header_Base_Info:
                data = self.read_buffer[0: size + 3]
                # print("Header_Base_Info", 0, size, len(data), data)
                self.base_info(data)
                pass
            elif header == Header_Vision_Sensor_info:
                data = self.read_buffer[0: size + 3]
                # print("Header_Vision_Sensor_info", 0, size, len(data), data)
                self.vision_sensor_info(data)
                pass
            elif header == Header_Sensor_info:
                data = self.read_buffer[0: size + 3]
                # print("Header_Sensor_info", 0, size, len(data), data)
                self.sensor_info(data)
                pass
            elif header == Header_Fh0cBase:
                data = self.read_buffer[0: size + 3]
                # print("Header_Sensor_info", 0, size, len(data), data)
                self.fh0c_base(data)
                pass
            elif header == Header_Others:
                data = self.read_buffer[0: size + 3]
                # print("Header_Others", 0, size, len(data), data)
                self.other(data)
                pass
            elif header[0] == 170:  # b'\xAA':
                flag = header[2]
                data = self.read_buffer[0: size + 3]
                if flag == 0:  # b'\x00':
                    # Header_Others_Hardware_Info like
                    self.hardware_info(data)
                    pass
                elif flag == 4:  # b'\x04':
                    # Header_Others_MultiSetting_Info like
                    self.multi_setting_info(data)
                    pass
                elif flag == 5:  # b'\x05':
                    # Header_Others_SingleSetting_Info like
                    self.single_setting_info(data)
                    pass
                else:
                    # don't care
                    pass
                pass

            self.read_buffer = self.read_buffer[size + 3:]
            # if len(self.read_buffer) > size + 3:
            #     self.read_buffer = self.read_buffer[size + 3:]
            #     self.try_parse()
            #     pass
            pass
        pass

    def base_info(self, data: bytearray):
        # print("Base_Info", data.hex(' '))
        params = data[2:len(data) - 1]
        m_base_info = BaseInfo(
            fly_id=unpack_from("!B", params, 1)[0],
            hardwareType=unpack_from("!B", params, 2)[0],
            fly_voltage=unpack_from("!h", params, 3)[0] * 100,
            rmt_voltages16=unpack_from("!h", params, 5)[0] * 100,
            nrf_ssi=unpack_from("!h", params, 7)[0],
            self_test_flag=unpack_from("!H", params, 9)[0],
            setting_flag=unpack_from("!H", params, 11)[0]
        )
        with self.m_info_lock:
            self.m_base_info = m_base_info
            pass
        # print("self._base_info", m_base_info)
        pass

    def vision_sensor_info(self, data: bytearray):
        # print("vision_sensor_info", data.hex(' '))
        params = data[2:len(data) - 1]
        m_vision_sensor_info = VisionSensorInfo(
            flow_x=unpack_from("!h", params, 1)[0],
            flow_y=unpack_from("!h", params, 3)[0],
            flow_qualt=unpack_from("!B", params, 5)[0],
            dot_x=unpack_from("!h", params, 6)[0],
            dot_y=unpack_from("!h", params, 8)[0],
            is_dot_ok=unpack_from("!B", params, 10)[0],
            line_x=unpack_from("!h", params, 11)[0],
            line_y=unpack_from("!h", params, 13)[0],
            line_x_angle=unpack_from("!h", params, 15)[0],
            line_y_angle=unpack_from("!h", params, 15)[0],
            line_flag=unpack_from("!B", params, 19)[0],
            tag_id=unpack_from("!h", params, 20)[0],
            is_tag_ok=unpack_from("!B", params, 22)[0],
            mv_mode=unpack_from("!B", params, 23)[0],
            obsDir=unpack_from("!B", params, 24)[0],
        )
        with self.m_info_lock:
            self.m_vision_sensor_info = m_vision_sensor_info
            pass
        # print("self._vision_sensor_info", m_vision_sensor_info)
        pass

    def sensor_info(self, data: bytearray):
        # print("sensor_info", data.hex(' '))
        # TODO
        params = data[2:len(data) - 1]
        m_sensor_info = SensorInfo(
            lock_flag=unpack_from("!B", params, 1)[0],
            reserved0=unpack_from("!B", params, 2)[0],
            rol=unpack_from("!h", params, 3)[0] * 100,
            pit=unpack_from("!h", params, 5)[0] * 100,
            yaw=unpack_from("!h", params, 7)[0] * 100,
            high=unpack_from("!i", params, 9)[0] * 100,
            id=unpack_from("!B", params, 13)[0],
            loc_x=unpack_from("!h", params, 14)[0],
            loc_y=unpack_from("!h", params, 16)[0],
            reserved1=unpack_from("!h", params, 18)[0],
        )
        with self.m_info_lock:
            self.m_sensor_info = m_sensor_info
            pass
        # print("self._sensor_info", m_sensor_info)
        pass

    def fh0c_base(self, data: bytearray):
        # print("fh0c_base", data.hex(' '))
        params = data[2:len(data) - 1]
        m_fh0c_base = Fh0cBase(
            id=unpack_from("!B", params, 1)[0],
            vol=unpack_from("!B", params, 2)[0],
            ssi=unpack_from("!B", params, 3)[0],
            state=unpack_from("!H", params, 4)[0],
            setting=unpack_from("!H", params, 6)[0],
            mv_flag=unpack_from("!B", params, 8)[0],
            mv_tagId=unpack_from("!H", params, 9)[0],
            mv_x0=unpack_from("!b", params, 11)[0],
            mv_y0=unpack_from("!b", params, 12)[0],
            mv_x1=unpack_from("!b", params, 13)[0],
            mv_y1=unpack_from("!b", params, 14)[0],
            flow_qual=unpack_from("!b", params, 15)[0],
            flow_x=unpack_from("!b", params, 16)[0],
            flow_y=unpack_from("!b", params, 17)[0],
            imu=(unpack_from("!h", params, 18)[0],
                 unpack_from("!h", params, 20)[0],
                 unpack_from("!h", params, 22)[0]),
            high=unpack_from("!h", params, 24)[0],
        )
        with self.m_info_lock:
            self.m_fh0c_base = m_fh0c_base
            pass
        print("self._fh0c_base", m_fh0c_base)
        pass

    def other(self, data: bytearray):
        # print("other", data.hex(' '))
        # empty
        pass

    def hardware_info(self, data: bytearray):
        # print("hardware_info", data.hex(' '))
        params = data[2:len(data) - 1]
        m_hardware_info = HardwareInfo(
            hardtype=unpack_from("!B", params, 1)[0],
            hardware=unpack_from("!H", params, 2)[0],
            software=unpack_from("!I", params, 4)[0],
            iap_ware=unpack_from("!H", params, 8)[0],
        )
        with self.m_info_lock:
            self.m_hardware_info = m_hardware_info
            pass
        print("self._hardware_info", m_hardware_info)
        pass

    def multi_setting_info(self, data: bytearray):
        # print("multi_setting_info", data.hex(' '))
        # TODO
        params = data[2:len(data) - 1]
        m_multi_setting_info = MultiSettingInfo(
            mode=unpack_from("!B", params, 1)[0],
            id=unpack_from("!B", params, 2)[0],
            channel=unpack_from("!B", params, 3)[0],
            addr=unpack_from("!I", params, 4)[0],
        )
        with self.m_info_lock:
            self.m_multi_setting_info = m_multi_setting_info
            pass
        print("self._multi_setting_info", m_multi_setting_info)
        pass

    def single_setting_info(self, data: bytearray):
        # print("single_setting_info", data.hex(' '))
        # TODO
        params = data[2:len(data) - 1]
        m_single_setting_info = SingleSettingInfo(
            mode=unpack_from("!B", params, 1)[0],
            channel=unpack_from("!B", params, 2)[0],
            addr=unpack_from("!I", params, 3)[0],
        )
        with self.m_info_lock:
            self.m_single_setting_info = m_single_setting_info
            pass
        print("self._single_setting_info", m_single_setting_info)
        pass

    def get_base_info(self):
        with self.m_info_lock:
            return self.m_base_info

    def get_vision_sensor_info(self):
        with self.m_info_lock:
            return self.m_vision_sensor_info

    def get_sensor_info(self):
        with self.m_info_lock:
            return self.m_sensor_info

    def get_fh0c_base(self):
        with self.m_info_lock:
            return self.m_fh0c_base

    def get_hardware_info(self):
        with self.m_info_lock:
            return self.m_hardware_info

    def get_multi_setting_info(self):
        with self.m_info_lock:
            return self.m_multi_setting_info

    def get_single_setting_info(self):
        with self.m_info_lock:
            return self.m_single_setting_info

    pass
