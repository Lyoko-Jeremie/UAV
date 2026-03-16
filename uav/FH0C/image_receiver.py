import dataclasses
import threading
import time
import typing
from struct import pack

if typing.TYPE_CHECKING:
    from . import AirplaneController
    from .CommandConstructor import CmdType


# 协议内容：
# 同时只有一张图在拍照和传输
# ===================================================
# 发送给无人机
#   id  22  count   u8 mode;//1拍照回传、2颜色采集回传、3颜色识别回传	photographMode
# --> 套用原数据包格式：[0x00, 0x16, _order_count(), 0x01]  // 拍照指令
# 已实现在 `send_cap_image()`
# ===================================================
# 无人机拍照后响应
# 0xAA	len	0x0A	"    u8 id;          //哪个编号的飞机回传的数据包 , 0x00
#                        u16 count;      //数据包序号 , 对应 send_cap_image 中的 count
#                        u32 size;       //数据包大小，单位：byte（图片的总大小）
#                        u16 type = 0;   //数据包类型（0图片数据1离线程序2错误信息）"	SUM
# header: [0xAA, len 0x0A, {0x0A]
# payload: [id 1, count 2, size 4, type 2}, checksum 1]
# <-- [0xAA, len 0x0A, {0x0A, id 1, count 2, size 4, type 2}, checksum 1]
# ===================================================
# 发送给无人机的开始发送指令和重发指令
# 根据数据包大小 size 计算分包数量，每包发送 26 字节，并且发送应答帧，这时候的应答帧的mark应该赋值为0
# TODO 重发 0 包是否会导致后续全部重发？重发某个包是否会导致后续全部重发？
# 0xBB	len	0x0A	"    u8 id;          //与数据包开始的id一样（飞机id）
#                        u16 count;      //对应 send_cap_image 中填写的的 count
#                        u32 mark;       //丢包序号"
# [0xBB, len 0x08, {0x0A, id 1, count 2, mark(4)}, checksum 1]
# 开始发送给无人机的指令：
# --> [0xBB, len 0x08, {0x0A, id 1, count 2, (0x00_00_00_00) 4}, checksum 1]  // 让无人机开始发送数据
# --> [0xBB, len 0x08, {0x0A, id 1, count 2, (0xnn_nn_nn_nn) 4}, checksum 1]  // 让无人机重发指定编号 0xnn_nn_nn_nn 的分包
# --> [0xBB, len 0x08, {0x0A, id 1, count 2, (0xFF_FF_FF_FF) 4}, checksum 1]  // 全部传输已完成，无人机删除存储的图像缓存
# 注意，无人机的重传逻辑是，从收到重传指令对应的包编号开始按顺序重传该包以及后续包，在传输到结尾后循环从重传指令处再次开始按顺序传输，直到5秒内仍然没有收到结束指令时超时，或在收到结束指令时结束
# 也就意味着，发送0开始传输，一直循环传输0-N，当发送开始/重传指令包M时，一直循环传输M-N。
# ===================================================
# 无人机真实发送数据包的格式：
# 0xAA	len	0x0B	"    u16 units;      //包序号
#                        u8 buff[26];    //数据"	SUM
# header: [0xAA, len 0x1D, {0x0B]
# payload: [units 2, buff 26, checksum 1]
# <-- [0xAA, len 0x1D, {0x0B, units 2, buff 26}, checksum 1]
# 其中 units 是分包的序号，从 0 开始递增，buff 是分包的数据内容， 最后一个分包的 buff 不足 26 字节时，多余空间不用管，jpg解析会忽略它。
# 总 units 数量 = ceil(size / 26)，当 units == ceil(size / 26) - 1 时，说明是最后一个分包
# TODO 丢包重发的逻辑：是否需要使用超时时间和数据包序号跳变来检测是否丢包？
# ===================================================
# 飞机


@dataclasses.dataclass
class ImageInfo:
    count_cmd_id: int
    total_size: int = 0
    total_packets: int = 0
    # dict[packet index , tuple(packet checksum, packet data)]
    # every packet is 26 bytes
    packet_cache: dict[int, tuple[int, bytes]] = dataclasses.field(default_factory=dict)
    # jpg formatted image data
    image_data: bytes = b""
    # 已请求重传的块索引，避免短时间内重复请求
    requested_packets: typing.Set[int] = dataclasses.field(default_factory=set)
    # 最大已收到的块索引，用于乱序检测
    max_received_index: int = -1
    # 最后收到块的时间，用于超时检测
    last_packet_time: float = dataclasses.field(default_factory=time.time)
    # 传输开始时间，用于总超时检测
    start_time: float = dataclasses.field(default_factory=time.time)


class ImageReceiver:
    airplane: AirplaneController
    # 任何时候只存在一张图片
    image_instance: ImageInfo | None = None
    # 超时检测定时器
    _timeout_timer: typing.Optional[threading.Timer]
    # 包超时时间（秒），每个包约20ms，设置为300ms（约10个包的时间）可容忍一定波动
    PACKET_TIMEOUT: float = 0.3
    # 乱序容忍阈值：收到的包索引比期望索引大于此值时才认为丢包
    # 由于包间有业务数据干扰，允许轻微乱序（约3个包）
    OUT_OF_ORDER_THRESHOLD: int = 3
    # 总超时时间（秒），超过此时间强制结束（丢包情况下约5秒）
    TOTAL_TIMEOUT: float = 6.0
    # 接收完成的图片表
    received_image_cache: dict[int, ImageInfo]

    def __init__(self, airplane: AirplaneController):
        self.received_image_cache = {}
        self.airplane = airplane

    def _check_sum1(self, data: bytearray):
        """
        impl checksum algorithm
        """
        return bytearray([sum(data) & 0xFF])

    def _check_sum2(self, header: bytearray, params: bytearray):
        """
        impl checksum algorithm
        """
        return bytearray([sum(header + params) & 0xFF])

    def on_receive_image_pack_info(
            self,
            _fly_id: int,
            photo_count_cmd_id: int,
            total_size: int,
            data_type: int,
            origin_data: bytearray,
    ):
        """接收到无人机拍照回传的图片信息包，包含图片的总大小和数据包数量等信息"""
        if data_type != 0:
            # skip none-image data
            return
        total_packets = (total_size + 25) // 26  # 每包26字节，向上取整
        if self.image_instance is None:
            # TODO clean it
            self._clean_remote_image()
            return
        if self.image_instance.count_cmd_id != photo_count_cmd_id:
            # TODO clean it
            self._clean_remote_image()
            return
        self.image_instance.total_size = total_size
        self.image_instance.total_packets = total_packets
        # 启动超时检测定时器
        # TODO
        self._start_time_received()
        print(
            f"Received image info: photo_count_cmd_id={photo_count_cmd_id}, total_size={total_size}, total_packets={total_packets}")
        pass

    def on_receive_image_packet_data(
            self,
            size_len: int,
            packet_id: int,
            buff: bytes,
            origin_data: bytearray,
    ):
        if self.image_instance is None:
            # TODO clean it
            return
        # TODO
        pass

    def _when_received_end(self):
        self._clean_remote_image()
        # TODO clean
        self.image_instance = None
        pass

    def _start_time_received(self):
        cc = self.airplane.s.ss
        (order_count, cmd) = self._send_transfer_pack(0x00_00_00_00)
        print("_start_time_received", order_count, cmd.hex(' '))
        cc.sendCommand(cmd)
        pass

    def _re_transfer_pack(self, pack_id: int):
        cc = self.airplane.s.ss
        (order_count, cmd) = self._send_transfer_pack(pack_id)
        print("_re_transfer_pack", order_count, cmd.hex(' '))
        cc.sendCommand(cmd)
        pass

    def _clean_remote_image(self):
        cc = self.airplane.s.ss
        (order_count, cmd) = self._send_transfer_pack(0xFF_FF_FF_FF)
        print("_clean_remote_image", order_count, cmd.hex(' '))
        cc.sendCommand(cmd)
        pass

    def _send_transfer_pack(self, pack_id: int):
        """
        发送传输指令
        pack_id: 0xNN_NN_NN_NN 从某个包编号开始循环传输
        pack_id: 0x00_00_00_00 开始传输，（开始从0循环传输）
        pack_id: 0xFF_FF_FF_FF 结束传输
        """
        cc = self.airplane.s.ss
        # [0xBB, len 0x08, {0x0A, id 1, count 2, (0xFF_FF_FF_FF) 4}, checksum 1]
        order_count = cc.order_count()
        cmd = bytearray([
            0xBB, 0X08,
            0x0A, 0x00,
            pack("<h", order_count),  # Int16LE
            pack("<I", pack_id),  # UInt32LE
        ])
        checksum = self._check_sum1(cmd)
        cmd = cmd + checksum
        return (order_count, cmd)

    def send_cap_image(self):
        """
        无人机拍照指令。此指令触发无人机拍照。后续的传输和接收逻辑在 ImageReceiver 中实现。
        :return:
        """
        cc = self.airplane.s.ss
        # [0x00, 0x16, _order_count(), 0x01]
        (params, order_count) = cc.build_cmd_params(22, 0x01)
        cmd = cc.join_cmd(CmdType.SINGLE_CONTROL, params)
        print("send_cap_image", order_count, cmd.hex(' '))
        cc.sendCommand(cmd)
        self.image_instance = ImageInfo(
            count_cmd_id=order_count,
        )
        pass
