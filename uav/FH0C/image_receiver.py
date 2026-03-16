from __future__ import annotations

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


# send_cap_image()
#     → [等待无人机响应]
# on_receive_image_pack_info()
#     → 重置 start_time
#     → _send_start_received()
#     → _start_timeout_timer()  ← 此处启动超时定时器
#     → [等待数据包]
# on_receive_image_packet_data()
#     → 每次收到包都会重置定时器


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
        self._timeout_timer = None
        self.image_instance = None

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
            # clean it
            self._clean_remote_image()
            return
        if self.image_instance.count_cmd_id != photo_count_cmd_id:
            # clean it
            self._clean_remote_image()
            return
        self.image_instance.total_size = total_size
        self.image_instance.total_packets = total_packets
        # 重置传输开始时间（用于总超时计算）
        self.image_instance.start_time = time.time()
        # 发送开始传输指令
        self._send_start_received()
        # 启动超时检测定时器，防止第一个数据包丢失或无人机无响应
        self._start_timeout_timer()
        print(
            f"Received image info: photo_count_cmd_id={photo_count_cmd_id}, total_size={total_size}, total_packets={total_packets}")

    def on_receive_image_packet_data(
            self,
            size_len: int,
            packet_id: int,
            buff: bytes,
            origin_data: bytearray,
    ):
        """
        接收到实际的图片数据包
        :param size_len: header中的第二个字段 len 的值，正常情况下为 0x1D ，由于最后一个包可能不足26字节，实际数据长度需要根据 size_len 和 buff 计算得出
        :param packet_id:
        :param buff:
        :param origin_data:
        :return:
        """
        if self.image_instance is None:
            # 没有正在进行的图片传输
            return
        if self.image_instance.total_packets == 0:
            # 还没有收到图片信息包
            return

        # 更新最后收到包的时间
        self.image_instance.last_packet_time = time.time()

        # 计算数据包的校验和（用于去重检测）
        packet_checksum = sum(origin_data) & 0xFF

        # 检查是否是重复包
        if packet_id in self.image_instance.packet_cache:
            existing_checksum, _ = self.image_instance.packet_cache[packet_id]
            if existing_checksum == packet_checksum:
                # 完全相同的包，忽略
                return

        # 存储数据包
        self.image_instance.packet_cache[packet_id] = (packet_checksum, bytes(buff))

        # 更新最大已收到的包索引
        if packet_id > self.image_instance.max_received_index:
            self.image_instance.max_received_index = packet_id

        # 从请求重传列表中移除已收到的包
        self.image_instance.requested_packets.discard(packet_id)

        print(f"Received packet {packet_id}/{self.image_instance.total_packets - 1}, "
              f"cache size: {len(self.image_instance.packet_cache)}")

        # 检查丢包：检测是否有跳过的包
        self._check_and_request_missing_packets(packet_id)

        # 检查是否已接收完所有包
        if len(self.image_instance.packet_cache) >= self.image_instance.total_packets:
            # 验证是否真的收齐了所有包
            if self._verify_all_packets_received():
                self._assemble_image()
                self._when_received_end()
        else:
            # 启动/重置超时检测定时器
            self._start_timeout_timer()

    def _check_and_request_missing_packets(self, current_packet_id: int):
        """检查并请求丢失的数据包"""
        if self.image_instance is None:
            return

        # 找出已收到的最大包索引之前缺失的包
        missing_packets = []
        for i in range(self.image_instance.max_received_index):
            if i not in self.image_instance.packet_cache:
                # 检查是否已经请求过重传且还在等待中
                if i not in self.image_instance.requested_packets:
                    missing_packets.append(i)

        # 如果发现丢包，且当前包索引比期望的下一个包大于阈值，请求重传
        expected_next = len(self.image_instance.packet_cache)
        if missing_packets and (current_packet_id - expected_next) >= self.OUT_OF_ORDER_THRESHOLD:
            # 请求从第一个丢失的包开始重传
            first_missing = min(missing_packets)
            self._re_transfer_pack(first_missing)
            # 标记所有缺失的包为已请求
            for p in missing_packets:
                self.image_instance.requested_packets.add(p)

    def _verify_all_packets_received(self) -> bool:
        """验证是否收齐了所有数据包"""
        if self.image_instance is None:
            return False
        for i in range(self.image_instance.total_packets):
            if i not in self.image_instance.packet_cache:
                return False
        return True

    def _assemble_image(self):
        """将所有数据包组装成完整的图片"""
        if self.image_instance is None:
            return

        # 按包序号顺序组装数据
        image_data = bytearray()
        for i in range(self.image_instance.total_packets):
            if i in self.image_instance.packet_cache:
                _, packet_data = self.image_instance.packet_cache[i]
                image_data.extend(packet_data)

        # 裁剪到实际图片大小（最后一个包可能有多余数据）
        self.image_instance.image_data = bytes(image_data[:self.image_instance.total_size])

        # 存储到已接收图片缓存
        self.received_image_cache[self.image_instance.count_cmd_id] = self.image_instance

        print(f"Image assembled successfully! Size: {len(self.image_instance.image_data)} bytes, "
              f"count_cmd_id: {self.image_instance.count_cmd_id}")

    def _start_timeout_timer(self):
        """启动或重置超时检测定时器"""
        # 取消现有定时器
        if self._timeout_timer is not None:
            self._timeout_timer.cancel()

        # 创建新的定时器
        self._timeout_timer = threading.Timer(self.PACKET_TIMEOUT, self._on_packet_timeout)
        self._timeout_timer.daemon = True
        self._timeout_timer.start()

    def _stop_timeout_timer(self):
        """停止超时检测定时器"""
        if self._timeout_timer is not None:
            self._timeout_timer.cancel()
            self._timeout_timer = None

    def _on_packet_timeout(self):
        """包接收超时处理"""
        if self.image_instance is None:
            return

        current_time = time.time()

        # 检查总超时
        if current_time - self.image_instance.start_time > self.TOTAL_TIMEOUT:
            print(f"Total timeout reached! Received {len(self.image_instance.packet_cache)}/{self.image_instance.total_packets} packets")
            # 尝试组装已有数据（可能不完整）
            if len(self.image_instance.packet_cache) > 0:
                self._assemble_image()
            self._when_received_end()
            return

        # 检查包超时 - 请求重传丢失的包
        missing_packets = []
        for i in range(self.image_instance.total_packets):
            if i not in self.image_instance.packet_cache:
                missing_packets.append(i)

        if missing_packets:
            # 请求从第一个丢失的包开始重传
            first_missing = min(missing_packets)
            print(f"Packet timeout! Missing {len(missing_packets)} packets, requesting retransmit from {first_missing}")
            self._re_transfer_pack(first_missing)
            # 重新启动超时定时器
            self._start_timeout_timer()
        else:
            # 所有包已收到
            self._assemble_image()
            self._when_received_end()

    def _when_received_end(self):
        """当接收全部结束后，进行清理工作"""
        # 停止超时定时器
        self._stop_timeout_timer()
        # 清理远端缓存
        self._clean_remote_image()
        # 清空当前图片实例
        self.image_instance = None

    def _send_start_received(self):
        """发送开始传输指令"""
        cc = self.airplane.s.ss
        (order_count, cmd) = self._send_transfer_pack(0x00_00_00_00)
        print("_start_time_received", order_count, cmd.hex(' '))
        cc.sendCommand(cmd)
        pass

    def _re_transfer_pack(self, pack_id: int):
        """发送从指定包开始重传的指令"""
        cc = self.airplane.s.ss
        (order_count, cmd) = self._send_transfer_pack(pack_id)
        print("_re_transfer_pack", order_count, cmd.hex(' '))
        cc.sendCommand(cmd)
        pass

    def _clean_remote_image(self):
        """在传输结束，全部接收完毕后，发送指令清除远端数据"""
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
        # [0xBB, len 0x08, {0x0A, id 1, count 2, (0xFF_FF_FF_FF) 4}, checksum 1]
        if self.image_instance is None:
            # bad state , never go there
            print("Warning: No order_count , image_instance in bad state")
            pass
        order_count = self.image_instance.count_cmd_id if self.image_instance else 0
        cmd = bytearray([0xBB, 0X08, 0x0A, 0x00])
        cmd.extend(pack("<H", order_count))  # Int16LE (count)
        cmd.extend(pack("<I", pack_id))  # UInt32LE (pack_id)
        checksum = self._check_sum1(cmd)
        cmd = cmd + checksum
        return (order_count, cmd)

    def send_cap_image(self):
        """
        无人机拍照指令。此指令触发无人机拍照。
        :return: 拍照指令的 order_count，用于后续匹配图片数据
        """
        # 检查是否有正在进行的传输
        if self.image_instance is not None:
            print("Warning: Previous image transfer still in progress")
            return None
        
        cc = self.airplane.s.ss
        # [0x00, 0x16, _order_count(), 0x01]
        from .CommandConstructor import CmdType
        (params, order_count) = cc.build_cmd_params(22, 0x01)
        cmd = cc.join_cmd(CmdType.SINGLE_CONTROL, params)
        print("send_cap_image", order_count, cmd.hex(' '))
        
        # 先创建 ImageInfo 实例
        self.image_instance = ImageInfo(
            count_cmd_id=order_count,
        )
        
        # 然后发送命令
        cc.sendCommand(cmd)
        return order_count

    def get_image(self, count_cmd_id: int) -> typing.Optional[bytes]:
        """
        获取已接收完成的图片数据
        :param count_cmd_id: 拍照指令的 order_count
        :return: 图片的二进制数据（JPG格式），如果不存在则返回 None
        """
        if count_cmd_id in self.received_image_cache:
            return self.received_image_cache[count_cmd_id].image_data
        return None

    def get_latest_image(self) -> typing.Optional[bytes]:
        """
        获取最新接收完成的图片数据
        :return: 图片的二进制数据（JPG格式），如果没有图片则返回 None
        """
        if not self.received_image_cache:
            return None
        latest_key = max(self.received_image_cache.keys())
        return self.received_image_cache[latest_key].image_data

    def is_transfer_in_progress(self) -> bool:
        """
        检查是否有正在进行的图片传输
        :return: True 如果有正在进行的传输
        """
        return self.image_instance is not None

    def get_transfer_progress(self) -> typing.Optional[tuple[int, int]]:
        """
        获取当前传输进度
        :return: (已接收包数量, 总包数量) 元组，如果没有传输则返回 None
        """
        if self.image_instance is None:
            return None
        return (len(self.image_instance.packet_cache), self.image_instance.total_packets)

    def save_image_to_file(self, count_cmd_id: int, file_path: str) -> bool:
        """
        将图片保存到文件
        :param count_cmd_id: 拍照指令的 order_count
        :param file_path: 保存的文件路径
        :return: True 如果保存成功
        """
        image_data = self.get_image(count_cmd_id)
        if image_data is None:
            return False
        try:
            with open(file_path, 'wb') as f:
                f.write(image_data)
            return True
        except IOError as e:
            print(f"Failed to save image: {e}")
            return False

    def save_latest_image_to_file(self, file_path: str) -> bool:
        """
        将最新接收的图片保存到文件
        :param file_path: 保存的文件路径
        :return: True 如果保存成功
        """
        image_data = self.get_latest_image()
        if image_data is None:
            return False
        try:
            with open(file_path, 'wb') as f:
                f.write(image_data)
            return True
        except IOError as e:
            print(f"Failed to save image: {e}")
            return False

    def clear_image_cache(self, count_cmd_id: typing.Optional[int] = None):
        """
        清除图片缓存
        :param count_cmd_id: 如果指定则只清除该图片，否则清除所有图片
        """
        if count_cmd_id is not None:
            if count_cmd_id in self.received_image_cache:
                del self.received_image_cache[count_cmd_id]
        else:
            self.received_image_cache.clear()

    def cancel_current_transfer(self):
        """
        取消当前正在进行的传输
        """
        if self.image_instance is not None:
            self._stop_timeout_timer()
            self._clean_remote_image()
            self.image_instance = None
