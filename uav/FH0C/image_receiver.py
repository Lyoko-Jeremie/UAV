from __future__ import annotations

import dataclasses
import threading
import time
import typing
from struct import pack

import numpy as np

from .image_process import image_file_2_mat, write_mat_2_file

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
# 重发某个包会导致后续全部重发
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
# [aa 03 0b ff ff b6] TODO ?????
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

# send_cap_image()           → 发送拍照指令，创建 ImageInfo
#     ↓
# on_receive_image_pack_info() → 收到图片信息，发送开始传输指令
#     ↓
# on_receive_image_packet_data() → 接收数据包，检测丢包
#     ↓
# on_receive_image_packet_data_eof() → 收到 EOF，检查完整性
#     ↓ (如有丢包)              ↓ (完整)
# _re_transfer_pack()        _assemble_image()
#     ↓                          ↓
# 循环等待数据...             _when_received_end()
#                                ↓
#                            _clean_remote_image() → 通知无人机清除缓存

# send_cap_image() → 发送拍照指令，创建 ImageInfo
#     ↓
# on_receive_image_pack_info() → 收到图片信息，发送开始传输指令
#     ↓
# on_receive_image_packet_data() → 接收数据包，更新缓存
#     ↓
# on_receive_image_packet_data_eof() → EOF 信号
#     ├─ 有丢包 → _re_transfer_pack(first_missing) → 继续等待
#     └─ 无丢包 → _assemble_image() → _when_received_end()
#                                         └→ _clean_remote_image() 清除远端缓存

@dataclasses.dataclass
class ImageInfo:
    count_cmd_id: int
    count_cmd_id_from_airplane: int = 0
    total_size: int = 0
    total_packets: int = 0
    # dict[packet index , tuple(packet checksum, packet data)]
    # every packet is 26 bytes
    packet_cache: dict[int, tuple[int, bytes]] = dataclasses.field(default_factory=dict)
    # jpg formatted image data
    image_data: bytes = b""
    # ========== 滑动窗口相关字段 ==========
    # 上次发送重传请求时的 first_missing_packet 值，用于冷却控制
    # -1 表示尚未发送过重传请求
    last_retransmit_at_packet: int = -1
    # 上次发送重传请求的时间，用于避免同一个缺包点被短时间重复请求
    last_retransmit_time: float = 0.0
    # 最后收到块的时间，用于超时检测
    last_packet_time: float = dataclasses.field(default_factory=time.time)
    # 传输开始时间，用于总超时检测
    start_time: float = dataclasses.field(default_factory=time.time)
    # ========== 统计信息字段 ==========
    # 总共收到的包数量（包括重复包）
    total_received_count: int = 0
    # 重复包数量
    duplicate_count: int = 0
    # 重传请求次数
    retransmit_request_count: int = 0
    # 滑动窗口触发的重传次数
    window_triggered_retransmit_count: int = 0
    # 超时触发的重传次数
    timeout_triggered_retransmit_count: int = 0
    # EOF触发的重传次数
    eof_triggered_retransmit_count: int = 0
    # ========== 重发取消相关字段 ==========
    # 当前待取消重发的命令ID（用于cleanSendRetry）
    pending_transfer_cmd_id: typing.Optional[int] = None
    # 期望收到的第一个包的ID（收到后取消重发）
    expected_first_packet: typing.Optional[int] = None
    # ========== 智能重发窗口状态机字段 ==========
    # 当前活跃的重发窗口列表 [(start, end, window_size), ...]
    # 由 smart_retransmission() 写入，由 _advance_retransmit_window_if_needed() 驱动推进
    _retransmit_windows: list = dataclasses.field(default_factory=list)
    # 当前正在处理的窗口索引
    _retransmit_window_idx: int = 0


# =============================================
# 智能重发窗口算法说明
#
# 伪代码：
# 输入: total_packets, received_packet_ids (set), max_packet_id
# 输出: 重发窗口起点列表 window_starts, 每个窗口大小 window_size
#
# 1. 计算缺失包ID列表 missing_ids = [i for i in range(total_packets) if i not in received_packet_ids]
# 2. 若 missing_ids 为空，结束流程
# 3. 将 missing_ids 聚类为连续区间 missing_ranges = [[start1, end1], [start2, end2], ...]
# 4. 对每个区间:
#     a. 计算区间长度 L = end - start + 1
#     b. 计算窗口大小 W = min(L + α, W_max)，其中 α 为经验放大系数，W_max为最大窗口限制
#     c. 将区间起点 start 加入 window_starts，窗口大小 W 加入 window_size
# 5. 顺序依次处理每个窗口:
#     a. 发送重发指令(start)
#     b. 实时监控收到的最大包ID cur_max_id
#     c. 若 cur_max_id >= end 或窗口内包全部收到，立即切换到下一区间
# 6. 最后一个区间补齐后，发送重发最后包ID终止重发
# 7. 若仍有缺失包，重复流程
#
# 窗口大小计算公式：
#   设区间长度为 L，经验放大系数为 α，最大窗口为 W_max：
#   窗口大小 W = min(L + α, W_max)
#   推荐 α 取值：3~10（根据实际丢包率调整）
#   W_max 取值：根据系统内存和协议限制设定
# =============================================

# ========== 智能重发窗口算法实现 ==========
def _cluster_missing_ranges(missing_ids):
    """
    将缺失包ID聚类为连续区间。
    输入: 有序缺失包ID列表
    输出: 区间列表 [[start1, end1], ...]
    """
    if not missing_ids:
        return []
    ranges = []
    start = prev = missing_ids[0]
    for idx in missing_ids[1:]:
        if idx == prev + 1:
            prev = idx
        else:
            ranges.append([start, prev])
            start = prev = idx
    ranges.append([start, prev])
    return ranges

def _compute_window_size(L, alpha=5, W_max=32):
    """
    计算窗口大小，L为区间长度，alpha为经验放大系数，W_max为最大窗口。
    """
    return min(L + alpha, W_max)

def _get_missing_ids(total_packets, received_packet_ids):
    """
    返回所有缺失包ID的有序列表。
    """
    return [i for i in range(total_packets) if i not in received_packet_ids]

class SmartRetransmissionController:
    """
    智能重发窗口调度器。
    """
    def __init__(self, total_packets, received_packet_ids, alpha=5, W_max=32):
        self.total_packets = total_packets
        self.received_packet_ids = set(received_packet_ids)
        self.alpha = alpha
        self.W_max = W_max
        self.missing_ids = _get_missing_ids(total_packets, self.received_packet_ids)
        self.ranges = _cluster_missing_ranges(self.missing_ids)
        self.windows = []  # [(start, end, window_size)]
        for start, end in self.ranges:
            L = end - start + 1
            W = _compute_window_size(L, alpha, W_max)
            self.windows.append((start, end, W))

    def get_windows(self):
        """
        返回所有重发窗口 (start, end, window_size)
        """
        return self.windows

    def update(self, received_packet_ids):
        """
        更新已收到包ID集合和窗口。
        """
        self.received_packet_ids = set(received_packet_ids)
        self.missing_ids = _get_missing_ids(self.total_packets, self.received_packet_ids)
        self.ranges = _cluster_missing_ranges(self.missing_ids)
        self.windows = []
        for start, end in self.ranges:
            L = end - start + 1
            W = _compute_window_size(L, self.alpha, self.W_max)
            self.windows.append((start, end, W))

class ImageReceiver:
    airplane: AirplaneController
    # 任何时候只存在一张圖片
    image_instance: ImageInfo | None = None
    # 超时检测定时器
    _timeout_timer: typing.Optional[threading.Timer]
    # ========== 滑动窗口配置 ==========
    # 包超时时间（秒）- 作为滑动窗口的后备机制，主要处理末尾包场景
    PACKET_TIMEOUT: float = 3.0
    # 总超时时间（秒），超过此时间强制结束
    TOTAL_TIMEOUT: float = 15.0
    # 接收完成的图片表
    # count_cmd_id_from_airplane ImageInfo
    received_image_cache: dict[int, ImageInfo]
    # 临界区锁，保护 image_instance 和相关状态
    _lock: threading.RLock
    # 命令ID计数器，用于生成唯一的 cmd_id_for_clean
    _cmd_id_counter: int

    user_receive_callback: typing.Callable[[bytes], None] | None = None
    user_progress_callback: typing.Callable[[int, int], None] | None = None

    def __init__(self, airplane: AirplaneController):
        self.received_image_cache = {}
        self.airplane = airplane
        self._timeout_timer = None
        self.image_instance = None
        self._lock = threading.RLock()
        self._cmd_id_counter = 0
        self._has_sent_start = False  # 防止重复发送 mark=0 指令

    def _generate_cmd_id(self) -> int:
        """生成唯一的命令ID用于 cleanSendRetry（调用时已持有锁）"""
        self._cmd_id_counter += 1
        return self._cmd_id_counter

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
        print(f"on_receive_image_pack_info: fly_id={_fly_id}, photo_count_cmd_id={photo_count_cmd_id}, "
              f"total_size={total_size}, data_type={data_type}")
        if data_type != 0:
            # skip none-image data
            return

        with self._lock:
            if self.image_instance is None:
                # 没有正在等待的图片传输请求
                return

            total_packets = (total_size + 25) // 26  # 每包26字节，向上取整
            self.image_instance.count_cmd_id_from_airplane = photo_count_cmd_id
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
        with self._lock:
            if self.image_instance is None:
                # 没有正在进行的图片传输
                return
            if self.image_instance.total_packets == 0:
                # 还没有收到图片信息包
                return

            # 原子检查-设置-发送，防止并发下多次补发
            if not self._has_sent_start:
                self._has_sent_start = True
                # print("[INFO] _has_sent_start is False, re-sending start (mark=0) command!")
                self._send_start_received()

            # 检查 packet_id 是否在有效范围内
            if packet_id >= self.image_instance.total_packets:
                # print(
                #     f"Warning: Received invalid packet_id={packet_id}, max valid={self.image_instance.total_packets - 1}")
                return

            # 统计：总收到包数量+1
            self.image_instance.total_received_count += 1

            # 检查是否需要取消重发（收到期望的第一个包时取消）
            if (self.image_instance.pending_transfer_cmd_id is not None and
                    self.image_instance.expected_first_packet is not None and
                    packet_id == self.image_instance.expected_first_packet):
                cc = self.airplane.s.ss
                cc.cleanSendRetry(self.image_instance.pending_transfer_cmd_id)
                # print(f"cleanSendRetry called for cmd_id={self.image_instance.pending_transfer_cmd_id}, "
                #       f"expected_first_packet={self.image_instance.expected_first_packet}")
                # 清除待取消的命令ID
                self.image_instance.pending_transfer_cmd_id = None
                self.image_instance.expected_first_packet = None

            # print(f"on_receive_image_packet_data: size_len={size_len}, packet_id={packet_id}, "
            #       f"total_packets={self.image_instance.total_packets}, cache_size={len(self.image_instance.packet_cache)}, "
            #       f"buff_len={len(buff)}")

            # 更新最后收到包的时间
            self.image_instance.last_packet_time = time.time()

            # 数据包已通过 ReadDataParser 的 checksum 验证
            # 检查是否是重复包
            if packet_id in self.image_instance.packet_cache:
                # 统计：重复包数量+1
                self.image_instance.duplicate_count += 1
                _, existing_data = self.image_instance.packet_cache[packet_id]
                if existing_data == buff:
                    # print(
                    #     f"Duplicate packet {packet_id} with same data, ignored (dup_count={self.image_instance.duplicate_count})")
                    pass
                else:
                    # print(f"Duplicate packet {packet_id} with different data! Keeping original.")
                    pass
                # 再次检查 _has_sent_start，防止极端情况下未发
                if not self._has_sent_start:
                    # print("[INFO] (dup) _has_sent_start is False, re-sending start (mark=0) command!")
                    self._send_start_received()
                    self._has_sent_start = True
                # 重复包也要重置定时器，因为数据流仍在正常传输
                self._start_timeout_timer()
                return

            # 存储数据包
            self.image_instance.packet_cache[packet_id] = (0, bytes(buff))

            if self.user_progress_callback is not None:
                self.user_progress_callback(len(self.image_instance.packet_cache), self.image_instance.total_packets)

            # print(f"Received packet {packet_id}/{self.image_instance.total_packets - 1}, "
            #       f"cache size: {len(self.image_instance.packet_cache)}, "
            #       f"stats: recv={self.image_instance.total_received_count}, dup={self.image_instance.duplicate_count}")


            # 检查是否已接收完所有包
            if len(self.image_instance.packet_cache) >= self.image_instance.total_packets:
                # 验证是否真的收齐了所有包
                if self._verify_all_packets_received():
                    print(f"All {self.image_instance.total_packets} packets received, assembling image...")
                    self._print_transfer_stats()
                    self._assemble_image()
                    self._when_received_end()
            else:
                # 启动/重置超时检测定时器（后备机制）
                self._start_timeout_timer()
                # 事件驱动推进重发窗口（实现算法第5条"立即切换到下一区间"的语义）
                self._advance_retransmit_window_if_needed(packet_id)

    def on_receive_image_packet_data_eof(self, size_len: int, origin_data: bytearray):
        """
        收到 EOF 包 [aa 03 0b ff ff b6] ，表示无人机完成了一轮数据传输
        EOF 包仅作为结束信号，需要检查是否有丢包并请求重传
        """
        with self._lock:
            if self.image_instance is None:
                return
            if self.image_instance.total_packets == 0:
                # 还没有收到图片信息包
                return

            # 收到 EOF 包也认为对端已开始发送，标记 _has_sent_start = True
            self._has_sent_start = True

            first_missing = self._calculate_first_missing_packet()
            # print(
            #     f"EOF received. Progress: {len(self.image_instance.packet_cache)}/{self.image_instance.total_packets}, "
            #     f"first_missing={first_missing}")

            # 更新最后收到包的时间
            self.image_instance.last_packet_time = time.time()

            # 检查是否收齐所有包
            if first_missing >= self.image_instance.total_packets:
                # 所有包已收到，组装图片并结束传输
                print(f"EOF received, all {self.image_instance.total_packets} packets received successfully")
                self._print_transfer_stats()
                self._assemble_image()
                self._when_received_end()
                return

            # 还有丢包，使用智能重发窗口算法替代原有重传逻辑
            print(f"EOF received, missing packets, using smart retransmission.")
            self.smart_retransmission()
            # 重新启动超时定时器
            self._start_timeout_timer()

    def _calculate_first_missing_packet(self) -> int:
        """
        计算从0开始第一个未收到的包索引（调用时已持有锁）
        返回值：第一个缺失包的索引，如果全部收齐则返回 total_packets
        """
        if self.image_instance is None:
            return 0
        for i in range(self.image_instance.total_packets):
            if i not in self.image_instance.packet_cache:
                return i
        return self.image_instance.total_packets

    def _verify_all_packets_received(self) -> bool:
        """验证是否收齐了所有数据包（调用时已持有锁）"""
        if self.image_instance is None:
            return False
        for i in range(self.image_instance.total_packets):
            if i not in self.image_instance.packet_cache:
                return False
        return True

    def _assemble_image(self):
        """将所有数据包组装成完整的图片（调用时已持有锁）"""
        if self.image_instance is None:
            return

        if not self._verify_all_packets_received():
            missing_ids = _get_missing_ids(
                self.image_instance.total_packets,
                set(self.image_instance.packet_cache.keys()),
            )
            print(
                f"[WARNING] _assemble_image called with incomplete packets: "
                f"received={len(self.image_instance.packet_cache)}/"
                f"{self.image_instance.total_packets}, "
                f"missing_count={len(missing_ids)}, "
                f"first_missing={missing_ids[0] if missing_ids else None}, "
                f"missing_preview={missing_ids[:20]}"
            )

        image_data = bytearray()
        missing_count = 0

        for i in range(self.image_instance.total_packets):
            if i in self.image_instance.packet_cache:
                _, packet_data = self.image_instance.packet_cache[i]
                image_data.extend(packet_data)
            else:
                missing_count += 1
                packet_size = (
                    self.image_instance.total_size - (i * 26)
                    if i == self.image_instance.total_packets - 1
                    else 26
                )
                image_data.extend(b"\x00" * packet_size)
                print(f"[WARNING] Missing packet {i}, filled with zeros.")

        if missing_count > 0:
            print(f"[WARNING] Image assembled with {missing_count} missing packets (filled with zeros).")

        self.image_instance.image_data = bytes(image_data[:self.image_instance.total_size])
        self.received_image_cache[self.image_instance.count_cmd_id_from_airplane] = self.image_instance

        if self.user_progress_callback:
            self.user_progress_callback(self.image_instance.total_packets, self.image_instance.total_packets)
        if self.user_receive_callback:
            self.user_receive_callback(self.image_instance.image_data)

        # print(
        #     f"[INFO] Image assembled! Size: {len(self.image_instance.image_data)} bytes, "
        #     f"count_cmd_id: {self.image_instance.count_cmd_id_from_airplane}, "
        #     f"complete: {missing_count == 0}."
        # )

    def _start_timeout_timer(self):
        """启动或重置超时检测定时器（调用时已持有锁）"""
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
        """
        Enhanced timeout handling with detailed logging.
        """
        with self._lock:
            if self.image_instance is None:
                return

            current_time = time.time()

            # Check total timeout
            if current_time - self.image_instance.start_time > self.TOTAL_TIMEOUT:
                print(
                    f"[ERROR] Total timeout reached! Received {len(self.image_instance.packet_cache)}/"
                    f"{self.image_instance.total_packets} packets. Aborting transfer."
                )
                self._print_transfer_stats()
                if len(self.image_instance.packet_cache) > 0:
                    self._assemble_image()
                self._when_received_end()
                return

            # Calculate first missing packet
            first_missing = self._calculate_first_missing_packet()

            if first_missing < self.image_instance.total_packets:
                self.image_instance.timeout_triggered_retransmit_count += 1
                self.smart_retransmission()
                self._start_timeout_timer()
            else:
                # print("[INFO] Timeout check: all packets received. Assembling image...")
                self._print_transfer_stats()
                self._assemble_image()
                self._when_received_end()

    def _print_transfer_stats(self):
        """打印传输统计信息（调用时已持有锁）"""
        if self.image_instance is None:
            return

        info = self.image_instance
        elapsed_time = time.time() - info.start_time
        unique_packets = len(info.packet_cache)

        # 计算传输效率
        efficiency = (unique_packets / info.total_received_count * 100) if info.total_received_count > 0 else 0

        print("=" * 60)
        print("Transfer Statistics:")
        print(f"  Total packets expected: {info.total_packets}")
        print(f"  Unique packets received: {unique_packets}")
        print(f"  Total packets received (including duplicates): {info.total_received_count}")
        print(f"  Duplicate packets: {info.duplicate_count}")
        print(f"  Transmission efficiency: {efficiency:.1f}%")
        print(f"  Total retransmit requests: {info.retransmit_request_count}")
        print(f"    - Window triggered: {info.window_triggered_retransmit_count}")
        print(f"    - Timeout triggered: {info.timeout_triggered_retransmit_count}")
        print(f"    - EOF triggered: {info.eof_triggered_retransmit_count}")
        print(f"  Total transfer time: {elapsed_time:.2f}s")
        print(
            f"  Effective throughput: {info.total_size / elapsed_time / 1024:.2f} KB/s" if elapsed_time > 0 else "  Effective throughput: N/A")
        print("=" * 60)

    def _when_received_end(self):
        """当接收全部结束后，进行清理工作（调用时已持有锁）"""
        # 停止超时定时器
        self._stop_timeout_timer()
        # 清理远端缓存
        self._clean_remote_image()
        # 清空当前图片实例
        self.image_instance = None
        self._has_sent_start = False  # 结束后重置

    def _send_start_received(self):
        """发送开始传输指令（调用时已持有锁）"""
        with self._lock:
            if hasattr(self, '_has_sent_start') and self._has_sent_start:
                # print("[WARNING] _send_start_received() called more than once! 忽略本次 mark=0 指令。")
                return
            self._has_sent_start = True
            cc = self.airplane.s.ss
            (order_count, cmd) = self._send_transfer_pack(0x00_00_00_00)
            # print("_start_time_received", order_count, cmd.hex(' '))
            # 生成唯一的命令ID用于取消重发
            cmd_id = self._generate_cmd_id()
            # 记录待取消的命令ID和期望收到的第一个包（第0包）
            if self.image_instance is not None:
                self.image_instance.pending_transfer_cmd_id = cmd_id
                self.image_instance.expected_first_packet = 0
            cc.sendCommand(cmd, max_retry=2, cmd_id_for_clean=cmd_id)

    def _re_transfer_pack(self, pack_id: int):
        """发送从指定包开始重传的指令（调用时已持有锁）"""
        cc = self.airplane.s.ss
        (order_count, cmd) = self._send_transfer_pack(pack_id)
        print("_re_transfer_pack", order_count, cmd.hex(' '))
        # 生成唯一的命令ID用于取消重发
        cmd_id = self._generate_cmd_id()
        # 记录待取消的命令ID和期望收到的第一个包（即请求重传的pack_id）
        if self.image_instance is not None:
            self.image_instance.pending_transfer_cmd_id = cmd_id
            self.image_instance.expected_first_packet = pack_id
        cc.sendCommand(cmd, max_retry=2, cmd_id_for_clean=cmd_id)
        pass

    def _clean_remote_image(self):
        """在传输结束，全部接收完毕后，发送指令清除远端数据"""
        cc = self.airplane.s.ss
        (order_count, cmd) = self._send_transfer_pack(0xFF_FF_FF_FF)
        # print("_clean_remote_image", order_count, cmd.hex(' '))
        cc.sendCommand(cmd, max_retry=3)
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
        order_count = self.image_instance.count_cmd_id_from_airplane if self.image_instance else 0
        cmd = bytearray([0xBB, 0X08, 0x0A, 0x00])
        cmd.extend(pack("<H", order_count))  # Int16LE (count)
        cmd.extend(pack("<I", pack_id))  # UInt32LE (pack_id)
        checksum = self._check_sum1(cmd)
        cmd = cmd + checksum
        return (order_count, cmd)

    def send_cap_image(self,
                       user_receive_callback: typing.Optional[typing.Callable[[bytes], None]] = None,
                       user_progress_callback: typing.Optional[typing.Callable[[int, int], None]] = None,
                       ) -> typing.Optional[int]:
        """
        无人机拍照指令。此指令触发无人机拍照。
        :return: 拍照指令的 order_count，用于后续匹配图片数据
        """
        self.user_receive_callback = user_receive_callback
        self.user_progress_callback = user_progress_callback
        with self._lock:
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

    def get_image(self, count_cmd_id: int) -> typing.Optional[np.ndarray]:
        """
        获取已接收完成的图片数据
        :param count_cmd_id: 拍照指令的 order_count
        :return: 图片的二进制数据（JPG格式），如果不存在则返回 None
        """
        with self._lock:
            if count_cmd_id in self.received_image_cache:
                return image_file_2_mat(self.received_image_cache[count_cmd_id].image_data)
            return None

    def get_latest_image(self) -> typing.Optional[bytes]:
        """
        获取最新接收完成的图片数据
        :return: 图片的二进制数据（JPG格式），如果没有图片则返回 None
        """
        with self._lock:
            if not self.received_image_cache:
                return None
            latest_key = max(self.received_image_cache.keys())
            return self.received_image_cache[latest_key].image_data

    def is_transfer_in_progress(self) -> bool:
        """
        检查是否有正在进行的图片传输
        :return: True 如果有正在进行的传输
        """
        with self._lock:
            return self.image_instance is not None

    def get_transfer_progress(self) -> typing.Optional[tuple[int, int]]:
        """
        获取当前传输进度
        :return: (已接收包数量, 总包数量) 元组，如果没有传输则返回 None
        """
        with self._lock:
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
            print(f"[ERROR] No image data found for count_cmd_id={count_cmd_id}.")
            return False
        try:
            write_mat_2_file(image_data, file_path)
            print(f"[INFO] Image saved successfully to {file_path}.")
            return True
        except Exception as e:
            print(f"[ERROR] Failed to save image to {file_path}: {e}")
            return False

    def save_latest_image_to_file(self, file_path: str) -> bool:
        """
        将最新接收的图片保存到文件
        :param file_path: 保存的文件路径
        :return: True 如果保存成功
        """
        image_data = self.get_latest_image()
        if image_data is None:
            print("[ERROR] No latest image data available.")
            return False
        try:
            with open(file_path, 'wb') as f:
                f.write(image_data)
            print(f"[INFO] Latest image saved successfully to {file_path}.")
            return True
        except IOError as e:
            print(f"[ERROR] Failed to save latest image to {file_path}: {e}")
            return False

    def clear_image_cache(self, count_cmd_id: typing.Optional[int] = None):
        """
        清除图片缓存
        :param count_cmd_id: 如果指定则只清除该图片，否则清除所有图片
        """
        with self._lock:
            if count_cmd_id is not None:
                if count_cmd_id in self.received_image_cache:
                    del self.received_image_cache[count_cmd_id]
            else:
                self.received_image_cache.clear()

    def cancel_current_transfer(self):
        """
        取消当前正在进行的传输
        """
        with self._lock:
            if self.image_instance is not None:
                self._stop_timeout_timer()
                self._clean_remote_image()
                self.image_instance = None

    def _is_window_filled(self, start: int, end: int) -> bool:
        """检查 [start, end] 区间内的所有包是否已全部接收（调用时已持有锁）"""
        if self.image_instance is None:
            return True
        return all(pid in self.image_instance.packet_cache for pid in range(start, end + 1))

    def _advance_retransmit_window_if_needed(self, packet_id: int):
        """
        在接收到新包后，检查当前重发窗口是否应推进到下一个窗口。

        这里采用“流已越过窗口就推进”的策略：
          - 如果当前窗口已补齐，推进；
          - 如果 packet_id >= 当前窗口 end，说明当前重传流已经越过该窗口，也推进；
          - 未补齐的包不会丢失跟踪，下一轮 timeout / EOF 会重新计算 missing list 再补。

        这样可以避免一直卡在第一个顽固缺包上，导致后面的缺包没有机会被针对性请求。
        """
        if self.image_instance is None:
            return

        windows = self.image_instance._retransmit_windows
        if not windows:
            return

        idx = self.image_instance._retransmit_window_idx
        if idx >= len(windows):
            return

        start, end, W = windows[idx]

        window_done = self._is_window_filled(start, end)
        stream_passed_window = packet_id >= end

        if not window_done and not stream_passed_window:
            return

        if stream_passed_window and not window_done:
            missing_in_window = [
                pid for pid in range(start, end + 1)
                if pid not in self.image_instance.packet_cache
            ]
            print(
                f"[smart_retransmission] Passed window [{start}, {end}] "
                f"but still missing {missing_in_window[:20]}. "
                f"Advancing anyway; it will be retried in the next missing scan."
            )

        idx += 1
        while idx < len(windows):
            ns, ne, _ = windows[idx]
            if not self._is_window_filled(ns, ne):
                break
            idx += 1

        self.image_instance._retransmit_window_idx = idx

        if idx < len(windows):
            ns, ne, nW = windows[idx]
            print(
                f"[smart_retransmission] Advancing to window "
                f"[{idx}/{len(windows) - 1}]: [{ns}, {ne}] "
                f"(gap_size={ne - ns + 1}, W={nW})"
            )
            self.image_instance.retransmit_request_count += 1
            self.image_instance.window_triggered_retransmit_count += 1
            self._re_transfer_pack(ns)
        else:
            print(
                f"[smart_retransmission] All {len(windows)} window(s) requested once. "
                f"Waiting for EOF/timeout to rescan remaining missing packets."
            )

    def smart_retransmission(self, alpha=5, W_max=32):
        """
        智能重发窗口主流程（非阻塞状态机版本）。

        算法流程（对应算法说明第1-7条）：
          1. 计算当前缺失包列表
          2. 聚类为连续区间（窗口）
          3. 计算每个窗口大小 W = min(L + α, W_max)
          4. 存储窗口列表到状态机，从第一个未填充窗口开始
          5. 发送第一个窗口的重传请求后立即返回（非阻塞！）
             ↳ 后续窗口推进由 _advance_retransmit_window_if_needed() 在每个
               新包到达时事件驱动执行，实现"实时监控/立即切换"语义
          6. 若超时仍有缺包，本方法会被再次调用，重新计算窗口并从当前状态继续
          7. 所有包收齐后由 on_receive_image_packet_data() 触发组装

        原设计中的死锁根因（已修复）：
          - 本方法在持有 self._lock 的情况下被调用
          - 原来的 while True: time.sleep(0.05) 在持锁状态下自旋等待 packet_cache 更新
          - on_receive_image_packet_data() 也需要 self._lock 才能写入 packet_cache
          - 锁永远不会释放 → packet_cache 永远不变 → 自旋永不退出 → 死锁
        """
        if self.image_instance is None:
            return

        total_packets = self.image_instance.total_packets
        received_packet_ids = set(self.image_instance.packet_cache.keys())
        controller = SmartRetransmissionController(total_packets, received_packet_ids, alpha, W_max)
        windows = controller.get_windows()

        if not windows:
            # 再次确认是否真的全部收齐（防御性处理）
            if self._verify_all_packets_received():
                self._print_transfer_stats()
                self._assemble_image()
                self._when_received_end()
            return

        # 存储最新窗口列表（每次调用重新计算，确保状态与当前 packet_cache 一致）
        self.image_instance._retransmit_windows = windows
        # 从第一个尚未填充的窗口开始（重入保护：跳过已填充的窗口）
        idx = 0
        while idx < len(windows) and self._is_window_filled(windows[idx][0], windows[idx][1]):
            idx += 1
        self.image_instance._retransmit_window_idx = idx

        if idx >= len(windows):
            # 所有窗口已被填充（罕见情况，防御性处理）
            if self._verify_all_packets_received():
                self._print_transfer_stats()
                self._assemble_image()
                self._when_received_end()
            return

        start, end, W = windows[idx]

        now = time.time()
        if (
                self.image_instance.last_retransmit_at_packet == start
                and now - self.image_instance.last_retransmit_time < 0.5
        ):
            print(
                f"[smart_retransmission] Skip duplicated retransmit request for packet {start} "
                f"within cooldown."
            )
            return

        self.image_instance.last_retransmit_at_packet = start
        self.image_instance.last_retransmit_time = now

        print(f"[smart_retransmission] {len(windows)} missing window(s). "
              f"Starting window [{idx}/{len(windows) - 1}]: [{start}, {end}] "
              f"(gap_size={end - start + 1}, W={W}). "
              f"Total missing: {len(controller.missing_ids)}")

        self.image_instance.retransmit_request_count += 1
        self.image_instance.window_triggered_retransmit_count += 1
        self._re_transfer_pack(start)
        # 立即返回，不自旋等待。
        # 后续窗口由 _advance_retransmit_window_if_needed() 在收包事件中驱动推进。
