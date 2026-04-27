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
# [aa 03 0b ff ff b6] EOF 包
# ===================================================

# 接收流程：
#
# on_receive_image_pack_info()   收到包头帧
#     → 初始化缓冲区
#     → 发送 mark=0（握手，通知无人机从头开始发）
#     → 启动超时定时器
#
# on_receive_image_packet_data() 收到每个数据帧
#     → 写入缓存（跳过重复包）
#     → 计算 lost_mark（第一个缺失包序号）
#     → 若 lost_mark == total_packets → 组装图片 → 结束
#     → 若 lost_mark != packet_id + 1 → 发送重传请求 mark=lost_mark
#     → 重置超时定时器
#
# on_receive_image_packet_data_eof() 收到 EOF 包（无人机发完一轮）
#     → 计算 lost_mark
#     → 若全收完 → 组装图片 → 结束
#     → 否则 → 发送重传请求 → 重置超时定时器
#
# 超时(_on_packet_timeout)
#     → 若总超时 → 强制组装 → 结束
#     → 否则 → 发送重传请求 → 重置超时定时器


@dataclasses.dataclass
class ImageInfo:
    count_cmd_id: int                    # 拍照指令的 order_count（本地生成）
    count_cmd_id_from_airplane: int = 0  # 无人机回传的 count（用于 control frame）
    total_size: int = 0                  # 图片总大小（字节）
    total_packets: int = 0              # 总分包数 = ceil(total_size / 26)
    # 已收到的数据包  {packet_index: data_bytes}
    packet_cache: dict[int, bytes] = dataclasses.field(default_factory=dict)
    # 组装完成的图片数据
    image_data: bytes = b""
    # 最后收到包的时间（用于包级超时）
    last_packet_time: float = dataclasses.field(default_factory=time.time)
    # 传输开始时间（用于总超时）
    start_time: float = dataclasses.field(default_factory=time.time)
    # 上次重传请求的 mark 值（用于冷却防抖）
    _last_retransmit_mark: int = -1
    # 上次重传请求的时间戳
    _last_retransmit_time: float = 0.0
    # 当前挂起的重传命令ID（用于 cleanSendRetry）
    _pending_cmd_id: typing.Optional[int] = None


class ImageReceiver:
    airplane: AirplaneController
    # 任何时候只存在一张圖片
    image_instance: ImageInfo | None = None
    # 超时检测定时器
    _timeout_timer: typing.Optional[threading.Timer]
    # 包超时时间（秒）：连续此时间内无包到达则触发重传请求
    PACKET_TIMEOUT: float = 1.5
    # 总超时时间（秒）：超过此时间强制结束
    TOTAL_TIMEOUT: float = 15.0
    # 重传请求冷却时间（秒）：同一 mark 在此时间内不重复发送
    RETRANSMIT_COOLDOWN: float = 0.3
    # 接收完成的图片表  {count_cmd_id_from_airplane: ImageInfo}
    received_image_cache: dict[int, ImageInfo]
    # 临界区锁
    _lock: threading.RLock
    # 命令ID计数器
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
        self._has_sent_start = False

    # ------------------------------------------------------------------
    # 内部工具
    # ------------------------------------------------------------------

    def _generate_cmd_id(self) -> int:
        self._cmd_id_counter += 1
        return self._cmd_id_counter

    def _check_sum1(self, data: bytearray):
        return bytearray([sum(data) & 0xFF])

    def _check_sum2(self, header: bytearray, params: bytearray):
        return bytearray([sum(header + params) & 0xFF])

    def _get_lost_mark(self) -> int:
        """
        从 0 开始扫描，返回第一个缺失包的序号。
        若全部收齐则返回 total_packets（等价于 C++ 的 TRANSFER_COMPLETE）。
        对应 C++ PackRreceive::getLostMark()。
        """
        if self.image_instance is None:
            return 0
        cache = self.image_instance.packet_cache
        for i in range(self.image_instance.total_packets):
            if i not in cache:
                return i
        return self.image_instance.total_packets

    # ------------------------------------------------------------------
    # 帧发送
    # ------------------------------------------------------------------

    def _send_transfer_pack(self, pack_id: int):
        """
        构造并返回控制帧。
        pack_id=0x00000000: 请求从头开始发送
        pack_id=0xNNNNNNNN: 请求从第N包开始重发
        pack_id=0xFFFFFFFF: 通知无人机清除缓存
        帧格式: [0xBB, 0x08, 0x0A, id(1), count(2LE), pack_id(4LE), checksum(1)]
        """
        if self.image_instance is None:
            print("Warning: _send_transfer_pack called with no active image_instance")
        order_count = self.image_instance.count_cmd_id_from_airplane if self.image_instance else 0
        cmd = bytearray([0xBB, 0x08, 0x0A, 0x00])
        cmd.extend(pack("<H", order_count))   # count: u16 LE
        cmd.extend(pack("<I", pack_id))       # mark:  u32 LE
        cmd += self._check_sum1(cmd)
        return order_count, cmd

    def _send_start_received(self):
        """发送 mark=0 控制帧，通知无人机从头开始发（每次传输只发一次）。"""
        if self._has_sent_start:
            return
        self._has_sent_start = True
        cc = self.airplane.s.ss
        order_count, cmd = self._send_transfer_pack(0x00_00_00_00)
        cmd_id = self._generate_cmd_id()
        if self.image_instance is not None:
            self.image_instance._pending_cmd_id = cmd_id
        print(f"[image_transfer] send mark=0 (start), order_count={order_count}, cmd={cmd.hex(' ')}")
        cc.sendCommand(cmd, max_retry=2, cmd_id_for_clean=cmd_id)

    def _request_retransmit(self, lost_mark: int):
        """
        发送重传请求控制帧（mark=lost_mark）。
        实现冷却防抖：同一 mark 在 RETRANSMIT_COOLDOWN 秒内不重复发送。
        对应 C++ PackRreceive::readPacks() 中 emit signalLostMark(lostMark) 的语义。
        """
        if self.image_instance is None:
            return

        now = time.time()
        info = self.image_instance

        # 冷却防抖：同一 mark 在冷却期内跳过
        if (info._last_retransmit_mark == lost_mark and
                now - info._last_retransmit_time < self.RETRANSMIT_COOLDOWN):
            return

        info._last_retransmit_mark = lost_mark
        info._last_retransmit_time = now

        cc = self.airplane.s.ss

        # 取消旧的挂起命令，避免无人机被多个重传指令同时驱动
        if info._pending_cmd_id is not None:
            cc.cleanSendRetry(info._pending_cmd_id)
            info._pending_cmd_id = None

        order_count, cmd = self._send_transfer_pack(max(0, lost_mark - 1))
        cmd_id = self._generate_cmd_id()
        info._pending_cmd_id = cmd_id
        print(f"\n[image_transfer] retransmit mark={lost_mark}, order_count={order_count}, cmd={cmd.hex(' ')}")
        cc.sendCommand(cmd, max_retry=3, cmd_id_for_clean=cmd_id)

    def _clean_remote_image(self):
        """发送 mark=0xFFFFFFFF，通知无人机清除图像缓存。"""
        cc = self.airplane.s.ss
        order_count, cmd = self._send_transfer_pack(0xFF_FF_FF_FF)
        cc.sendCommand(cmd, max_retry=3)

    # ------------------------------------------------------------------
    # 超时定时器
    # ------------------------------------------------------------------

    def _start_timeout_timer(self):
        """启动或重置包超时定时器（调用时已持有锁）。"""
        if self._timeout_timer is not None:
            self._timeout_timer.cancel()
        self._timeout_timer = threading.Timer(self.PACKET_TIMEOUT, self._on_packet_timeout)
        self._timeout_timer.daemon = True
        self._timeout_timer.start()

    def _stop_timeout_timer(self):
        if self._timeout_timer is not None:
            self._timeout_timer.cancel()
            self._timeout_timer = None

    def _on_packet_timeout(self):
        """
        超时回调：与 C++ PackRreceive::readTimeOut() 类似。
        若总超时则强制结束；否则重新发送重传请求并重置定时器。
        """
        with self._lock:
            if self.image_instance is None:
                return

            # 检查总超时
            if time.time() - self.image_instance.start_time > self.TOTAL_TIMEOUT:
                received = len(self.image_instance.packet_cache)
                total = self.image_instance.total_packets
                print(f"[image_transfer] Total timeout! received={received}/{total}. Force finish.")
                if received > 0:
                    self._assemble_image()
                self._when_received_end()
                return

            lost_mark = self._get_lost_mark()
            if lost_mark >= self.image_instance.total_packets:
                # 已全部收完（极端情况下定时器触发时刚好收齐）
                self._assemble_image()
                self._when_received_end()
                return

            received = len(self.image_instance.packet_cache)
            total = self.image_instance.total_packets
            print(f"[image_transfer] Packet timeout, lost_mark={lost_mark}, "
                  f"received={received}/{total}. Retransmitting.")
            self._request_retransmit(lost_mark)
            self._start_timeout_timer()

    # ------------------------------------------------------------------
    # 接收回调（核心逻辑）
    # ------------------------------------------------------------------

    def on_receive_image_pack_info(
            self,
            _fly_id: int,
            photo_count_cmd_id: int,
            total_size: int,
            data_type: int,
            origin_data: bytearray,
    ):
        """
        收到无人机包头帧（0xAA...0x0A...）。
        对应 C++ PackRreceive::readStart()：初始化缓冲区，发送 mark=0 握手。
        """
        print(f"on_receive_image_pack_info: fly_id={_fly_id}, count={photo_count_cmd_id}, "
              f"total_size={total_size}, data_type={data_type}")
        if data_type != 0:
            return  # 只处理图片数据

        with self._lock:
            if self.image_instance is None:
                return  # 没有等待中的拍照请求

            total_packets = (total_size + 25) // 26
            self.image_instance.count_cmd_id_from_airplane = photo_count_cmd_id
            self.image_instance.total_size = total_size
            self.image_instance.total_packets = total_packets
            self.image_instance.start_time = time.time()

            # 发送 mark=0，通知无人机从头开始发（握手）
            self._send_start_received()
            # 启动超时定时器
            self._start_timeout_timer()

    def on_receive_image_packet_data(
            self,
            size_len: int,
            packet_id: int,
            buff: bytes,
            origin_data: bytearray,
    ):
        """
        收到数据帧（0xAA...0x0B...）。
        对应 C++ PackRreceive::readPacks()：写入缓存，计算 lost_mark，按需请求重传。

        核心逻辑（完全对应 readPacks）：
          1. 写入缓存（跳过重复包）
          2. lost_mark = 第一个缺失包序号
          3. 若 lost_mark == total_packets → 全收完 → 组装 → 结束
          4. 若 lost_mark != packet_id + 1 → 发重传请求(lost_mark)
          5. 重置超时定时器
        """
        with self._lock:
            if self.image_instance is None or self.image_instance.total_packets == 0:
                return

            # 保底：确保 mark=0 已发出
            if not self._has_sent_start:
                self._send_start_received()

            # 越界包丢弃
            if packet_id >= self.image_instance.total_packets:
                return

            # 跳过重复包（对应 C++ readUnits 中 !getMark(index) 的保护）
            if packet_id in self.image_instance.packet_cache:
                # 重复包也重置定时器，避免因重复流导致超时
                self.image_instance.last_packet_time = time.time()
                self._start_timeout_timer()
                return

            # 写入缓存
            self.image_instance.packet_cache[packet_id] = bytes(buff)
            self.image_instance.last_packet_time = time.time()
            print(f".{packet_id}", end=" ")

            if self.user_progress_callback is not None:
                self.user_progress_callback(
                    len(self.image_instance.packet_cache),
                    self.image_instance.total_packets,
                )

            # 计算当前最早缺失包序号（对应 C++ getLostMark()）
            lost_mark = self._get_lost_mark()

            if lost_mark >= self.image_instance.total_packets:
                # 全部收完（对应 C++ lostMark == TRANSFER_COMPLETE）
                print(f"\n[image_transfer] All {self.image_instance.total_packets} packets received. Assembling.")
                self._assemble_image()
                self._when_received_end()
                return

            # 对应 C++:  if( lostMark != (pack.units + 1) ) emit signalLostMark(lostMark)
            if lost_mark != packet_id + 1:
                self._request_retransmit(lost_mark)

            # 重置超时定时器（对应 C++ timeout.start(TIME_OUT)）
            self._start_timeout_timer()

    def on_receive_image_packet_data_eof(self, size_len: int, origin_data: bytearray):
        """
        收到 EOF 包 [aa 03 0b ff ff ...]，表示无人机完成了本轮循环发送。
        检查是否有缺包：有则请求重传，无则结束。
        """
        with self._lock:
            if self.image_instance is None or self.image_instance.total_packets == 0:
                return

            self._has_sent_start = True  # EOF 到达说明无人机已在发送，标记已开始
            self.image_instance.last_packet_time = time.time()

            lost_mark = self._get_lost_mark()
            received = len(self.image_instance.packet_cache)
            total = self.image_instance.total_packets

            if lost_mark >= total:
                print(f"[image_transfer] EOF: all {total} packets received. Assembling.")
                self._assemble_image()
                self._when_received_end()
                return

            print(f"[image_transfer] EOF: received={received}/{total}, lost_mark={lost_mark}. Retransmitting.")
            self._request_retransmit(lost_mark)
            self._start_timeout_timer()

    # ------------------------------------------------------------------
    # 组装与收尾
    # ------------------------------------------------------------------

    def _assemble_image(self):
        """将所有数据包组装成完整图片（调用时已持有锁）。"""
        if self.image_instance is None:
            return

        image_data = bytearray()
        missing_count = 0

        for i in range(self.image_instance.total_packets):
            if i in self.image_instance.packet_cache:
                image_data.extend(self.image_instance.packet_cache[i])
            else:
                missing_count += 1
                # 末尾包可能不足26字节
                if i == self.image_instance.total_packets - 1:
                    remain = self.image_instance.total_size - i * 26
                    image_data.extend(b"\x00" * max(remain, 0))
                else:
                    image_data.extend(b"\x00" * 26)
                print(f"[WARNING] Missing packet {i}, filled with zeros.")

        if missing_count:
            print(f"[WARNING] Assembled with {missing_count} missing packets.")

        self.image_instance.image_data = bytes(image_data[:self.image_instance.total_size])
        self.received_image_cache[self.image_instance.count_cmd_id_from_airplane] = self.image_instance

        elapsed = time.time() - self.image_instance.start_time
        unique = len(self.image_instance.packet_cache)
        total = self.image_instance.total_packets
        print(f"[image_transfer] Assembled: {unique}/{total} packets, "
              f"size={len(self.image_instance.image_data)}B, time={elapsed:.2f}s")

        if self.user_progress_callback:
            self.user_progress_callback(total, total)
        if self.user_receive_callback:
            self.user_receive_callback(self.image_instance.image_data)

    def _when_received_end(self):
        """传输结束后的清理工作（调用时已持有锁）。"""
        self._stop_timeout_timer()
        self._clean_remote_image()
        self.image_instance = None
        self._has_sent_start = False

    # ------------------------------------------------------------------
    # 公开 API
    # ------------------------------------------------------------------

    def send_cap_image(
            self,
            user_receive_callback: typing.Optional[typing.Callable[[bytes], None]] = None,
            user_progress_callback: typing.Optional[typing.Callable[[int, int], None]] = None,
    ) -> typing.Optional[int]:
        """
        发送拍照指令，触发无人机拍照并回传图片。
        :return: 拍照指令的 order_count，用于后续匹配图片，失败返回 None
        """
        self.user_receive_callback = user_receive_callback
        self.user_progress_callback = user_progress_callback
        with self._lock:
            if self.image_instance is not None:
                print("Warning: Previous image transfer still in progress")
                return None

            cc = self.airplane.s.ss
            from .CommandConstructor import CmdType
            (params, order_count) = cc.build_cmd_params(22, 0x01)
            cmd = cc.join_cmd(CmdType.SINGLE_CONTROL, params)
            print("send_cap_image", order_count, cmd.hex(' '))

            self.image_instance = ImageInfo(count_cmd_id=order_count)
            self._has_sent_start = False
            cc.sendCommand(cmd)
            return order_count

    def get_image(self, count_cmd_id: int) -> typing.Optional[np.ndarray]:
        """获取已接收完成的图片（numpy 矩阵）。"""
        with self._lock:
            if count_cmd_id in self.received_image_cache:
                return image_file_2_mat(self.received_image_cache[count_cmd_id].image_data)
            return None

    def get_latest_image(self) -> typing.Optional[bytes]:
        """获取最新接收完成的图片原始字节。"""
        with self._lock:
            if not self.received_image_cache:
                return None
            latest_key = max(self.received_image_cache.keys())
            return self.received_image_cache[latest_key].image_data

    def is_transfer_in_progress(self) -> bool:
        """是否有正在进行的图片传输。"""
        with self._lock:
            return self.image_instance is not None

    def get_transfer_progress(self) -> typing.Optional[tuple[int, int]]:
        """获取当前传输进度 (已收包数, 总包数)，无传输时返回 None。"""
        with self._lock:
            if self.image_instance is None:
                return None
            return len(self.image_instance.packet_cache), self.image_instance.total_packets

    def save_image_to_file(self, count_cmd_id: int, file_path: str) -> bool:
        """将指定图片保存到文件。"""
        image_data = self.get_image(count_cmd_id)
        if image_data is None:
            print(f"[ERROR] No image data found for count_cmd_id={count_cmd_id}.")
            return False
        try:
            write_mat_2_file(image_data, file_path)
            print(f"[INFO] Image saved to {file_path}.")
            return True
        except Exception as e:
            print(f"[ERROR] Failed to save image: {e}")
            return False

    def save_latest_image_to_file(self, file_path: str) -> bool:
        """将最新接收的图片保存到文件。"""
        image_data = self.get_latest_image()
        if image_data is None:
            print("[ERROR] No latest image data available.")
            return False
        try:
            with open(file_path, 'wb') as f:
                f.write(image_data)
            print(f"[INFO] Latest image saved to {file_path}.")
            return True
        except IOError as e:
            print(f"[ERROR] Failed to save latest image: {e}")
            return False

    def clear_image_cache(self, count_cmd_id: typing.Optional[int] = None):
        """清除图片缓存，不指定则清除全部。"""
        with self._lock:
            if count_cmd_id is not None:
                self.received_image_cache.pop(count_cmd_id, None)
            else:
                self.received_image_cache.clear()

    def cancel_current_transfer(self):
        """取消当前正在进行的传输。"""
        with self._lock:
            if self.image_instance is not None:
                self._stop_timeout_timer()
                self._clean_remote_image()
                self.image_instance = None
                self._has_sent_start = False
