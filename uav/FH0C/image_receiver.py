import dataclasses
import threading
import time
import typing


# 协议内容：
# ===================================================
# 发送给无人机
#   id  22  count   u8 mode;//1拍照回传、2颜色采集回传、3颜色识别回传	photographMode
# --> 套用原数据包格式：[0x00, 0x16, _order_count(), 0x01]  // 拍照指令
# ===================================================
# 无人机拍照后响应
# 0xAA	len	0x0A	"    u8 id;          //哪个编号的飞机回传的数据包 TODO 是否就是 0x00
#                        u16 count;      //数据包序号 TODO 拍照的第几张图片的序号？
#                        u32 size;       //数据包大小，单位：byte（TODO 图片的总大小？）
#                        u16 type = 0;   //数据包类型（0图片数据1离线程序2错误信息）"	SUM
# header: [0xAA, len 0x0A, {0x0A]
# payload: [id 1, count 2, size 4, type 2}, checksum 1]
# <-- [0xAA, len 0x0A, {0x0A, id 1, count 2, size 4, type 2}, checksum 1]
# ===================================================
# 发送给无人机的开始发送指令和重发指令
# 根据数据包大小 size 计算分包数量，每包发送 26 字节，并且发送应答帧，这时候的应答帧的mark应该赋值为0
# TODO 重发 0 包是否会导致后续全部重发？重发某个包是否会导致后续全部重发？
# 0xBB	len	0x0A	"    u8 id;          //与数据包开始的id一样（飞机id）
#                        u16 count;      //与数据包开始的count一样， TODO 需要获取的图片的序号？如果不存在这个图片会返回什么？
#                        u32 mark;       //丢包序号"
# [0xBB, len 0x08, {0x0A, id 1, count 2, mark(4)}, checksum 1]
# 开始发送给无人机的指令：
# --> [0xBB, len 0x08, {0x0A, id 1, count 2, (0x00_00_00_00) 4}, checksum 1]  // 让无人机开始发送数据
# --> [0xBB, len 0x08, {0x0A, id 1, count 2, (0xnn_nn_nn_nn) 4}, checksum 1]  // 让无人机重发指定编号 0xnn_nn_nn_nn 的分包
# --> [0xBB, len 0x08, {0x0A, id 1, count 2, (0xFF_FF_FF_FF) 4}, checksum 1]  // 全部传输已完成，无人机删除存储的图像缓存
# ===================================================
# 无人机真实发送数据包的格式：
# 0xAA	len	0x0B	"    u16 units;      //包序号
#                        u8 buff[26];    //数据"	SUM
# header: [0xAA, len 0x1D, {0x0B]
# payload: [units 2, buff 26, checksum 1]
# <-- [0xAA, len 0x1D, {0x0B, units 2, buff 26}, checksum 1]
# 其中 units 是分包的序号，从 0 开始递增，buff 是分包的数据内容， TODO 最后一个分包的 buff 不足 26 字节时，多余空间是否为 0 ？
# 总 units 数量 = ceil(size / 26)，当 units == ceil(size / 26) - 1 时，说明是最后一个分包
# TODO 丢包重发的逻辑：是否需要使用超时时间和数据包序号跳变来检测是否丢包？
# ===================================================


@dataclasses.dataclass
class ImageInfo:
    photo_id: int
    total_size: int
    total_packets: int
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
    # dict[photo_id, ImageInfo]
    image_table: dict[int, ImageInfo]
    # 超时检测定时器
    _timeout_timer: typing.Optional[threading.Timer]
    # 包超时时间（秒），每个包约20ms，设置为300ms（约10个包的时间）可容忍一定波动
    PACKET_TIMEOUT: float = 0.3
    # 乱序容忍阈值：收到的包索引比期望索引大于此值时才认为丢包
    # 由于包间有业务数据干扰，允许轻微乱序（约3个包）
    OUT_OF_ORDER_THRESHOLD: int = 3
    # 总超时时间（秒），超过此时间强制结束（丢包情况下约5秒）
    TOTAL_TIMEOUT: float = 6.0
