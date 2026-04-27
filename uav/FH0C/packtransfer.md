# PackTransfer 通信协议数据收发逻辑详解

本文档基于 `packtransfer.h` / `packtransfer.cpp` 源码及 `image_receiver.py` 头部注释，完整还原数据接收与组装、以及请求远端重发的工作逻辑。

---

## 一、基本常量与数据结构

```cpp
#define UNITS_LEN  26        // 每个分包的数据载荷长度（字节）
#define TIME_OUT   5000      // 接收侧超时时间（毫秒）

enum {
    TRANSFER_ERROR    = 0xFFFFFFFE,  // 发送侧特殊状态：等待接收方确认包头
    TRANSFER_COMPLETE = 0xFFFFFFFF,  // 收发双方：全部数据已传输完毕
};
```

### 帧格式（来自 image_receiver.py 注释）

| 方向 | 帧类型 | 格式 |
|------|--------|------|
| 发送方→接收方 | **包头帧**（start frame） | `[0xAA, len=0x0A, {0x0A, id(1), count(2), size(4), type(2)}, checksum(1)]` |
| 发送方→接收方 | **数据帧**（data frame）  | `[0xAA, len=0x1D, {0x0B, units(2), buff(26)}, checksum(1)]` |
| 接收方→发送方 | **丢包/控制帧**（lost/ack frame） | `[0xBB, len=0x08, {0x0A, id(1), count(2), mark(4)}, checksum(1)]` |

其中 `mark` 字段含义：

| 值 | 含义 |
|----|------|
| `0x00000000` | 通知发送方"准备就绪，请从第 0 包开始发送" |
| `0xNNNNNNNN` | 通知发送方"第 N 包丢失，请从第 N 包开始重发" |
| `0xFFFFFFFF` | 通知发送方"全部收完，可以清除缓存"（TRANSFER_COMPLETE） |

### start_t（传输会话描述符）

```cpp
struct start_t {
    u8  id;     // 设备编号（哪台无人机）
    u16 count;  // 本次传输的序号，每次新传输递增，用于区分不同传输会话
    u32 size;   // 本次传输的数据总大小（字节）
    u16 type;   // 数据类型（0=图片, 1=Python程序, 2=错误信息）
};
```

### pack_t（单个分包）

```cpp
struct pack_t {
    u16 units;       // 分包序号，从 0 开始递增
    u8  buff[26];    // 分包数据内容（末尾包不足 26 字节时多余空间忽略）
};
```

### lost_t（丢包/控制信号）

```cpp
struct lost_t {
    u8  id;     // 设备编号
    u16 count;  // 传输序号（与 start_t.count 对应）
    u32 mark;   // 含义见上文 mark 字段说明
};
```

---

## 二、PackRreceive（接收方）

### 2.1 核心字段

| 字段 | 类型 | 说明 |
|------|------|------|
| `unitsMark[800]` | `u8[]` | 已收到分包的**位图**；每个 bit 对应一个分包，置 1 表示已收到。800 字节 = 6400 bit，最多跟踪 6400 个分包 |
| `packBuff[153600]` | `u8[]` | 已收到数据的组装缓冲区，最大 150 KB（320×240×2） |
| `packLen` | `u32` | 数据总大小（字节），由包头帧中的 `size` 字段赋值 |
| `markLen` | `u32` | 总分包数 = ⌈packLen / 26⌉ |
| `lostMark` | `u32` | **关键状态量**：当前最小的未收到分包序号；等于 `TRANSFER_COMPLETE` 时表示全部收完 |
| `reading` | `bool` | 是否正在一次传输中 |
| `start` | `start_t` | 当前传输会话的元信息 |
| `timeout` | `QTimer` | 5000 ms 超时定时器；任何时候连续 5 秒收不到包则超时失败 |

### 2.2 位图操作

```cpp
// 标记第 index 号分包已收到
void setMark(u32 index) {
    u32 cnt = index / 8;
    unitsMark[cnt] |= 1 << (index - cnt * 8);
}

// 查询第 index 号分包是否已收到（非零即已收到）
u8 getMark(u32 index) {
    u32 cnt = index / 8;
    return unitsMark[cnt] & (1 << (index - cnt * 8));
}

// 从 lostMark 当前位置向后扫描，将 lostMark 推进到第一个尚未收到的分包序号；
// 若全部收到则设为 TRANSFER_COMPLETE
void getLostMark(void) {
    for (; lostMark < markLen; lostMark++)
        if (!getMark(lostMark)) break;
    if (lostMark == markLen)
        lostMark = TRANSFER_COMPLETE;
}
```

> **重要**：`getLostMark()` 只会**单调递增** `lostMark`，因此它始终反映"目前为止最早的那个缺口"，不会因为后续包先到而回退。

### 2.3 初始化：readInit(buffLen)

```
lostMark = 0
packLen  = buffLen
markLen  = ceil(buffLen / 26)
unitsMark[0..799] = 全零（清空位图）
```

### 2.4 收到包头帧：readStart(start_t read)

```
if (!reading):
    // 空闲状态：任意设备的、id 或 count 与上次不同的包头都可以触发新传输
    if (start.id != read.id || start.count != read.count):
        start   = read
        reading = true
        timeout.start(5000)
        readInit(start.size)
        emit showProgress(0)

else:  // reading == true
    // 传输进行中：只允许同一设备（id 相同）发起一次新的传输（count 不同）打断当前传输
    if (start.id == read.id && start.count != read.count):
        start   = read
        reading = true
        timeout.start(5000)
        readInit(start.size)
        emit showProgress(0)

// 无论上述哪种情况，只要当前正在读取且 id+count 均匹配，就立即回应：
// 发送 mark=0，意思是"准备好了，请从第 0 包开始发"
if (reading && start.count == read.count && start.id == read.id):
    emit signalLostMark({id=start.id, count=start.count, mark=0})
```

**说明**：每次收到合法包头，接收方都会无条件发出一个 `mark=0` 的控制帧，将 `lostMark` 从 0 开始告知发送方。这既是"握手确认"，也是"请求从头发送"的信号。

### 2.5 收到数据帧：readPacks(pack_t pack)

```
// Step 1: 写入数据并更新 lostMark
readUnits(pack.units, pack.buff):
    if index < markLen && !getMark(index):
        计算本包实际长度（末尾包可能 < 26 字节）:
            remain = packLen - index * 26
            len    = min(26, remain)
        memcpy(packBuff + index*26, buff, len)
        setMark(index)
    getLostMark()   // 推进 lostMark 到最早缺口

// Step 2: 判断是否需要请求重传
if lostMark != (pack.units + 1):
    // 如果 lostMark 正好等于当前包序号+1，说明 0..units 都已收齐，不需要重传；
    // 否则存在丢包缺口，需要告诉发送方当前最早缺口位置。
    emit signalLostMark({id=start.id, count=start.count, mark=lostMark})

// Step 3: 判断是否全部完成
if lostMark == TRANSFER_COMPLETE:
    if reading:
        readStop()
        emit signalSucceed(start)   // 通知上层：传输成功
        emit showProgress(100)
else:
    timeout.start(5000)             // 重置超时
    emit showProgress(lostMark * 100 / markLen)
```

**重传触发举例**：

- 包 0-4 收到，包 5 丢失，包 6 收到 → `lostMark=5`，`units=6`，`5 ≠ 7` → 发出控制帧 `mark=5`
- 包 5 补收 → `lostMark` 推进，若无其他缺口则 → `lostMark=TRANSFER_COMPLETE`

### 2.6 超时处理：readTimeOut()

```
readStop()
emit showProgress(-1)   // 上报失败
```

连续 5 秒内未收到任何数据包则超时失败。

### 2.7 readStop()

```
timeout.stop()
start.count = 0xFFFF   // 重置会话标识（使下一次任意包头均可触发新传输）
reading = false
```

---

## 三、PackTransmit（发送方）

### 3.1 核心字段

| 字段 | 类型 | 说明 |
|------|------|------|
| `packArray` | `QByteArray` | 待发送的完整数据 |
| `packStart` | `start_t` | 本次传输的元信息（`count` 每次递增以区分会话） |
| `markLen` | `u32` | 总分包数 = ⌈size / 26⌉ |
| `lostMark` | `u32` | 接收方报告的最新"最小未收到包序号"；初始为 `TRANSFER_ERROR`（等待握手） |
| `sendMark` | `u32` | 当前正在发送的分包序号；发完末包后回绕到 `lostMark` |
| `writing` | `bool` | 是否正在发送 |
| `timer` | `QTimer` | 每 **2 ms** 触发一次 `run()`，驱动数据连续发送 |
| `updateTime` | `QTime` | 最后一次收到接收方回复的时间，用于超时检测 |

### 3.2 开始发送：write(id, type, pack)

```
if writing: return  // 防止并发

writing         = true
packArray       = pack
packStart.id    = id
packStart.type  = type
packStart.size  = pack.length()
packStart.count += 1        // 递增 count，与上次传输区分（溢出从 0 开始）
markLen         = ceil(size / 26)
lostMark        = TRANSFER_ERROR   // → 等待握手状态
sendMark        = 0
updateTime      = 当前时间
timer.start(2)              // 每 2ms 驱动一次 run()
emit showProgress(0)
```

### 3.3 收到接收方控制帧：readLostMark(lost_t lost)

```
// 只处理匹配当前传输会话的信号
if lost.id != packStart.id || lost.count != packStart.count: return

lostMark   = lost.mark    // 更新最新丢包位置
updateTime = 当前时间      // 心跳：重置超时计时

if sendMark < lostMark:
    // 发送进度落后于接收方期望，快进跟上
    sendMark = lostMark

else:
    // sendMark >= lostMark：发送方已到达或超过了缺口位置，需要回退重发
    // 引入 20ms 冷却防抖，避免同一缺口多次回报导致频繁回退
    if lostTemp != lostMark:
        // 新的丢包位置出现，记录时间和值
        lostTime = 当前时间
        lostTemp = lostMark
    else:
        // 同一个丢包位置持续出现，等待冷却后再回退
        if lostTime 距今 > 20ms:
            lostTime = 当前时间
            sendMark = lostMark   // 确认回退：从缺口处开始重发
```

**说明**：`20ms` 冷却机制防止接收方因网络延迟多次回报同一个丢包位置而导致发送方反复回退，浪费带宽。

### 3.4 发送主循环：run()（每 2ms 调用一次）

```
switch (lostMark):

    case TRANSFER_ERROR:        // 等待接收方响应包头（握手阶段）
        if updateTime 距今 > 1000ms:
            stop(); emit showProgress(-1)       // 握手超时，失败
        else:
            emit signalPackStart(packStart)     // 持续重发包头帧，直到收到接收方 mark=0

    case TRANSFER_COMPLETE:     // 全部数据已确认收完
        stop(); emit showProgress(100)

    default:                    // 正在发数据（lostMark 为有效分包序号）
        if updateTime 距今 > TIME_OUT(5000ms):
            stop(); emit showProgress(-1)       // 数据传输超时，失败
        else:
            emit showProgress(lostMark * 100 / markLen)
            sendPack(sendMark)                  // 发送当前分包
            sendMark++
            if sendMark >= markLen:
                sendMark = lostMark             // ← 关键：回绕到最早缺口，而非从 0 开始
```

**循环重传设计**：`sendMark` 到达末包后不是回到 0，而是回到 `lostMark`。这保证发送方持续在 `[lostMark, markLen-1]` 范围内循环发送，优先补齐接收方尚未收到的包。

### 3.5 发送一个分包：sendPack(mark)

```
unitsLen = min(26, packStart.size - mark * 26)  // 末尾包实际长度
pack.units = mark
pack.buff  = packArray[mark*26 .. mark*26+unitsLen]
emit signalSendPack(pack)
```

---

## 四、完整工作流程

### 4.1 正常无丢包流程

```
发送方                                      接收方
  │                                           │
  │  write() → lostMark=TRANSFER_ERROR        │
  │  timer 每2ms: emit signalPackStart        │
  │──────── 包头帧 [0xAA...0x0A...] ─────────►│
  │                                           │ readStart():
  │                                           │   初始化缓冲区, 启动5s超时
  │◄──────── 控制帧 [0xBB...mark=0] ──────────│   emit signalLostMark(mark=0)
  │                                           │
  │  readLostMark(mark=0):                    │
  │    lostMark=0, sendMark=0                 │
  │                                           │
  │  run() 每2ms: sendPack(0), sendPack(1)... │
  │──────── 数据帧 units=0 ───────────────────►│ lostMark→1, 1==0+1: 不触发重传
  │──────── 数据帧 units=1 ───────────────────►│ lostMark→2, 2==1+1: 不触发重传
  │  ...                                       │  ...
  │──────── 数据帧 units=N（最后包）──────────►│ lostMark→TRANSFER_COMPLETE
  │                                           │   emit signalSucceed
  │◄──────── 控制帧 mark=0xFFFFFFFF ──────────│   （上层发出清除缓存指令）
  │                                           │
  │  readLostMark(TRANSFER_COMPLETE):         │
  │    lostMark=TRANSFER_COMPLETE             │
  │  run(): stop(), showProgress(100)         │
```

### 4.2 有丢包的重传流程

假设总共 10 个包（0-9），包 5 丢失：

```
发送方                                      接收方
  │──────── 数据帧 units=0~4 ────────────────►│ lostMark 推进到 5（getMark(5)=0）
  │──────── 数据帧 units=5（丢失）────────────✗│
  │──────── 数据帧 units=6 ───────────────────►│ readPacks(6):
  │                                           │   lostMark 仍为 5（5号未收到）
  │                                           │   5 ≠ 6+1=7 → signalLostMark(mark=5)
  │◄──────── 控制帧 mark=5 ───────────────────│
  │                                           │
  │  readLostMark(mark=5):                    │
  │    lostMark=5, sendMark=6（此时）          │
  │    sendMark(6) >= lostMark(5)             │
  │    → 冷却：20ms 后将 sendMark 回退到 5    │
  │                                           │
  │  run(): sendPack(5,6,7,8,9)              │
  │         sendMark=9+1=10 ≥ markLen(10)    │
  │         → sendMark 回绕到 lostMark=5      │
  │         循环: 5,6,7,8,9,5,6,...           │
  │──────── 数据帧 units=5（重发）────────────►│ readPacks(5):
  │                                           │   setMark(5), lostMark 推进
  │                                           │   无其他缺口: lostMark→TRANSFER_COMPLETE
  │◄──────── 控制帧 mark=0xFFFFFFFF ──────────│
  │  readLostMark(TRANSFER_COMPLETE)          │
  │  run(): stop(), showProgress(100)         │
```

### 4.3 多处丢包场景（连续缺口）

若包 3、5、6 均丢失：

1. 收到包 4 时 → `lostMark=3`，发出 `mark=3`，发送方从 3 开始重发
2. 包 3 补收 → `lostMark=5`（4 已有），发出 `mark=5`，发送方 `sendMark` 快进到 5
3. 包 5、6 补收 → `lostMark=TRANSFER_COMPLETE`，传输完成

---

## 五、关键设计要点总结

### 5.1 lostMark 的双重含义

- **接收方**：`lostMark` = 当前位图中第一个为 0 的 bit 的索引，即"我最早还没收到的包序号"
- **发送方**：`lostMark` = 接收方最后一次告诉我的最早缺口；发送方据此决定循环发送的起点

### 5.2 发送方循环区间 [lostMark, markLen-1]

发送方的 `run()` 循环不是从 0 到 markLen-1，而是从 `lostMark` 到 `markLen-1`，再回到 `lostMark`。这与协议注释中描述的无人机行为完全吻合：

> 无人机的重传逻辑是，从收到重传指令对应的包编号开始按顺序重传该包及后续包，在传输到结尾后循环从重传指令处再次开始，直到5秒内仍然没有收到结束指令时超时。

### 5.3 握手机制（TRANSFER_ERROR 状态）

发送方在 `lostMark == TRANSFER_ERROR` 状态下，每 2ms 重发一次包头帧，直到接收方回应 `mark=0`：

1. 发送方发出包头帧（携带 `size`、`type` 等元信息）
2. 接收方收到后初始化缓冲区，并立即回应 `mark=0`
3. 发送方收到 `mark=0` 后将 `lostMark` 从 `TRANSFER_ERROR` 更新为 0，进入数据发送状态

若 1 秒内收不到任何接收方回应，握手超时，发送方报告失败。

### 5.4 超时保护层次

| 层次 | 位置 | 时间 | 触发条件 |
|------|------|------|----------|
| 包头握手超时 | 发送方 | 1000 ms | 发出包头后 1s 内未收到接收方任何回应 |
| 数据传输超时 | 发送方 | 5000 ms | 最后一次收到接收方信号后 5s 无任何回应 |
| 接收侧超时   | 接收方 | 5000 ms | 最后一次收到数据包后 5s 无任何包到达 |

### 5.5 重复包防御

`readUnits()` 中有 `!getMark(index)` 条件检查：若该包已收到（bit 已置 1），则跳过写入，避免重复包覆盖正确数据。

### 5.6 末尾包处理

最后一个分包的数据可能不足 26 字节，收发两端都做了处理：

- **接收方** (`readUnits`)：`remain = packLen - index*26`；若 `len > remain` 则 `len = remain`
- **发送方** (`sendPack`)：同样计算 `remain = packStart.size - mark*26`；取 `min(26, remain)`

末尾包只写入/读取有效部分，多余的 `buff` 空间不处理，jpg 解析会自动忽略尾部冗余。

---

## 六、实现指南（Python 侧对照）

在 Python 接收端实现中，对应关系如下：

| C++ 组件 | Python 对应 |
|---------|-------------|
| `PackRreceive` | `ImageReceiver` 类中的数据接收逻辑 |
| `PackTransmit` | 无人机固件侧；Python 端只需实现接收逻辑 + 发送控制帧 |
| `unitsMark` 位图 | `ImageInfo.packet_cache` 字典（`key=包序号`） |
| `lostMark` | 遍历 `packet_cache` 找第一个缺失包（等价于 `getLostMark()`） |
| `signalLostMark` | 发送 `[0xBB, 0x08, 0x0A, id, count(2LE), mark(4LE), checksum]` 帧 |
| `TRANSFER_COMPLETE` | `mark = 0xFFFFFFFF`，即调用 `_clean_remote_image()` |
| `readStart` | `on_receive_image_pack_info()` |
| `readPacks` | `on_receive_image_packet_data()` |
| 5s 超时定时器 | `_timeout_timer`（`threading.Timer`） |

### 最小化接收侧伪代码

```python
TRANSFER_COMPLETE = 0xFFFFFFFF

# 收到包头帧时（0xAA...0x0A...）
def on_pack_info(id, count, size, data_type):
    init_receive(id, count, size)       # 重置缓存，记录总大小
    send_control_frame(id, count, mark=0)   # 回应：准备好，从头开始

# 收到数据帧时（0xAA...0x0B...）
def on_pack_data(id, count, units, buff):
    if units not in packet_cache:
        packet_cache[units] = buff          # 写入缓存（跳过重复包）

    lost = get_lostMark()                   # 扫描第一个缺包序号

    if lost != units + 1:
        # 存在缺口，立即请求重传
        send_control_frame(id, count, mark=lost)

    if lost == TRANSFER_COMPLETE:
        assemble_data()                     # 拼装完整数据
        send_control_frame(id, count, mark=0xFFFFFFFF)  # 通知远端清除缓存

def get_lostMark(total_packets, packet_cache):
    for i in range(total_packets):
        if i not in packet_cache:
            return i
    return TRANSFER_COMPLETE

def send_control_frame(id, count, mark):
    # [0xBB, 0x08, 0x0A, id(1), count(2LE), mark(4LE), checksum(1)]
    payload = bytes([0x0A, id]) + count.to_bytes(2, 'little') + mark.to_bytes(4, 'little')
    header  = bytes([0xBB, 0x08])
    checksum = sum(header + payload) & 0xFF
    send_bytes(header + payload + bytes([checksum]))
```

