import serial
from queue import Queue, Empty
from time import sleep
from threading import Thread

from .ReadDataParser import ReadDataParser, Fh0cBase
from .CommandConstructor import CommandConstructor
from .QueueSignal import QueueSignal


class ThreadLocal:
    """供工作线程使用的本地数据容器"""

    latest_cmd: bytearray | None = None
    """最近一次待发送的命令字节数组，为 None 表示当前没有待发送的命令"""

    latest_cmd_id: int | None = None
    """最近一次命令的唯一标识符，用于与 CLEAN 信号配对，为 None 表示无 ID"""

    latest_cmd_send_count = 0
    """当前命令已发送的次数"""

    latest_cmd_send_count_limit: None | int = None
    """当前命令允许发送的最大次数，超过后自动清除命令；为 None 表示不限制发送次数"""

    # 队列元素格式：(QueueSignal, data|None, int|None,            int|None)
    # CMD  信号：   (CMD,         bytearray, 最大重试次数或None,   消息ID或None)  消息ID用于 CLEAN 信号匹配
    # CLEAN信号：   (CLEAN,       None,      None,                消息ID)        消息ID与 CMD 中的消息ID对应
    q: Queue[tuple[QueueSignal, bytearray | None, int | None, int | None]] = None
    """命令队列，主线程通过此队列向工作线程发送控制信号和命令数据"""

    s: serial.Serial = None
    """串口对象，用于与飞控硬件进行串口通信"""

    t: Thread = None
    """与本数据容器关联的工作线程对象"""

    exit_queue: Queue = Queue()
    """退出信号队列，主线程向此队列放入 SHUTDOWN 信号以通知工作线程退出"""

    rdp: ReadDataParser = None
    """串口数据解析器，用于解析从串口读取到的原始字节流"""

    def __init__(self):
        pass

    pass


def task_write(thead_local: ThreadLocal):
    """
    串口写入工作线程函数，负责将命令通过串口发送给飞控硬件。
    此函数必须在独立线程中运行。

    :param thead_local: 线程本地数据对象，包含串口、命令队列等共享资源
    """
    print("task_write")
    while True:
        sleep(0.1)
        try:
            # 检查主线程是否发来退出信号
            if thead_local.exit_queue.get(block=False) is QueueSignal.SHUTDOWN:
                break
        except Empty:
            pass
        try:
            # 从命令队列中取出新命令
            d = thead_local.q.get(block=False, timeout=-1)
            print("task_write d:", (d[0], d[1].hex(' ')))
            if isinstance(d, tuple):
                if d[0] == QueueSignal.CLEAN:
                    # 收到 CLEAN 信号：若消息ID匹配，则清除当前待发送命令
                    print("if d[0] == QueueSignal.CLEAN")
                    if len(d) > 3 and d[3] is not None and thead_local.latest_cmd_id == d[3]:
                        thead_local.latest_cmd = None
                    pass
                elif d[0] == QueueSignal.CMD and d[1] is not None and len(d[1]) > 0:
                    # 收到 CMD 信号：更新待发送命令及相关参数
                    print("task_write Tuple:", (d[0], d[1].hex(' ')))
                    thead_local.latest_cmd = d[1]
                    thead_local.latest_cmd_send_count = 0
                    if len(d) > 2 and d[2] is not None:
                        thead_local.latest_cmd_send_count_limit = d[2]
                    else:
                        thead_local.latest_cmd_send_count_limit = None
                    if len(d) > 3 and d[3] is not None:
                        thead_local.latest_cmd_id = d[3]
                    else:
                        thead_local.latest_cmd_id = None
                    print("thead_local", thead_local)
                pass
        except Empty:
            pass
        except Exception as ex:
            print("task_write exception:", ex)
        # 发送命令
        # 需要多次发送以确保命令被飞控成功接收
        if thead_local.latest_cmd is not None and len(thead_local.latest_cmd) > 0:
            # print("write:", thead_local.latest_cmd.hex(" "))
            thead_local.s.write(thead_local.latest_cmd)
            thead_local.latest_cmd_send_count += 1
            if thead_local.latest_cmd_send_count_limit is not None and thead_local.latest_cmd_send_count > thead_local.latest_cmd_send_count_limit:
                # 已达到最大发送次数，清除当前命令
                print("task_write: cmd send count limit reached, clean latest_cmd")
                thead_local.latest_cmd = None
                thead_local.latest_cmd_id = None
                thead_local.latest_cmd_send_count_limit = None
                thead_local.latest_cmd_send_count = 0
                pass
        pass
    print("task_write done.")
    pass


def task_read(thead_local: ThreadLocal):
    """
    串口读取工作线程函数，负责从串口持续读取飞控返回的数据并交由解析器处理。
    此函数必须在独立线程中运行。

    :param thead_local: 线程本地数据对象，包含串口、数据解析器等共享资源
    """
    print("task_read")
    while True:
        sleep(0.1)
        try:
            # 检查主线程是否发来退出信号
            if thead_local.exit_queue.get(block=False) is QueueSignal.SHUTDOWN:
                break
        except Empty:
            pass
        # 从串口读取数据
        d = thead_local.s.read(65535)
        # 将读取到的原始数据推入解析器
        thead_local.rdp.push(d)
    print("task_read done.")
    pass


class SerialThreadCore:
    """
    串口控制核心类，可在主线程或独立线程中运行。
    负责管理串口连接、启动读写工作线程，并提供底层数据读取接口。
    """

    s: serial.Serial = None
    port: str = None
    thead_local_write: ThreadLocal = None
    thead_local_read: ThreadLocal = None

    def __init__(self, port: str, airplane: 'AirplaneController'):
        """
        初始化串口控制核心。
        创建串口对象和读写两个工作线程，并启动它们。

        :param self: 类实例自身引用
        :param port: str 串口名称，例如 'COM3'
        :return: 无返回值
        """
        self.port = port
        self.q_write: Queue = Queue()
        self.q_read: Queue = Queue()
        self.s = serial.Serial(port, baudrate=500000, timeout=0.01)

        # 初始化写线程本地数据
        self.thead_local_write = ThreadLocal()
        self.thead_local_write.q = self.q_write
        self.thead_local_write.s = self.s
        self.thead_local_write.t = Thread(target=task_write, args=(self.thead_local_write,))

        # 初始化读线程本地数据
        self.thead_local_read = ThreadLocal()
        self.thead_local_read.q = self.q_read
        self.thead_local_read.s = self.s
        self.thead_local_read.rdp = ReadDataParser(self.thead_local_read.q, airplane.image_receiver)
        self.thead_local_read.t = Thread(target=task_read, args=(self.thead_local_read,))

        # 启动读写工作线程
        self.thead_local_write.t.start()
        self.thead_local_read.t.start()
        pass

    def send_cmd(self, cmd: bytearray):
        """调试用函数，直接将命令放入写队列，正常使用请勿直接调用"""
        self.q_write.put((QueueSignal.CMD, cmd))
        pass

    def shutdown(self):
        """
        安全关闭串口及工作线程，执行所有清理工作。
        向读写工作线程发送退出信号，等待线程结束后关闭串口。
        """
        # 向工作线程发送退出信号
        self.thead_local_write.exit_queue.put(QueueSignal.SHUTDOWN)
        self.thead_local_read.exit_queue.put(QueueSignal.SHUTDOWN)
        self.thead_local_write.t.join()
        self.thead_local_read.t.join()
        self.s.close()
        pass

    def fh0c_base(self) -> Fh0cBase:
        """获取飞控基础状态数据"""
        return self.thead_local_read.rdp.get_fh0c_base()

    pass


class SerialThread(SerialThreadCore):
    """
    SerialThreadCore 的扩展类，封装了 CommandConstructor 以提供更易用的命令发送接口。
    """

    ss: CommandConstructor = None

    def __init__(self, port: str, airplane: 'AirplaneController'):
        super().__init__(port, airplane=airplane)
        self.ss = CommandConstructor(self.thead_local_write.q)
        print("ss", self.ss)
        pass

    def send(self) -> CommandConstructor:
        """获取命令构造器，用于构造并发送飞控命令"""
        return self.ss

    pass

# class SerialThreadWrapper(SerialThread):
#
#     def __init__(self, port: str):
#         super().__init__(port)
#         pass
#
#     pass


# # TODO manual test code on here
# if __name__ == '__main__':
#     # st = SerialThreadWrapper("COM3")
#     st = SerialThread("COM3")
#     sleep(2)
#     # d = bytearray(b'\xBB\x0B\xF3\x08\x02\x00\x00\x00\x02')
#     # sum = sum(d)
#     # print(sum, sum & 0xFF)
#     # d.append(sum & 0xFF)
#     # print(d.hex(' '))
#     # st.send_cmd(d)
#     print("st", st)
#     print("st.s", st.s)
#     print("st.ss", st.ss)
#     print("st.call()", st.send())
#     # print("st.hardware_info", st.hardware_info())
#     # st.call().led(2, 255, 0, 255)
#     # st.send().read_hardware_setting()
#     # sleep(0.5)
#     # st.send().read_multi_setting()
#     # sleep(0.5)
#     # st.send().read_single_setting()
#     sleep(0.5)
#     st.send().takeoff(50)
#     sleep(5)
#     st.send().up(50)
#     sleep(1)
#     # print("st.base_info", st.base_info())
#     # print("st.sensor_info", st.sensor_info())
#     # print("st.vision_sensor_info", st.vision_sensor_info())
#     sleep(5)
#     # st.send().flip_forward(1)
#     # sleep(5)
#     # st.send().cw(90)
#     # sleep(5)
#     st.send().land()
#     sleep(1)
#     # print("st.base_info", st.base_info())
#     # print("st.sensor_info", st.sensor_info())
#     # print("st.vision_sensor_info", st.vision_sensor_info())
#     # print("st.hardware_info", st.hardware_info())
#     # print("st.single_setting_info", st.single_setting_info())
#     # print("st.multi_setting_info", st.multi_setting_info())
#     sleep(1)
#     st.shutdown()
#     pass
