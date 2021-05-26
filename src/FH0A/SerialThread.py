import serial
from queue import Queue, Empty
from time import sleep
from threading import Thread
from typing import List, Dict, Any, Tuple, Union, Literal
from enum import Enum
from struct import pack, unpack, pack_into, unpack_from

from ReadDataParser import ReadDataParser


class QueueSignal(Enum):
    SHUTDOWN = 0
    CMD = 1
    pass


class ThreadLocal:
    """used by thead"""
    latest_cmd: bytearray = None
    q: Queue = None
    s: serial.Serial = None
    t: Thread = None
    exit_queue: Queue = Queue()

    rdp: ReadDataParser = None

    def __init__(self):
        pass

    pass


def task_write(thead_local: ThreadLocal):
    print("task_write")
    while True:
        sleep(0.1)
        try:
            if thead_local.exit_queue.get(block=False) is QueueSignal.SHUTDOWN:
                break
        except Empty:
            pass
        try:
            d = thead_local.q.get(block=False, timeout=-1)
            if isinstance(d, tuple):
                print("Tuple:", d)
                if d[0] is QueueSignal.CMD and len(d[1]) > 0:
                    thead_local.latest_cmd = d[1]
                    pass
                pass
        except Empty:
            pass
        # send cmd
        if thead_local.latest_cmd is not None and len(thead_local.latest_cmd) > 0:
            thead_local.s.write(thead_local.latest_cmd)
        pass
    print("task_write done.")
    pass


def task_read(thead_local: ThreadLocal):
    print("task_read")
    while True:
        sleep(0.1)
        try:
            if thead_local.exit_queue.get(block=False) is QueueSignal.SHUTDOWN:
                break
        except Empty:
            pass
        # TODO read from serial port
        d = thead_local.s.read(65535)
        # print("read:", d)
        # if len(d) > 0:
        thead_local.rdp.push(d)
    print("task_read done.")
    pass


class SerialThreadBase:
    s: serial.Serial = None
    port: str = None
    thead_local_write: ThreadLocal = None
    thead_local_read: ThreadLocal = None

    def __init__(self, port: str):
        self.port = port
        self.q_write = Queue()
        self.q_read = Queue()
        self.s = serial.Serial(port, baudrate=115200, timeout=0.01)

        self.thead_local_write = ThreadLocal()
        self.thead_local_write.q = self.q_write
        self.thead_local_write.s = self.s
        self.thead_local_write.t = Thread(target=task_write, args=(self.thead_local_write,))

        self.thead_local_read = ThreadLocal()
        self.thead_local_read.q = self.q_read
        self.thead_local_read.s = self.s
        self.thead_local_read.rdp = ReadDataParser(self.thead_local_read.q)
        self.thead_local_read.t = Thread(target=task_read, args=(self.thead_local_read,))

        self.thead_local_write.t.start()
        self.thead_local_read.t.start()
        pass

    def send_cmd(self, cmd: bytearray):
        self.q_write.put((QueueSignal.CMD, cmd))
        pass

    def shutdown(self):
        self.thead_local_write.exit_queue.put(QueueSignal.SHUTDOWN)
        self.thead_local_read.exit_queue.put(QueueSignal.SHUTDOWN)
        self.thead_local_write.t.join()
        self.thead_local_read.t.join()
        self.s.close()
        pass

    def base_info(self):
        return self.thead_local_read.rdp.m_base_info

    def sensor_info(self):
        return self.thead_local_read.rdp.m_sensor_info

    def vision_sensor_info(self):
        return self.thead_local_read.rdp.m_vision_sensor_info

    def hardware_info(self):
        return self.thead_local_read.rdp.m_hardware_info

    def single_setting_info(self):
        return self.thead_local_read.rdp.m_single_setting_info

    def multi_setting_info(self):
        return self.thead_local_read.rdp.m_multi_setting_info

    pass


class CmdType(Enum):
    MULTI_SETTINGS = 1
    SINGLE_SETTINGS = 2
    SINGLE_CONTROL = 3
    pass


class SerialThread(SerialThreadBase):
    order_last = 0

    def order_count(self):
        self.order_last = (self.order_last + 2) % 127
        return self.order_last
        pass

    def check_sum(self, data: bytearray):
        return sum(data) & 0xFF
        pass

    def __init__(self, port: str):
        super().__init__(port)
        pass

    def join_cmd(self, type: CmdType, params: bytearray):
        header = bytearray(b'\xBB\x00\x00')
        if type == CmdType.MULTI_SETTINGS:
            header[1] = 0x08
            header[2] = 0x04
            checksum = bytearray(self.check_sum(header + params))
            return header + params + checksum
            pass
        elif type == CmdType.SINGLE_SETTINGS:
            header[1] = 0x07
            header[2] = 0x05
            checksum = bytearray(self.check_sum(header + params))
            return header + params + checksum
            pass
        elif type == CmdType.SINGLE_CONTROL:
            header[1] = 0x0B
            header[2] = 0xF3
            params.append(self.order_count())
            checksum = bytearray(self.check_sum(header + params))
            return header + params + checksum
            pass
        pass

    pass


class SerialThreadWrapper(SerialThread):

    def __init__(self, port: str):
        super().__init__(port)
        pass

    pass


if __name__ == '__main__':
    st = SerialThreadWrapper("COM3")
    sleep(2)
    d = bytearray(b'\xBB\x0B\xF3\x08\x02\x00\x00\x00\x02')
    sum = sum(d)
    print(sum, sum & 0xFF)
    d.append(sum & 0xFF)
    print(d.hex(' '))
    st.send_cmd(d)
    sleep(5)
    st.shutdown()
    pass
