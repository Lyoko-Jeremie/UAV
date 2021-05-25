import serial
from queue import Queue, Empty
from time import sleep
from threading import Thread
from typing import List, Dict, Any, Tuple, Union, Literal
from enum import Enum

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
        # TODO send cmd
        # if thead_local.latest_cmd is not None:
        #     thead_local.s.write(thead_local.latest_cmd)
        pass
    print("task_write done.")
    pass


def task_read(thead_local: ThreadLocal):
    print("task_read")
    thead_local.rdp = ReadDataParser()
    while True:
        sleep(0.1)
        try:
            if thead_local.exit_queue.get(block=False) is QueueSignal.SHUTDOWN:
                break
        except Empty:
            pass
        # TODO read from serial port
        d = thead_local.s.read(1024)
        print("read:", d)
        # if len(d) > 0:
        thead_local.rdp.push(d)
    print("task_read done.")
    pass


class SerialThread:
    s: serial.Serial = None
    port: str = None
    thead_local_write: ThreadLocal = None
    thead_local_read: ThreadLocal = None

    def __init__(self, port: str):
        self.port = port
        self.q_write = Queue()
        self.q_read = Queue()
        self.s = serial.Serial(port, timeout=1)

        self.thead_local_write = ThreadLocal()
        self.thead_local_write.q = self.q_write
        self.thead_local_write.s = self.s
        self.thead_local_write.t = Thread(target=task_write, args=(self.thead_local_write,))

        self.thead_local_read = ThreadLocal()
        self.thead_local_read.q = self.q_read
        self.thead_local_read.s = self.s
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

    pass


if __name__ == '__main__':
    st = SerialThread("COM1")
    st.send_cmd(bytearray())
    sleep(1)
    st.shutdown()
    pass
