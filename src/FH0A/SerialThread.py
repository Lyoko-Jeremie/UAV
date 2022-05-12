import serial
from queue import Queue, Empty
from time import sleep
from threading import Thread

from .ReadDataParser import ReadDataParser, BaseInfo, SensorInfo, VisionSensorInfo, HardwareInfo, SingleSettingInfo, \
    MultiSettingInfo
from .CommandConstructor import CommandConstructor
from .QueueSignal import QueueSignal


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
    """
    serial port write worker thread, this function do the write task in a independent thread.
    this function must run in a independent thread
    :param thead_local: the thread local data object
    """
    print("task_write")
    while True:
        sleep(0.1)
        try:
            # check if exit signal comes from main thread
            if thead_local.exit_queue.get(block=False) is QueueSignal.SHUTDOWN:
                break
        except Empty:
            pass
        try:
            # check if new command comes from CommandConstructor
            d = thead_local.q.get(block=False, timeout=-1)
            if isinstance(d, tuple):
                print("Tuple:", (d[0], d[1].hex(' ')))
                if d[0] is QueueSignal.CMD and len(d[1]) > 0:
                    thead_local.latest_cmd = d[1]
                    pass
                pass
        except Empty:
            pass
        # send cmd
        # must send multi time to ensure command are sent successfully
        if thead_local.latest_cmd is not None and len(thead_local.latest_cmd) > 0:
            # print("write:", thead_local.latest_cmd)
            thead_local.s.write(thead_local.latest_cmd)
        pass
    print("task_write done.")
    pass


def task_read(thead_local: ThreadLocal):
    """
    serial port read worker thread, this function do the write task in a independent thread.
    this function must run in a independent thread
    :param thead_local: the thread local data object
    """
    print("task_read")
    while True:
        sleep(0.1)
        try:
            # check if exit signal comes from main thread
            if thead_local.exit_queue.get(block=False) is QueueSignal.SHUTDOWN:
                break
        except Empty:
            pass
        # read from serial port
        d = thead_local.s.read(65535)
        # write new data to ReadDataParser
        thead_local.rdp.push(d)
    print("task_read done.")
    pass


class SerialThreadCore:
    """
    the core function of serial control , this can run in main thread or a independent thread.
    """

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
        """this is a debug function, dont call it directly in normal"""
        self.q_write.put((QueueSignal.CMD, cmd))
        pass

    def shutdown(self):
        """
        this function safely shutdown serial port and the control thread,
         it do all the cleanup task.
        """
        # send exit signal comes to worker thread
        self.thead_local_write.exit_queue.put(QueueSignal.SHUTDOWN)
        self.thead_local_read.exit_queue.put(QueueSignal.SHUTDOWN)
        self.thead_local_write.t.join()
        self.thead_local_read.t.join()
        self.s.close()
        pass

    def base_info(self) -> BaseInfo:
        return self.thead_local_read.rdp.get_base_info()

    def sensor_info(self) -> SensorInfo:
        return self.thead_local_read.rdp.get_sensor_info()

    def vision_sensor_info(self) -> VisionSensorInfo:
        return self.thead_local_read.rdp.get_vision_sensor_info()

    def hardware_info(self) -> HardwareInfo:
        return self.thead_local_read.rdp.get_hardware_info()

    def single_setting_info(self) -> SingleSettingInfo:
        return self.thead_local_read.rdp.get_single_setting_info()

    def multi_setting_info(self) -> MultiSettingInfo:
        return self.thead_local_read.rdp.get_multi_setting_info()

    pass


class SerialThread(SerialThreadCore):
    """
    this class extends SerialThreadCore, and implements more useful functions
    """

    cc: CommandConstructor = None

    def __init__(self, port: str):
        super().__init__(port)
        self.ss = CommandConstructor(self.thead_local_write.q)
        print("ss", self.ss)
        pass

    def send(self) -> CommandConstructor:
        return self.ss

    pass


# class SerialThreadWrapper(SerialThread):
#
#     def __init__(self, port: str):
#         super().__init__(port)
#         pass
#
#     pass


# TODO manual test code on here
if __name__ == '__main__':
    # st = SerialThreadWrapper("COM3")
    st = SerialThread("COM3")
    sleep(2)
    # d = bytearray(b'\xBB\x0B\xF3\x08\x02\x00\x00\x00\x02')
    # sum = sum(d)
    # print(sum, sum & 0xFF)
    # d.append(sum & 0xFF)
    # print(d.hex(' '))
    # st.send_cmd(d)
    print("st", st)
    print("st.s", st.s)
    print("st.cc", st.cc)
    print("st.call()", st.send())
    print("st.hardware_info", st.hardware_info())
    # st.call().led(2, 255, 0, 255)
    st.send().read_hardware_setting()
    sleep(0.5)
    st.send().read_multi_setting()
    sleep(0.5)
    st.send().read_single_setting()
    sleep(0.5)
    st.send().takeoff(50)
    sleep(5)
    st.send().up(50)
    sleep(1)
    print("st.base_info", st.base_info())
    print("st.sensor_info", st.sensor_info())
    print("st.vision_sensor_info", st.vision_sensor_info())
    sleep(5)
    # st.send().flip_forward(1)
    # sleep(5)
    # st.send().cw(90)
    # sleep(5)
    st.send().land()
    sleep(1)
    print("st.base_info", st.base_info())
    print("st.sensor_info", st.sensor_info())
    print("st.vision_sensor_info", st.vision_sensor_info())
    print("st.hardware_info", st.hardware_info())
    print("st.single_setting_info", st.single_setting_info())
    print("st.multi_setting_info", st.multi_setting_info())
    sleep(1)
    st.shutdown()
    pass
