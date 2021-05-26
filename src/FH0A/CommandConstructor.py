from queue import Queue
from struct import pack, unpack, pack_into, unpack_from
from enum import Enum

from QueueSignal import QueueSignal


class CmdType(Enum):
    MULTI_SETTINGS = 1
    SINGLE_SETTINGS = 2
    SINGLE_CONTROL = 3
    READ_SETTINGS = 4
    pass


class CommandConstructorCore:
    order_last = 0
    q_write: Queue = None

    def _order_count(self):
        self.order_last = (self.order_last + 1) % 127
        return self.order_last
        pass

    def _check_sum(self, header: bytearray, params: bytearray):
        return bytearray([sum(header + params) & 0xFF])
        pass

    def sendCommand(self, data: bytearray):
        self.q_write.put((QueueSignal.CMD, data), block=True)
        pass

    def __init__(self, q_write: Queue):
        self.q_write = q_write
        pass

    def join_cmd(self, type: CmdType, params: bytearray):
        header = bytearray(b'\xBB\x00\x00')
        if type == CmdType.MULTI_SETTINGS:  # 编队设置 0xBB, 0x08, 0x04
            pack_into("!B", header, 1, 0x08)
            pack_into("!B", header, 2, 0x04)
            # header[1] = 0x08
            # header[2] = 0x04
            checksum = self._check_sum(header, params)
            return header + params + checksum
            pass
        elif type == CmdType.SINGLE_SETTINGS:  # 单机设置 0xBB, 0x07, 0x05
            pack_into("!B", header, 1, 0x07)
            pack_into("!B", header, 2, 0x05)
            # header[1] = 0x07
            # header[2] = 0x05
            checksum = self._check_sum(header, params)
            return header + params + checksum
            pass
        elif type == CmdType.READ_SETTINGS:  # 读设置 0xBB, 0x02, 0x02
            pack_into("!B", header, 1, 0x02)
            pack_into("!B", header, 2, 0x02)
            checksum = self._check_sum(header, params)
            return header + params + checksum
            pass
        elif type == CmdType.SINGLE_CONTROL:  # 串口控制 0xBB, 0x0B, 0xF3
            pack_into("!B", header, 1, 0x0B)
            pack_into("!B", header, 2, 0xF3)
            # header[1] = 0x0B
            # header[2] = 0xF3
            pack_into("!B", params, 9, self._order_count())
            # params.append(self._order_count())
            checksum = self._check_sum(header, params)
            return header + params + checksum
            pass
        pass

    pass


class CommandConstructor(CommandConstructorCore):

    def __init__(self, q_write: Queue):
        super().__init__(q_write)
        pass

    def led(self, mode: int, r: int, g: int, b: int):
        if mode < 0 or mode > 2:
            raise ValueError("mode illegal")

        params = bytearray(10)
        pack_into("!B", params, 0, 0x08)
        pack_into("!B", params, 1, mode)
        pack_into("!B", params, 2, r)
        pack_into("!B", params, 3, g)
        pack_into("!B", params, 4, b)
        cmd = self.join_cmd(CmdType.SINGLE_CONTROL, params)
        print("cmd", cmd.hex(' '))
        self.sendCommand(cmd)

        pass

    def takeoff(self, high: int, ):
        params = bytearray(10)
        pack_into("!B", params, 0, 0x01)
        pack_into("!h", params, 1, high)
        cmd = self.join_cmd(CmdType.SINGLE_CONTROL, params)
        print("cmd", cmd.hex(' '))
        self.sendCommand(cmd)

        pass

    def read_multi_setting(self):
        return self.read_setting(0x02)

    def read_single_setting(self):
        return self.read_setting(0x04)

    def read_hardware_setting(self):
        return self.read_setting(0xA0)

    def read_setting(self, mode: int):
        params = bytearray(1)
        pack_into("!B", params, 0, mode)
        cmd = self.join_cmd(CmdType.READ_SETTINGS, params)
        print("cmd", cmd.hex(' '))
        self.sendCommand(cmd)

        pass

    pass
