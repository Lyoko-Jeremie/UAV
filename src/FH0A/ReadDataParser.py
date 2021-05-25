from typing import List, Dict, Any, Tuple, Union, Literal

Header_Power_Info = b'\xAA\x0D\x07'
Header_Sensor_info = b'\xAA\x19\x30'
Header_Vision_Sensor_info = b'\xAA\x14\x01'


class ReadDataParser:
    read_buffer: bytearray = bytearray()
    split_buffer: List[bytearray] = []

    def push(self, data: Union[bytearray, bytes]):
        self.read_buffer = self.read_buffer + data
        self.try_parse()
        pass

    def try_parse(self):
        # TODO
        # self.read_buffer.find()
        if self.read_buffer.find(Header_Power_Info) != -1:
            self.read_buffer[self.read_buffer.find(Header_Power_Info),]
            pass
        pass

    pass
