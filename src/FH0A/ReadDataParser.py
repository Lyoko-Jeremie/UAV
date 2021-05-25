from typing import List, Dict, Any, Tuple, Union, Literal

Header_Power_Info = b'\xAA\x0D\x07'
Header_Sensor_info = b'\xAA\x19\x30'
Header_Vision_Sensor_info = b'\xAA\x14\x01'
Header_Others = '\xAA\x09\xf1'
Header_Others_Hardware_Info = '\xAA\x00\x00'
Header_Others_MultiSetting_Info = '\xAA\x00\x04'
Header_Others_SingleSetting_Info = '\xAA\x00\x05'


class ReadDataParser:
    read_buffer: bytearray = bytearray()
    split_buffer: List[bytearray] = []

    def push(self, data: Union[bytearray, bytes]):
        self.read_buffer = self.read_buffer + data
        self.try_parse()
        pass

    def try_parse(self):
        if len(self.read_buffer) > 3:
            header = self.read_buffer[0:3]
            size = header[1]
            # print("header", header, size, header[0], header[1], header[2])
            if header == Header_Power_Info:
                data = self.read_buffer[0: size + 3]
                # print("Header_Power_Info", 0, size, len(data), data)
                self.power_info(data)
                pass
            elif header == Header_Sensor_info:
                data = self.read_buffer[0: size + 3]
                # print("Header_Sensor_info", 0, size, len(data), data)
                self.sensor_info(data)
                pass
            elif header == Header_Vision_Sensor_info:
                data = self.read_buffer[0: size + 3]
                # print("Header_Vision_Sensor_info", 0, size, len(data), data)
                self.vision_sensor_info(data)
                pass
            elif header == Header_Others:
                data = self.read_buffer[0: size + 3]
                # print("Header_Others", 0, size, len(data), data)
                self.other(data)
                pass
            elif header[0] == b'\xAA':
                flag = header[2]
                data = self.read_buffer[0: size + 3]
                if flag == b'\x00':
                    # Header_Others_Hardware_Info like
                    self.hardware_info(data)
                    pass
                elif flag == b'\x04':
                    # Header_Others_MultiSetting_Info like
                    self.multi_setting_info(data)
                    pass
                elif flag == b'\x05':
                    # Header_Others_SingleSetting_Info like
                    self.single_setting_info(data)
                    pass
                pass

            if len(self.read_buffer) > size + 3:
                self.read_buffer = self.read_buffer[size + 3:]
                self.try_parse()
                pass
            pass
        pass

    def power_info(self, data: bytearray):
        print("Power_Info", data.hex(' '))
        # TODO
        pass

    def sensor_info(self, data: bytearray):
        print("Sensor_info", data.hex(' '))
        # TODO
        pass

    def vision_sensor_info(self, data: bytearray):
        print("Vision_Sensor_info", data.hex(' '))
        # TODO
        pass

    def other(self, data: bytearray):
        print("other", data.hex(' '))
        # empty
        pass

    def hardware_info(self, data: bytearray):
        print("hardware_info", data.hex(' '))
        # TODO
        pass

    def multi_setting_info(self, data: bytearray):
        print("multi_setting_info", data.hex(' '))
        # TODO
        pass

    def single_setting_info(self, data: bytearray):
        print("single_setting_info", data.hex(' '))
        # TODO
        pass

    pass
