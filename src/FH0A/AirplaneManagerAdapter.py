from typing import Dict, Optional

from .SerialThread import SerialThread


class AirplaneController(object):
    s: SerialThread

    def __init__(self, port):
        self.s = SerialThread(port=port)
        pass

    def takeoff(self, high: int, ):
        self.s.send().takeoff(high)
        pass

    def land(self, ):
        self.s.send().land()
        pass

    def up(self, distance: int):
        self.s.send().up(distance)
        pass

    def down(self, distance: int):
        self.s.send().down(distance)
        pass

    def forward(self, distance: int):
        self.s.send().forward(distance)
        pass

    def back(self, distance: int):
        self.s.send().back(distance)
        pass

    def left(self, distance: int):
        self.s.send().left(distance)
        pass

    def right(self, distance: int):
        self.s.send().right(distance)
        pass

    def goto(self, x: int, y: int, h: int):
        self.s.send().arrive(x, y, h)
        pass

    def rotate(self, degree: int):
        self.s.send().rotate(degree)
        pass

    def cw(self, degree: int):
        self.s.send().cw(degree)
        pass

    def ccw(self, degree: int):
        self.s.send().ccw(degree)
        pass

    def speed(self, speed: int):
        self.s.send().speed(speed)
        pass

    def high(self, high: int):
        self.s.send().high(high)
        pass

    def led(self, mode: int, r: int, g: int, b: int):
        self.s.send().led(mode, r, g, b)
        pass

    # def bln(self, r: int, g: int, b: int):
    #     self.s.send().bln(r, g, b)
    #     pass

    # def rainbow(self, r: int, g: int, b: int):
    #     self.s.send().rainbow(r, g, b)
    #     pass

    def stop(self):
        self.s.send().hovering()
        pass

    def hover(self):
        self.s.send().hovering()
        pass

    pass


class AirplaneControllerExtended(AirplaneController):

    def airplane_mode(self, mode: int):
        self.s.send().airplane_mode(mode)
        pass

    def move(self, direction: int, distance: int):
        self.s.send().move(direction, distance)
        pass

    def flip(self, direction: int, circle: int = 1):
        self.s.send().flip(direction, circle)
        pass

    def flip_forward(self, circle: int = 1):
        self.s.send().flip_forward(circle)
        pass

    def flip_back(self, circle: int = 1):
        self.s.send().flip_back(circle)
        pass

    def flip_left(self, circle: int = 1):
        self.s.send().flip_left(circle)
        pass

    def flip_right(self, circle: int = 1):
        self.s.send().flip_right(circle)
        pass

    def vision_mode(self, mode: int):
        self.s.send().vision_mode(mode)
        pass

    def vision_color(self, L_L: int, L_H: int, A_L: int, A_H: int, B_L: int, B_H: int, ):
        self.s.send().vision_color(L_L, L_H, A_L, A_H, B_L, B_H)
        pass

    def request_read_multi_setting_info(self):
        self.s.send().read_multi_setting()
        pass

    def multi_setting_info(self):
        self.s.multi_setting_info()
        pass

    def request_read_single_setting_info(self):
        self.s.send().read_single_setting()
        pass

    def single_setting_info(self):
        self.s.single_setting_info()
        pass

    def request_read_hardware_info(self):
        self.s.send().read_hardware_setting()
        pass

    def hardware_setting_info(self):
        self.s.hardware_info()
        pass

    def vision_sensor_info(self):
        self.s.vision_sensor_info()
        pass

    def sensor_info(self):
        self.s.sensor_info()
        pass

    def base_info(self):
        self.s.base_info()
        pass

    def shutdown(self):
        self.s.shutdown()
        pass

    pass


class AirplaneManager(object):
    airplanes_table: Dict[str, AirplaneControllerExtended] = {}

    def ping(self):
        return {'ok': True, 'r': ''}

    def ping_volatile(self):
        return {'ok': True, 'r': ''}

    def start(self):
        return {'ok': True, 'r': ''}

    def get_airplane(self, id: str) -> Optional[AirplaneController]:
        return self.get_airplane_extended(id)
        pass

    def get_airplane_extended(self, id: str) -> Optional[AirplaneControllerExtended]:
        a = self.airplanes_table.get(id)
        if a is not None:
            return a
        else:
            self.airplanes_table[id] = AirplaneControllerExtended(id)
            return self.airplanes_table.get(id)
        pass

    def flush(self):
        return None
        pass

    def destroy(self):
        for i in self.airplanes_table.values():
            i.shutdown()
            pass
        self.airplanes_table = {}


airplane_manager_singleton = AirplaneManager()


def get_airplane_manager():
    return airplane_manager_singleton
