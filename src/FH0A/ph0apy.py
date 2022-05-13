from time import sleep
from .AirplaneManagerAdapter import get_airplane_manager, AirplaneManager


class FH0A:
    airs: AirplaneManager = get_airplane_manager()

    def sleep(self, time):
        sleep(time)
        pass

    def add_uav(self, port: str):
        self.airs.get_airplane_extended(port)
        pass

    def p(self, port: str):
        return self.airs.get_airplane_extended(port)

    def land(self, port: str):
        self.p(port).land()

    # def emergency(self, port: str):
    #     self.p(port).stop()

    def takeoff(self, port: str, high: int):
        self.p(port).takeoff(high)

    def up(self, port: str, distance: int):
        self.p(port).up(distance)

    def down(self, port: str, distance: int):
        self.p(port).down(distance)

    def forward(self, port: str, distance: int):
        self.p(port).forward(distance)

    def back(self, port: str, distance: int):
        self.p(port).back(distance)

    def left(self, port: str, distance: int):
        self.p(port).left(distance)

    def right(self, port: str, distance: int):
        self.p(port).right(distance)

    def goto(self, port: str, x: int, y: int, h: int):
        self.p(port).goto(x, y, h)

    def flip(self, port: str, direction: str):
        if direction == 'f':
            self.p(port).flip_forward()
        elif direction == 'b':
            self.p(port).flip_back()
        elif direction == 'r':
            self.p(port).flip_right()
        elif direction == 'l':
            self.p(port).flip_left()

    def rotate(self, port: str, degree: int):
        self.p(port).rotate(degree)

    def cw(self, port: str, degree: int):
        self.p(port).cw(degree)

    def ccw(self, port: str, degree: int):
        self.p(port).ccw(degree)

    def speed(self, port: str, speed: int):
        self.p(port).speed(speed)

    def high(self, port: str, high: int):
        self.p(port).high(high)

    def led(self, port: str, r: int, g: int, b: int):
        self.p(port).led(r, g, b)

    # def bln(self, port: str, r: int, g: int, b: int):
    #     self.p(port).bln(r, g, b)

    # def rainbow(self, port: str, r: int, g: int, b: int):
    #     self.p(port).rainbow(r, g, b)

    def mode(self, port: str, mode: int):
        self.p(port).airplane_mode(mode)

    def color_detect(self, port: str, L_L: int, L_H: int, A_L: int, A_H: int, B_L: int, B_H: int):
        self.p(port).vision_color(L_L, L_H, A_L, A_H, B_L, B_H)

    # def color_detect_label(self, port: str, label: str):
    #     self.p(port).color_detect_label(label)

    def stop(self, port: str):
        self.p(port).stop()

    def hover(self, port: str):
        self.p(port).hover()

    pass
