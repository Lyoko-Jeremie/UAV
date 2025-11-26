from time import sleep
from .AirplaneManagerAdapter import get_airplane_manager, AirplaneManager


class FH0C:
    airs: AirplaneManager = get_airplane_manager()

    def sleep(self, time):
        """sleep 单位 秒"""
        sleep(time)
        pass

    def destroy(self):
        self.airs.destroy()
        pass

    def add_uav(self, port: str):
        """添加（注册）无人机"""
        a = self.airs.get_airplane_extended(port)
        a.airplane_mode(1)
        pass

    def p(self, port: str):
        return self.airs.get_airplane_extended(port)

    def land(self, port: str):
        """降落"""
        self.p(port).land()

    # def emergency(self, port: str):
    #     self.p(port).stop()

    def takeoff(self, port: str, high: int):
        """起飞到指定高度 单位cm"""
        self.p(port).takeoff(high)

    def up(self, port: str, distance: int):
        """上升指定距离 单位cm"""
        self.p(port).up(distance)

    def down(self, port: str, distance: int):
        """下降指定距离 单位cm"""
        self.p(port).down(distance)

    def forward(self, port: str, distance: int):
        """前进指定距离 单位cm"""
        self.p(port).forward(distance)

    def back(self, port: str, distance: int):
        """后退指定距离 单位cm"""
        self.p(port).back(distance)

    def left(self, port: str, distance: int):
        """左移指定距离 单位cm"""
        self.p(port).left(distance)

    def right(self, port: str, distance: int):
        """右移指定距离 单位cm"""
        self.p(port).right(distance)

    def goto(self, port: str, x: int, y: int, h: int):
        """移动到指定坐标处"""
        self.p(port).goto(x, y, h)

    def flip(self, port: str, direction: str):
        """做翻转（翻跟头）动作
        :param port:
        :param direction: 翻转方向：f(向前)/b(向后)/l(向左)/r(向右)
        """
        if direction == 'f':
            self.p(port).flip_forward()
        elif direction == 'b':
            self.p(port).flip_back()
        elif direction == 'r':
            self.p(port).flip_right()
        elif direction == 'l':
            self.p(port).flip_left()

    def rotate(self, port: str, degree: int):
        """顺时旋转指定角度"""
        self.p(port).rotate(degree)

    def cw(self, port: str, degree: int):
        """顺时针旋转指定角度"""
        self.p(port).cw(degree)

    def ccw(self, port: str, degree: int):
        """逆时针旋转指定角度"""
        self.p(port).ccw(degree)

    def speed(self, port: str, speed: int):
        """设置飞行速度"""
        self.p(port).speed(speed)

    def high(self, port: str, high: int):
        """移动到指定高度处"""
        self.p(port).high(high)

    def led(self, port: str, r: int, g: int, b: int):
        """设置无人机led色彩"""
        self.p(port).led(r, g, b)

    def bln(self, port: str, r: int, g: int, b: int):
        """设置无人机led呼吸灯色彩"""
        self.p(port).bln(r, g, b)

    def rainbow(self, port: str, r: int, g: int, b: int):
        """设置无人机led彩虹色彩"""
        self.p(port).rainbow(r, g, b)

    def mode(self, port: str, mode: int):
        """设置无人机飞行模式
        :param port:
        :param mode: 1常规2巡线3跟随4单机编队 通常情况下使用模式4
        """
        self.p(port).airplane_mode(mode)

    # def color_detect(self, port: str, L_L: int, L_H: int, A_L: int, A_H: int, B_L: int, B_H: int):
    #     """颜色检测 检测指定颜色
    #     L_*/A_*/B_* 为色彩在 Lab 色彩空间上的L/a/b三个色彩通道
    #     *_L/*_H 为色彩在 Lab 色彩空间上各个的色彩通道的上下限范围
    #     """
    #     self.p(port).vision_color(L_L, L_H, A_L, A_H, B_L, B_H)

    # def color_detect_label(self, port: str, label: str):
    #     self.p(port).color_detect_label(label)

    # def vision_mode(self, port: str, mode: int):
    #     """设置视觉工作模式
    #     :param port:
    #     :param mode: 1点检测2线检测3标签检测4二维码扫描5条形码扫描
    #     """
    #     self.p(port).vision_mode(mode)

    def stop(self, port: str):
        """悬停"""
        self.p(port).stop()

    def hover(self, port: str):
        """悬停"""
        self.p(port).hover()

    pass
