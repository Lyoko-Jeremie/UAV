from time import sleep

from src.FH0A import get_airplane_manager
from src.FH0A.AirplaneManagerAdapter import AirplaneManager
from src.FH0A.AirplaneManagerAdapter import AirplaneController

if __name__ == '__main__':
    m: AirplaneManager = get_airplane_manager()

    # print('airplanes_table', m.airplanes_table)

    m.flush()

    # print('airplanes_table', m.airplanes_table)

    print(m.start())

    a = m.get_airplane_extended('COM3')
    b = m.get_airplane_extended('COM4')
    c = m.get_airplane_extended('COM5')
    d = m.get_airplane_extended('COM6')
    e = m.get_airplane_extended('COM7')
    f = m.get_airplane_extended('COM8')

    add_uav(a)
    add_uav(b)
    add_uav(c)
    add_uav(d)
    add_uav(e)
    add_uav(f)
    sleep(3)

    takeoff(a, 100)
    takeoff(b, 100)
    takeoff(c, 100)
    takeoff(d, 100)
    takeoff(e, 100)
    takeoff(f, 100)
    sleep(4)

    mode(a, 4)
    mode(b, 4)
    mode(c, 4)
    mode(d, 4)
    mode(e, 4)
    mode(f, 4)
    sleep(2)

    speed(a, 80)
    speed(b, 80)
    speed(c, 70)
    speed(d, 80)
    speed(e, 80)
    speed(f, 80)
    sleep(1)
    # 箭头开始
    goto(b, 200, 150, 100)
    goto(f, 300, 200, 100)
    goto(d, 400, 150, 100)
    sleep(3)

    goto(a, 100, 100, 100)
    goto(c, 300, 100, 100)
    goto(e, 500, 100, 100)
    sleep(3)
    # 箭头完成
    # 灯光开始
    led(d, 0, 255, 0)
    led(f, 255, 0, 0)
    led(b, 255, 255, 0)

    led(a, 0, 0, 255)
    led(c, 255, 255, 255)
    led(e, 255, 0, 255)
    sleep(2)

    # 变换1
    led(e, 0, 255, 0)
    led(d, 255, 0, 0)
    led(f, 255, 255, 0)

    led(b, 0, 0, 255)
    led(a, 255, 255, 255)
    led(c, 255, 0, 255)
    sleep(1)
    # 变换2
    led(c, 255, 0, 0)
    led(e, 255, 255, 0)
    led(d, 0, 255, 0)

    led(f, 0, 0, 255)
    led(b, 255, 255, 255)
    led(a, 255, 0, 255)
    sleep(1)
    # 变换2
    led(a, 255, 0, 0)
    led(c, 255, 255, 0)
    led(e, 0, 255, 0)

    led(d, 0, 0, 255)
    led(f, 255, 255, 255)
    led(b, 255, 0, 255)
    sleep(1)
    # 六角星
    speed(c, 80)
    sleep(1)
    goto(b, 200, 550, 100)
    goto(f, 300, 600, 100)
    goto(d, 400, 550, 100)
    goto(a, 200, 400, 100)
    goto(c, 300, 350, 100)
    goto(e, 400, 400, 100)
    sleep(5)
    # 六角星转动1
    speed(a, 50)
    speed(b, 50)
    speed(c, 50)
    speed(d, 50)
    speed(e, 50)
    speed(f, 50)
    sleep(1)
    # goto(a,200,550,100)
    # goto(b,300,600,100)
    # goto(f,400,550,100)
    # goto(c,200,400,100)

    # goto(d,400,400,100)
    # goto(e,300,350,100)
    # sleep(6)

    # goto(c,200,550,100)
    # goto(a,300,600,100)
    # goto(b,400,550,100)
    # goto(e,200,400,100)
    # goto(d,300,350,100)
    # goto(f,400,400,100)

    # sleep(6)

    # goto(e,200,550,100)
    # goto(c,300,600,100)
    # goto(a,400,550,100)
    # goto(d,200,400,100)

    # goto(b,400,400,100)
    # goto(f,300,350,100)
    # sleep(5)

    # goto(d,200,550,100)
    # goto(e,300,600,100)
    # goto(c,400,550,100)
    # goto(f,200,400,100)

    # goto(a,400,400,100)
    # goto(b,300,350,100)
    # sleep(3)

    # goto(f,200,550,100)
    # goto(d,300,600,100)
    # goto(e,400,550,100)
    # goto(b,200,400,100)

    # goto(c,400,400,100)
    # goto(a,300,350,100)
    # sleep(5)

    # goto(b,200,550,100)
    # goto(f,300,600,100)
    # goto(d,400,550,100)
    # goto(a,200,400,100)

    # goto(e,400,400,100)
    # goto(c,300,350,100)
    # sleep(5)
    # 转动完成
    # 外扩
    speed(a, 80)
    speed(b, 80)
    speed(c, 80)
    speed(d, 80)
    speed(e, 80)
    speed(f, 80)
    sleep(1)

    bln(a, 255, 0, 0)
    bln(b, 255, 0, 0)
    bln(c, 255, 0, 0)
    bln(d, 255, 0, 0)
    bln(e, 255, 0, 0)
    bln(f, 255, 0, 0)
    sleep(1)
    goto(a, 300, 250, 100)
    goto(c, 500, 400, 100)
    goto(e, 500, 550, 100)
    goto(d, 300, 700, 100)
    goto(f, 100, 550, 100)
    goto(b, 100, 400, 100)
    sleep(6)

    speed(a, 70)
    speed(b, 70)
    speed(c, 70)
    speed(d, 70)
    speed(e, 70)
    speed(f, 70)
    sleep(1)
    # 外扩完成，逆时针转动1

    goto(b, 300, 250, 100)
    goto(a, 500, 400, 100)
    goto(c, 500, 550, 100)
    goto(e, 300, 700, 100)
    goto(d, 100, 550, 100)
    goto(f, 100, 400, 100)
    sleep(4)
    # 外扩完成，逆时针转动2
    goto(f, 300, 250, 100)
    goto(b, 500, 400, 100)
    goto(a, 500, 550, 100)
    goto(c, 300, 700, 100)
    goto(e, 100, 550, 100)
    goto(d, 100, 400, 100)
    sleep(4)
    # 外扩完成，逆时针转动3
    goto(d, 300, 250, 100)
    goto(f, 500, 400, 100)
    goto(b, 500, 550, 100)
    goto(a, 300, 700, 100)
    goto(c, 100, 550, 100)
    goto(e, 100, 400, 100)
    sleep(4)
    # 外扩完成，逆时针转动4
    goto(e, 300, 250, 100)
    goto(d, 500, 400, 100)
    goto(f, 500, 550, 100)
    goto(b, 300, 700, 100)
    goto(a, 100, 550, 100)
    goto(c, 100, 400, 100)
    sleep(4)
    # 外扩完成，逆时针转动5
    goto(c, 300, 250, 100)
    goto(e, 500, 400, 100)
    goto(d, 500, 550, 100)
    goto(f, 300, 700, 100)
    goto(b, 100, 550, 100)
    goto(a, 100, 400, 100)
    sleep(4)
    # 外扩完成，逆时针转动6完成
    goto(a, 300, 250, 100)
    goto(c, 500, 400, 100)
    goto(e, 500, 550, 100)
    goto(d, 300, 700, 100)
    goto(f, 100, 550, 100)
    goto(b, 100, 400, 100)
    sleep(6)
    # 一排
    rainbow(a, 100, 0, 0)
    rainbow(b, 100, 0, 0)
    rainbow(c, 100, 0, 0)
    rainbow(d, 100, 0, 0)
    rainbow(e, 100, 0, 0)
    rainbow(f, 100, 0, 0)
    sleep(1)
    goto(d, 300, 700, 80)
    goto(f, 300, 600, 80)
    goto(e, 300, 500, 80)

    goto(b, 300, 400, 80)
    goto(c, 300, 300, 80)
    goto(a, 300, 200, 80)
    sleep(3)
    # 开始波浪
    # 1
    d.goto(300, 700, 60)
    f.goto(300, 600, 130)
    e.goto(300, 500, 60)

    b.goto(300, 400, 130)
    c.goto(300, 300, 60)
    a.goto(300, 200, 130)
    sleep(2)
    # 2
    d.goto(300, 700, 130)
    f.goto(300, 600, 60)
    e.goto(300, 500, 130)

    b.goto(300, 400, 60)
    c.goto(300, 300, 130)
    a.goto(300, 200, 60)
    sleep(2)
    # 最后
    d.goto(300, 700, 60)
    f.goto(300, 600, 130)
    e.goto(300, 500, 60)

    b.goto(300, 400, 130)
    c.goto(300, 300, 60)
    a.goto(300, 200, 130)
    sleep(3)
    # 翻滚
    f.flip_forward()
    b.flip_back()
    a.flip_forward()
    sleep(3)
    # 一排结束
    d.goto(300, 700, 80)
    f.goto(300, 600, 80)
    e.goto(300, 500, 80)

    b.goto(300, 400, 80)
    c.goto(300, 300, 80)
    a.goto(300, 200, 80)
    sleep(3)
    # 33换位
    d.goto(450, 700, 100)
    f.goto(450, 600, 100)
    e.goto(450, 500, 100)

    b.goto(150, 400, 100)
    c.goto(150, 300, 100)
    a.goto(150, 200, 100)
    sleep(3)
    # 水平上下移动
    d.goto(450, 400, 100)
    f.goto(450, 300, 100)
    e.goto(450, 200, 100)

    b.goto(150, 700, 100)
    c.goto(150, 600, 100)
    a.goto(150, 500, 100)
    sleep(3)
    # 水平上下移动
    b.goto(450, 700, 100)
    c.goto(450, 600, 100)
    a.goto(450, 500, 100)

    d.goto(150, 400, 100)
    f.goto(150, 300, 100)
    e.goto(150, 200, 100)
    sleep(3)
    # 换位完成，成型
    b.goto(400, 650, 100)
    c.goto(400, 500, 100)
    a.goto(400, 350, 100)

    d.goto(200, 650, 100)
    f.goto(200, 500, 100)
    e.goto(200, 350, 100)
    sleep(3)

    a.led(255, 0, 0)
    b.led(255, 0, 0)
    c.led(255, 0, 0)
    d.led(255, 0, 0)
    e.led(255, 0, 0)
    f.led(255, 0, 0)
    sleep(1)

    # 五角星
    b.goto(300, 700, 100)
    c.goto(450, 500, 100)
    a.goto(400, 300, 100)
    f.goto(300, 500, 100)
    d.goto(150, 500, 100)
    e.goto(200, 300, 100)
    sleep(3)

    b.goto(300, 700, 60)
    c.goto(450, 500, 60)
    a.goto(400, 300, 60)
    f.goto(300, 500, 60)
    d.goto(150, 500, 60)
    e.goto(200, 300, 60)
    sleep(5)

    a.land()
    b.land()
    c.land()
    d.land()
    e.land()
    f.land()
    sleep(5)





    a.shutdown()
    b.shutdown()
    c.shutdown()
    d.shutdown()
    e.shutdown()
    f.shutdown()
    exit(0)
