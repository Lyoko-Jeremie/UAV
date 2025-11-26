from uav.FH0A.ph0apy import FH0A

fh = FH0A()
a = 'COM6'
b = 'COM25'
c = 'COM26'
d = 'COM27'
e = 'COM28'
f = 'COM29'

fh.add_uav(a)
fh.add_uav(b)
fh.add_uav(c)
fh.add_uav(d)
fh.add_uav(e)
fh.add_uav(f)
fh.sleep(3)

fh.takeoff(a, 100)
fh.takeoff(b, 100)
fh.takeoff(c, 100)
fh.takeoff(d, 100)
fh.takeoff(e, 100)
fh.takeoff(f, 100)
fh.sleep(4)

fh.mode(a, 4)
fh.mode(b, 4)
fh.mode(c, 4)
fh.mode(d, 4)
fh.mode(e, 4)
fh.mode(f, 4)
fh.sleep(2)

fh.speed(a, 80)
fh.speed(b, 80)
fh.speed(c, 70)
fh.speed(d, 80)
fh.speed(e, 80)
fh.speed(f, 80)
fh.sleep(1)
# 箭头开始
fh.goto(b, 200, 150, 100)
fh.goto(f, 300, 200, 100)
fh.goto(d, 400, 150, 100)
fh.sleep(3)

fh.goto(a, 100, 100, 100)
fh.goto(c, 300, 100, 100)
fh.goto(e, 500, 100, 100)
fh.sleep(3)
# 箭头完成
# 灯光开始
fh.led(d, 0, 255, 0)
fh.led(f, 255, 0, 0)
fh.led(b, 255, 255, 0)

fh.led(a, 0, 0, 255)
fh.led(c, 255, 255, 255)
fh.led(e, 255, 0, 255)
fh.sleep(2)

# 变换1
fh.led(e, 0, 255, 0)
fh.led(d, 255, 0, 0)
fh.led(f, 255, 255, 0)

fh.led(b, 0, 0, 255)
fh.led(a, 255, 255, 255)
fh.led(c, 255, 0, 255)
fh.sleep(1)
# 变换2
fh.led(c, 255, 0, 0)
fh.led(e, 255, 255, 0)
fh.led(d, 0, 255, 0)

fh.led(f, 0, 0, 255)
fh.led(b, 255, 255, 255)
fh.led(a, 255, 0, 255)
fh.sleep(1)
# 变换2
fh.led(a, 255, 0, 0)
fh.led(c, 255, 255, 0)
fh.led(e, 0, 255, 0)

fh.led(d, 0, 0, 255)
fh.led(f, 255, 255, 255)
fh.led(b, 255, 0, 255)
fh.sleep(1)
# 六角星
fh.speed(c, 80)
fh.sleep(1)
fh.goto(b, 200, 550, 100)
fh.goto(f, 300, 600, 100)
fh.goto(d, 400, 550, 100)
fh.goto(a, 200, 400, 100)
fh.goto(c, 300, 350, 100)
fh.goto(e, 400, 400, 100)
fh.sleep(5)
# 六角星转动1
fh.speed(a, 50)
fh.speed(b, 50)
fh.speed(c, 50)
fh.speed(d, 50)
fh.speed(e, 50)
fh.speed(f, 50)
fh.sleep(1)
# fh.goto(a,200,550,100)
# fh.goto(b,300,600,100)
# fh.goto(f,400,550,100)
# fh.goto(c,200,400,100)

# fh.goto(d,400,400,100)
# fh.goto(e,300,350,100)
# fh.sleep(6)

# fh.goto(c,200,550,100)
# fh.goto(a,300,600,100)
# fh.goto(b,400,550,100)
# fh.goto(e,200,400,100)
# fh.goto(d,300,350,100)
# fh.goto(f,400,400,100)

# fh.sleep(6)

# fh.goto(e,200,550,100)
# fh.goto(c,300,600,100)
# fh.goto(a,400,550,100)
# fh.goto(d,200,400,100)

# fh.goto(b,400,400,100)
# fh.goto(f,300,350,100)
# fh.sleep(5)

# fh.goto(d,200,550,100)
# fh.goto(e,300,600,100)
# fh.goto(c,400,550,100)
# fh.goto(f,200,400,100)

# fh.goto(a,400,400,100)
# fh.goto(b,300,350,100)
# fh.sleep(3)

# fh.goto(f,200,550,100)
# fh.goto(d,300,600,100)
# fh.goto(e,400,550,100)
# fh.goto(b,200,400,100)

# fh.goto(c,400,400,100)
# fh.goto(a,300,350,100)
# fh.sleep(5)

# fh.goto(b,200,550,100)
# fh.goto(f,300,600,100)
# fh.goto(d,400,550,100)
# fh.goto(a,200,400,100)

# fh.goto(e,400,400,100)
# fh.goto(c,300,350,100)
# fh.sleep(5)
# 转动完成
# 外扩
fh.speed(a, 80)
fh.speed(b, 80)
fh.speed(c, 80)
fh.speed(d, 80)
fh.speed(e, 80)
fh.speed(f, 80)
fh.sleep(1)

fh.bln(a, 255, 0, 0)
fh.bln(b, 255, 0, 0)
fh.bln(c, 255, 0, 0)
fh.bln(d, 255, 0, 0)
fh.bln(e, 255, 0, 0)
fh.bln(f, 255, 0, 0)
fh.sleep(1)
fh.goto(a, 300, 250, 100)
fh.goto(c, 500, 400, 100)
fh.goto(e, 500, 550, 100)
fh.goto(d, 300, 700, 100)
fh.goto(f, 100, 550, 100)
fh.goto(b, 100, 400, 100)
fh.sleep(6)

fh.speed(a, 70)
fh.speed(b, 70)
fh.speed(c, 70)
fh.speed(d, 70)
fh.speed(e, 70)
fh.speed(f, 70)
fh.sleep(1)
# 外扩完成，逆时针转动1

fh.goto(b, 300, 250, 100)
fh.goto(a, 500, 400, 100)
fh.goto(c, 500, 550, 100)
fh.goto(e, 300, 700, 100)
fh.goto(d, 100, 550, 100)
fh.goto(f, 100, 400, 100)
fh.sleep(4)
# 外扩完成，逆时针转动2
fh.goto(f, 300, 250, 100)
fh.goto(b, 500, 400, 100)
fh.goto(a, 500, 550, 100)
fh.goto(c, 300, 700, 100)
fh.goto(e, 100, 550, 100)
fh.goto(d, 100, 400, 100)
fh.sleep(4)
# 外扩完成，逆时针转动3
fh.goto(d, 300, 250, 100)
fh.goto(f, 500, 400, 100)
fh.goto(b, 500, 550, 100)
fh.goto(a, 300, 700, 100)
fh.goto(c, 100, 550, 100)
fh.goto(e, 100, 400, 100)
fh.sleep(4)
# 外扩完成，逆时针转动4
fh.goto(e, 300, 250, 100)
fh.goto(d, 500, 400, 100)
fh.goto(f, 500, 550, 100)
fh.goto(b, 300, 700, 100)
fh.goto(a, 100, 550, 100)
fh.goto(c, 100, 400, 100)
fh.sleep(4)
# 外扩完成，逆时针转动5
fh.goto(c, 300, 250, 100)
fh.goto(e, 500, 400, 100)
fh.goto(d, 500, 550, 100)
fh.goto(f, 300, 700, 100)
fh.goto(b, 100, 550, 100)
fh.goto(a, 100, 400, 100)
fh.sleep(4)
# 外扩完成，逆时针转动6完成
fh.goto(a, 300, 250, 100)
fh.goto(c, 500, 400, 100)
fh.goto(e, 500, 550, 100)
fh.goto(d, 300, 700, 100)
fh.goto(f, 100, 550, 100)
fh.goto(b, 100, 400, 100)
fh.sleep(6)
# 一排
fh.rainbow(a, 100, 0, 0)
fh.rainbow(b, 100, 0, 0)
fh.rainbow(c, 100, 0, 0)
fh.rainbow(d, 100, 0, 0)
fh.rainbow(e, 100, 0, 0)
fh.rainbow(f, 100, 0, 0)
fh.sleep(1)
fh.goto(d, 300, 700, 80)
fh.goto(f, 300, 600, 80)
fh.goto(e, 300, 500, 80)

fh.goto(b, 300, 400, 80)
fh.goto(c, 300, 300, 80)
fh.goto(a, 300, 200, 80)
fh.sleep(3)
# 开始波浪
# 1
fh.goto(d, 300, 700, 60)
fh.goto(f, 300, 600, 130)
fh.goto(e, 300, 500, 60)

fh.goto(b, 300, 400, 130)
fh.goto(c, 300, 300, 60)
fh.goto(a, 300, 200, 130)
fh.sleep(2)
# 2
fh.goto(d, 300, 700, 130)
fh.goto(f, 300, 600, 60)
fh.goto(e, 300, 500, 130)

fh.goto(b, 300, 400, 60)
fh.goto(c, 300, 300, 130)
fh.goto(a, 300, 200, 60)
fh.sleep(2)
# 最后
fh.goto(d, 300, 700, 60)
fh.goto(f, 300, 600, 130)
fh.goto(e, 300, 500, 60)

fh.goto(b, 300, 400, 130)
fh.goto(c, 300, 300, 60)
fh.goto(a, 300, 200, 130)
fh.sleep(3)
# 翻滚
fh.flip(f, 'f')
fh.flip(b, 'b')
fh.flip(a, 'f')
fh.sleep(3)
# 一排结束
fh.goto(d, 300, 700, 80)
fh.goto(f, 300, 600, 80)
fh.goto(e, 300, 500, 80)

fh.goto(b, 300, 400, 80)
fh.goto(c, 300, 300, 80)
fh.goto(a, 300, 200, 80)
fh.sleep(3)
# 33换位
fh.goto(d, 450, 700, 100)
fh.goto(f, 450, 600, 100)
fh.goto(e, 450, 500, 100)

fh.goto(b, 150, 400, 100)
fh.goto(c, 150, 300, 100)
fh.goto(a, 150, 200, 100)
fh.sleep(3)
# 水平上下移动
fh.goto(d, 450, 400, 100)
fh.goto(f, 450, 300, 100)
fh.goto(e, 450, 200, 100)

fh.goto(b, 150, 700, 100)
fh.goto(c, 150, 600, 100)
fh.goto(a, 150, 500, 100)
fh.sleep(3)
# 水平上下移动
fh.goto(b, 450, 700, 100)
fh.goto(c, 450, 600, 100)
fh.goto(a, 450, 500, 100)

fh.goto(d, 150, 400, 100)
fh.goto(f, 150, 300, 100)
fh.goto(e, 150, 200, 100)
fh.sleep(3)
# 换位完成，成型
fh.goto(b, 400, 650, 100)
fh.goto(c, 400, 500, 100)
fh.goto(a, 400, 350, 100)

fh.goto(d, 200, 650, 100)
fh.goto(f, 200, 500, 100)
fh.goto(e, 200, 350, 100)
fh.sleep(3)

fh.led(a, 255, 0, 0)
fh.led(b, 255, 0, 0)
fh.led(c, 255, 0, 0)
fh.led(d, 255, 0, 0)
fh.led(e, 255, 0, 0)
fh.led(f, 255, 0, 0)
fh.sleep(1)

# 五角星
fh.goto(b, 300, 700, 100)
fh.goto(c, 450, 500, 100)
fh.goto(a, 400, 300, 100)
fh.goto(f, 300, 500, 100)
fh.goto(d, 150, 500, 100)
fh.goto(e, 200, 300, 100)
fh.sleep(3)

fh.goto(b, 300, 700, 60)
fh.goto(c, 450, 500, 60)
fh.goto(a, 400, 300, 60)
fh.goto(f, 300, 500, 60)
fh.goto(d, 150, 500, 60)
fh.goto(e, 200, 300, 60)
fh.sleep(5)

fh.land(a)
fh.land(b)
fh.land(c)
fh.land(d)
fh.land(e)
fh.land(f)
fh.sleep(5)


fh.destroy()
