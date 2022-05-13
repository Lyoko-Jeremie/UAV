from src.FH0A.ph0apy import FH0A

fh = FH0A()

a = "COM3"
b = "COM4"
c = "COM5"
d = "COM6"
e = "COM7"
f = "COM8"

# 添加飞机
fh.add_uav(a)
fh.add_uav(b)
fh.add_uav(c)
fh.add_uav(d)
fh.add_uav(e)
fh.add_uav(f)
fh.sleep(1)

# 设置模式
fh.mode(a,4)
fh.mode(b,4)
fh.mode(c,4)
fh.mode(d,4)
fh.mode(e,4)
fh.mode(f,4)
fh.sleep(1)

# 设置速度
fh.speed(a,70)
fh.speed(b,70)
fh.speed(c,70)
fh.speed(d,70)
fh.speed(e,70)
fh.speed(f,70)
fh.sleep(1)

# 起飞
fh.takeoff(a,100)
fh.takeoff(b,100)
fh.takeoff(c,100)
fh.takeoff(d,100)
fh.takeoff(e,100)
fh.takeoff(f,100)
fh.sleep(3)

# 指定动作
# 绿灯开始
fh.led(a,0,255,0)
fh.led(b,0,255,0)
fh.led(c,0,255,0)
fh.led(d,0,255,0)
fh.led(e,0,255,0)
fh.led(f,0,255,0)
fh.sleep(1)

# 调整高度
fh.goto(a,50,50,200)
fh.goto(b,150,50,80)
fh.goto(c,250,50,80)
fh.goto(d,350,50,200)
fh.goto(e,450,50,80)
fh.goto(f,550,50,200)
fh.sleep(3)

fh.goto(a,150,450,200)
fh.goto(b,150,150,80)
fh.goto(c,300,450,80)
fh.goto(d,300,150,200)
fh.goto(e,450,150,80)
fh.goto(f,450,450,200)
fh.sleep(7)

fh.goto(d,450,150,80)
fh.goto(b,300,150,200)
fh.goto(e,150,150,80)
fh.goto(a,300,450,80)
fh.goto(f,150,450,200)
fh.goto(c,450,450,200)
fh.sleep(6)


fh.goto(f,450,450,200)
fh.goto(c,300,150,200)
fh.goto(b,150,450,200)
fh.goto(a,150,150,80)
fh.goto(e,450,150,80)
fh.goto(d,300,450,80)
fh.sleep(6)

fh.goto(c,450,150,80)
fh.goto(a,300,150,200)
fh.goto(e,150,150,80)
fh.goto(b,300,450,80)
fh.goto(f,150,450,200)
fh.goto(d,450,450,200)
fh.sleep(6)


fh.goto(f,450,450,200)
fh.goto(d,300,150,200)
fh.goto(a,150,450,200)
fh.goto(b,150,150,80)
fh.goto(e,450,150,80)
fh.goto(c,300,450,80)
fh.sleep(6)
# 正方形

fh.goto(a,450,450,200)
fh.goto(f,450,150,80)
fh.goto(e,150,150,80)
fh.goto(b,150,450,200)
fh.sleep(1)
fh.goto(c,150,300,140)
fh.goto(d,450,300,140)
fh.sleep(6)


fh.goto(c,300,150,200)
fh.goto(d,300,450,80)
fh.sleep(1)
fh.goto(b,450,450,200)
fh.goto(a,450,150,80)
fh.goto(f,150,150,80)
fh.goto(e,150,450,200)
fh.sleep(6)



fh.goto(e,450,450,200)
fh.goto(b,450,150,80)
fh.goto(a,150,150,80)
fh.goto(f,150,450,200)
fh.sleep(1)
fh.goto(c,450,300,140)
fh.goto(d,150,300,140)
fh.sleep(6)


fh.goto(c,300,450,80)
fh.goto(d,300,150,200)
fh.sleep(1)
fh.goto(f,450,450,200)
fh.goto(e,450,150,80)
fh.goto(b,150,150,80)
fh.goto(a,150,450,200)
fh.sleep(6)

# 红灯结束
fh.led(a,255,0,0)
fh.led(b,255,0,0)
fh.led(c,255,0,0)
fh.led(d,255,0,0)
fh.led(e,255,0,0)
fh.led(f,255,0,0)
fh.sleep(1)
#########################################################
# DIY动作
# 绿灯
fh.led(a,0,255,0)
fh.led(b,0,255,0)
fh.led(c,0,255,0)
fh.led(d,0,255,0)
fh.led(e,0,255,0)
fh.led(f,0,255,0)
fh.sleep(1)



# 三角形定位
fh.goto(a,100,300,80)
fh.goto(b,200,300,140)
fh.goto(d,300,300,200)
fh.goto(c,300,300,80)
fh.goto(e,400,300,140)
fh.goto(f,500,300,80)
fh.sleep(5)
# 蓝灯
fh.led(a,0,0,255)
fh.led(b,0,0,255)
fh.led(c,0,0,255)
fh.led(d,0,0,255)
fh.led(e,0,0,255)
fh.led(f,0,0,255)
fh.sleep(1)



#定位1
fh.goto(a,100,500,80)
fh.goto(b,200,400,140)
fh.goto(e,400,200,140)
fh.goto(f,500,100,80)
fh.sleep(5)
# 黄灯
fh.led(a,255,255,0)
fh.led(b,255,255,0)
fh.led(c,255,255,0)
fh.led(d,255,255,0)
fh.led(e,255,255,0)
fh.led(f,255,255,0)
fh.sleep(1)



#定位3
fh.goto(a,500,500,80)
fh.goto(b,400,400,140)
fh.goto(e,200,200,140)
fh.goto(f,100,100,80)
fh.sleep(5)
# 紫灯
fh.led(a,255,0,255)
fh.led(b,255,0,255)
fh.led(c,255,0,255)
fh.led(d,255,0,255)
fh.led(e,255,0,255)
fh.led(f,255,0,255)
fh.sleep(1)


#定位4回到原点
fh.goto(a,500,300,80)
fh.goto(b,400,300,140)
fh.goto(e,200,300,140)
fh.goto(f,100,300,80)
fh.sleep(5)
#绿灯
fh.led(a,0,255,0)
fh.led(b,0,255,0)
fh.led(c,0,255,0)
fh.led(d,0,255,0)
fh.led(e,0,255,0)
fh.led(f,0,255,0)
fh.sleep(1)

###############################
#六边形定点
fh.goto(f,100,400,160)
fh.goto(e,100,200,120)
fh.goto(c,300,100,80)
fh.goto(a,500,200,120)
fh.goto(b,500,400,160)
fh.goto(d,300,500,200)
fh.sleep(8)
# 蓝灯
fh.led(a,0,0,255)
fh.led(b,0,0,255)
fh.led(c,0,0,255)
fh.led(d,0,0,255)
fh.led(e,0,0,255)
fh.led(f,0,0,255)
fh.sleep(1)


#定位1
fh.goto(f,100,300,140)
fh.goto(e,200,150,100)
fh.goto(c,400,150,100)
fh.goto(a,500,300,140)
fh.goto(b,400,450,180)
fh.goto(d,200,450,180)
fh.sleep(3)
#黄灯
fh.led(a,255,255,0)
fh.led(b,255,255,0)
fh.led(c,255,255,0)
fh.led(d,255,255,0)
fh.led(e,255,255,0)
fh.led(f,255,255,0)
fh.sleep(1)


#定位2
fh.goto(d,100,400,160)
fh.goto(f,100,200,120)
fh.goto(e,300,100,80)
fh.goto(c,500,200,120)
fh.goto(a,500,400,160)
fh.goto(b,300,500,200)
fh.sleep(3)
#紫灯
fh.led(a,255,0,255)
fh.led(b,255,0,255)
fh.led(c,255,0,255)
fh.led(d,255,0,255)
fh.led(e,255,0,255)
fh.led(f,255,0,255)
fh.sleep(1)


#定位3
fh.goto(d,100,300,140)
fh.goto(f,200,150,100)
fh.goto(e,400,150,100)
fh.goto(c,500,300,140)
fh.goto(a,400,450,180)
fh.goto(b,200,450,180)
fh.sleep(3)
# 青灯
fh.led(a,0,255,255)
fh.led(b,0,255,255)
fh.led(c,0,255,255)
fh.led(d,0,255,255)
fh.led(e,0,255,255)
fh.led(f,0,255,255)
fh.sleep(1)


#定位4
fh.goto(b,100,400,160)
fh.goto(d,100,200,120)
fh.goto(f,300,100,80)
fh.goto(e,500,200,120)
fh.goto(c,500,400,160)
fh.goto(a,300,500,200)
fh.sleep(3)
#绿灯
fh.led(a,0,255,0)
fh.led(b,0,255,0)
fh.led(c,0,255,0)
fh.led(d,0,255,0)
fh.led(e,0,255,0)
fh.led(f,0,255,0)
fh.sleep(1)


#定位5
fh.goto(b,100,300,140)
fh.goto(d,200,150,100)
fh.goto(f,400,150,100)
fh.goto(e,500,300,140)
fh.goto(c,400,450,180)
fh.goto(a,200,450,180)
fh.sleep(3)
# 蓝灯
fh.led(a,0,0,255)
fh.led(b,0,0,255)
fh.led(c,0,0,255)
fh.led(d,0,0,255)
fh.led(e,0,0,255)
fh.led(f,0,0,255)
fh.sleep(1)


#定位6
fh.goto(a,100,400,160)
fh.goto(b,100,200,120)
fh.goto(d,300,100,80)
fh.goto(f,500,200,120)
fh.goto(e,500,400,160)
fh.goto(c,300,500,200)
fh.sleep(3)
#红灯结束
fh.led(a,255,0,0)
fh.led(b,255,0,0)
fh.led(c,255,0,0)
fh.led(d,255,0,0)
fh.led(e,255,0,0)
fh.led(f,255,0,0)
fh.sleep(1)



# 返回原点
fh.goto(a,50,50,160)
fh.goto(b,150,50,120)
fh.goto(c,250,50,200)
fh.goto(d,350,50,80)
fh.goto(e,450,50,160)
fh.goto(f,550,50,120)
fh.sleep(8)
# 调高
fh.goto(a,50,50,50)
fh.goto(b,150,50,50)
fh.goto(c,250,50,50)
fh.goto(d,350,50,50)
fh.goto(e,450,50,50)
fh.goto(f,550,50,50)
fh.sleep(3)
# 降落
fh.land(a)
fh.land(b)
fh.land(c)
fh.land(d)
fh.land(e)
fh.land(f)
fh.sleep(3)

fh.destroy()
