from src.FH0A.ph0apy import FH0A

# from FH0A.ph0apy import FH0A

# 共轴转动；执行此段代码前，请确认无人机的初始位置，避免碰撞
def co_axial_rotation():
    fpy = (350, 300, 250, 300)
    fpz = (100, 150, 100, 50)

    a = 'COM3'
    b = 'COM6'
    c = 'COM7'

    fh = FH0A()
    fh.add_uav(a)
    fh.add_uav(b)
    fh.add_uav(c)

    # 设置模式
    fh.mode(a, 4)
    fh.mode(b, 4)
    fh.mode(c, 4)
    fh.sleep(5)

    # 无人机起飞
    fh.takeoff(a, 100)
    fh.takeoff(b, 100)
    fh.takeoff(c, 100)
    fh.sleep(5)

    # 开始执行共轴转动编队动作
    # 前往动作初始位置
    fh.goto(a, 200, 300, 50)
    fh.goto(b, 300, 300, 50)
    fh.goto(c, 400, 300, 50)
    fh.sleep(5)

    for i in range(2):
        for j in range(4):
            fh.goto(a, 200, fpy[j], fpz[j])
            fh.goto(b, 300, fpy[j], fpz[j])
            fh.goto(c, 400, fpy[j], fpz[j])
            fh.sleep(5)

    # 无人机降落
    fh.land(a)
    fh.land(b)
    fh.land(c)
    fh.sleep(5)
    fh.destroy()


# V字移动；执行此段代码前，请确认无人机的初始位置，避免碰撞
def v_move():
    a = 'COM3'
    b = 'COM6'
    c = 'COM7'

    fh = FH0A()
    fh.add_uav(a)
    fh.add_uav(b)
    fh.add_uav(c)

    # 设置模式
    fh.mode(a, 4)
    fh.mode(b, 4)
    fh.mode(c, 4)
    fh.sleep(5)

    # 无人机起飞
    fh.takeoff(a, 100)
    fh.takeoff(b, 100)
    fh.takeoff(c, 100)
    fh.sleep(5)

    # 开始执行v字移动编队动作
    # 前往动作初始位置
    fh.goto(a, 250, 300, 180)
    fh.goto(b, 300, 300, 100)
    fh.goto(c, 350, 300, 180)
    fh.sleep(5)

    fh.goto(c, 350, 250, 180)
    fh.goto(a, 350, 300, 180)
    fh.goto(b, 250, 300, 180)
    fh.goto(c, 300, 300, 100)
    fh.sleep(5)

    fh.goto(a, 300, 350, 180)
    fh.goto(b, 300, 250, 180)
    fh.sleep(5)

    fh.goto(a, 250, 300, 180)
    fh.goto(b, 350, 300, 180)
    fh.sleep(5)

    fh.goto(b, 350, 250, 180)
    fh.goto(a, 350, 300, 180)
    fh.goto(c, 250, 300, 180)
    fh.goto(b, 300, 300, 100)
    fh.sleep(5)

    fh.goto(a, 300, 350, 180)
    fh.goto(c, 300, 250, 180)
    fh.sleep(5)

    fh.goto(a, 250, 300, 180)
    fh.goto(c, 350, 300, 180)
    fh.sleep(5)

    fh.goto(c, 350, 250, 180)
    fh.goto(a, 350, 300, 180)
    fh.goto(b, 250, 300, 180)
    fh.goto(c, 300, 300, 100)
    fh.sleep(5)

    fh.goto(a, 300, 350, 180)
    fh.goto(b, 300, 250, 180)
    fh.sleep(5)

    fh.goto(a, 250, 300, 180)
    fh.goto(b, 350, 300, 180)
    fh.sleep(5)

    # 无人机降落
    fh.land(a)
    fh.land(b)
    fh.land(c)
    fh.sleep(5)


# 数字100展示；执行此段代码前，请确认无人机的初始位置，避免碰撞
def num_100():
    a = 'COM3'
    b = 'COM4'
    c = 'COM5'

    fh = FH0A()
    fh.add_uav(a)
    fh.add_uav(b)
    fh.add_uav(c)

    # 设置模式
    fh.mode(a, 4)
    fh.mode(b, 4)
    fh.mode(c, 4)
    fh.sleep(5)

    # 无人机起飞
    fh.takeoff(a, 100)
    fh.takeoff(b, 100)
    fh.takeoff(c, 100)
    fh.sleep(5)

    # 开始执行数字100展示编队动作
    # 前往动作初始位置
    fh.goto(a, 100, 250, 150)
    fh.goto(b, 250, 250, 150)
    fh.goto(c, 450, 250, 150)
    fh.sleep(5)

    for i in range(2):
        fh.goto(a, 100, 350, 150)
        fh.goto(b, 300, 300, 150)
        fh.goto(c, 400, 300, 150)
        fh.sleep(5)

        fh.goto(a, 100, 250, 150)
        fh.goto(b, 250, 350, 150)
        fh.goto(c, 450, 350, 150)
        fh.sleep(5)

        fh.goto(a, 100, 350, 150)
        fh.goto(b, 200, 300, 150)
        fh.goto(c, 500, 300, 150)
        fh.sleep(5)

        fh.goto(a, 100, 250, 150)
        fh.goto(b, 250, 250, 150)
        fh.goto(c, 450, 250, 150)
        fh.sleep(5)

    # 无人机降落
    fh.land(a)
    fh.land(b)
    fh.land(c)
    fh.sleep(5)


if __name__ == '__main__':
    # 共轴转动
    # co_axial_rotation()
    #
    # v字移动
    v_move()
    #
    # 数字100展示
    # num_100()
    exit()
    # ============================================================================
