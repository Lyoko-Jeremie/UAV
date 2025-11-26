from uav import UAV

# 共轴转动；执行此段代码前，请确认无人机的初始位置，避免碰撞
def co_axial_rotation():
    fpy = (350, 300, 250, 300)
    fpz = (100, 150, 100, 50)

    a = 'FH0C:COM3'
    b = 'FH0C:COM6'
    c = 'FH0C:COM7'

    u = UAV()
    u.add_uav(a)
    u.add_uav(b)
    u.add_uav(c)

    # 设置模式
    u.mode(a, 4)
    u.mode(b, 4)
    u.mode(c, 4)
    u.sleep(5)

    # 无人机起飞
    u.takeoff(a, 100)
    u.takeoff(b, 100)
    u.takeoff(c, 100)
    u.sleep(5)

    # 开始执行共轴转动编队动作
    # 前往动作初始位置
    u.goto(a, 200, 300, 50)
    u.goto(b, 300, 300, 50)
    u.goto(c, 400, 300, 50)
    u.sleep(5)

    for i in range(2):
        for j in range(4):
            u.goto(a, 200, fpy[j], fpz[j])
            u.goto(b, 300, fpy[j], fpz[j])
            u.goto(c, 400, fpy[j], fpz[j])
            u.sleep(5)

    # 无人机降落
    u.land(a)
    u.land(b)
    u.land(c)
    u.sleep(5)
    u.destroy()


# V字移动；执行此段代码前，请确认无人机的初始位置，避免碰撞
def v_move():
    a = 'FH0C:COM3'
    b = 'FH0C:COM6'
    c = 'FH0C:COM7'

    u = UAV()
    u.add_uav(a)
    u.add_uav(b)
    u.add_uav(c)

    # 设置模式
    u.mode(a, 4)
    u.mode(b, 4)
    u.mode(c, 4)
    u.sleep(5)

    # 无人机起飞
    u.takeoff(a, 100)
    u.takeoff(b, 100)
    u.takeoff(c, 100)
    u.sleep(5)

    # 开始执行v字移动编队动作
    # 前往动作初始位置
    u.goto(a, 250, 300, 180)
    u.goto(b, 300, 300, 100)
    u.goto(c, 350, 300, 180)
    u.sleep(5)

    u.goto(c, 350, 250, 180)
    u.goto(a, 350, 300, 180)
    u.goto(b, 250, 300, 180)
    u.goto(c, 300, 300, 100)
    u.sleep(5)

    u.goto(a, 300, 350, 180)
    u.goto(b, 300, 250, 180)
    u.sleep(5)

    u.goto(a, 250, 300, 180)
    u.goto(b, 350, 300, 180)
    u.sleep(5)

    u.goto(b, 350, 250, 180)
    u.goto(a, 350, 300, 180)
    u.goto(c, 250, 300, 180)
    u.goto(b, 300, 300, 100)
    u.sleep(5)

    u.goto(a, 300, 350, 180)
    u.goto(c, 300, 250, 180)
    u.sleep(5)

    u.goto(a, 250, 300, 180)
    u.goto(c, 350, 300, 180)
    u.sleep(5)

    u.goto(c, 350, 250, 180)
    u.goto(a, 350, 300, 180)
    u.goto(b, 250, 300, 180)
    u.goto(c, 300, 300, 100)
    u.sleep(5)

    u.goto(a, 300, 350, 180)
    u.goto(b, 300, 250, 180)
    u.sleep(5)

    u.goto(a, 250, 300, 180)
    u.goto(b, 350, 300, 180)
    u.sleep(5)

    # 无人机降落
    u.land(a)
    u.land(b)
    u.land(c)
    u.sleep(5)


# 数字100展示；执行此段代码前，请确认无人机的初始位置，避免碰撞
def num_100():
    a = 'FH0C:COM3'
    b = 'FH0C:COM4'
    c = 'FH0C:COM5'

    u = UAV()
    u.add_uav(a)
    u.add_uav(b)
    u.add_uav(c)

    # 设置模式
    u.mode(a, 4)
    u.mode(b, 4)
    u.mode(c, 4)
    u.sleep(5)

    # 无人机起飞
    u.takeoff(a, 100)
    u.takeoff(b, 100)
    u.takeoff(c, 100)
    u.sleep(5)

    # 开始执行数字100展示编队动作
    # 前往动作初始位置
    u.goto(a, 100, 250, 150)
    u.goto(b, 250, 250, 150)
    u.goto(c, 450, 250, 150)
    u.sleep(5)

    for i in range(2):
        u.goto(a, 100, 350, 150)
        u.goto(b, 300, 300, 150)
        u.goto(c, 400, 300, 150)
        u.sleep(5)

        u.goto(a, 100, 250, 150)
        u.goto(b, 250, 350, 150)
        u.goto(c, 450, 350, 150)
        u.sleep(5)

        u.goto(a, 100, 350, 150)
        u.goto(b, 200, 300, 150)
        u.goto(c, 500, 300, 150)
        u.sleep(5)

        u.goto(a, 100, 250, 150)
        u.goto(b, 250, 250, 150)
        u.goto(c, 450, 250, 150)
        u.sleep(5)

    # 无人机降落
    u.land(a)
    u.land(b)
    u.land(c)
    u.sleep(5)


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
