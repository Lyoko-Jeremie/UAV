from time import sleep

from uav import get_airplane_manager, UAVAirplaneManager

if __name__ == '__main__':
    m: UAVAirplaneManager = get_airplane_manager()

    # print('airplanes_table', m.airplanes_table)

    m.flush()

    # print('airplanes_table', m.airplanes_table)

    print(m.start())

    a = m.get_airplane_extended('FH0C:COM3')

    a.takeoff(100)
    m.sleep(5)

    a.mode(4)

    # a.left(50)
    # sleep(5)
    # a.right(50)
    # sleep(5)
    #
    # a.forward(100)
    # sleep(5)
    # a.back(100)
    # sleep(5)

    a.cw(100)
    sleep(8)
    a.ccw(100)
    sleep(8)

    a.land()
    sleep(2)

    a.shutdown()
    exit(0)
