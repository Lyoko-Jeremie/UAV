from time import sleep

from UAV.FH0A import get_airplane_manager
from UAV.FH0A.AirplaneManagerAdapter import AirplaneManager
from UAV.FH0A.AirplaneManagerAdapter import AirplaneController

if __name__ == '__main__':
    m: AirplaneManager = get_airplane_manager()

    # print('airplanes_table', m.airplanes_table)

    m.flush()

    # print('airplanes_table', m.airplanes_table)

    print(m.start())

    a = m.get_airplane_extended('COM3')
    b = m.get_airplane_extended('COM4')
    c = m.get_airplane_extended('COM5')

    for i in [a, b, c]:
        i.takeoff(100)
    print('takeoff')
    sleep(5)

    for i in [a, b, c]:
        i.left(50)
    print('left')
    sleep(5)
    for i in [a, b, c]:
        i.right(50)
    print('right')
    sleep(5)

    for i in [a, b, c]:
        i.forward(100)
    print('forward')
    sleep(5)
    for i in [a, b, c]:
        i.back(100)
    print('back')
    sleep(5)

    for i in [a, b, c]:
        i.cw(360)
    print('cw')
    sleep(8)
    for i in [a, b, c]:
        i.ccw(360)
    print('ccw')
    sleep(8)

    for i in [a, b, c]:
        i.land()
    print('land')
    sleep(2)

    for i in [a, b, c]:
        i.shutdown()
    print('shutdown')
    sleep(2)
    exit(0)
