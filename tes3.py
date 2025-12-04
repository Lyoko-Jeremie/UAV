from time import sleep

from uav import get_airplane_manager, UAVAirplaneManager

if __name__ == '__main__':
    m: UAVAirplaneManager = get_airplane_manager()

    # print('airplanes_table', m.airplanes_table)

    m.flush()

    # print('airplanes_table', m.airplanes_table)

    print(m.start())

    a = m.get_airplane_extended('FH0C:COM3')

    # a.takeoff(100)
    # m.sleep(10)
    #
    # a.land()
    # sleep(5)

    sleep(20)

    a.shutdown()
    exit(0)
