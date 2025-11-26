from uav import UAV

#================================================================

u = UAV()

#================================================================

a = 'FH0C:COM3'
b = 'FH0C:COM4'
c = 'FH0C:COM5'

u.sleep(2)

u.takeoff(a, 100)
u.takeoff(b, 100)
u.takeoff(c, 100)
print('takeoff')
u.sleep(5)

u.forward(a, 100)
u.forward(b, 100)
u.forward(c, 100)
print('forward')
u.sleep(5)
u.back(a, 100)
u.back(b, 100)
u.back(c, 100)
print('back')
u.sleep(5)

u.cw(a, 100)
u.cw(b, 100)
u.cw(c, 100)
print('cw')
u.sleep(8)
u.ccw(a, 100)
u.ccw(b, 100)
u.ccw(c, 100)
print('ccw')
u.sleep(8)

u.land(a)
u.land(b)
u.land(c)
print('land')
u.sleep(2)



#============================================================================
u.destroy()

exit()
