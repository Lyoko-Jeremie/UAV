from uav import UAV

#================================================================

u = UAV()

#================================================================

a = 'FH0C:COM3'

u.sleep(2)

u.takeoff(a, 100)
print('takeoff')
u.sleep(5)

# u.forward(a, 100)
# print('forward')
# u.sleep(5)
# u.back(a, 100)
# print('back')
# u.sleep(5)

u.cw(a, 100)
print('cw')
u.sleep(8)
u.ccw(a, 100)
print('ccw')
u.sleep(8)

u.land(a)
print('land')
u.sleep(7)



#============================================================================
u.destroy()

exit()
