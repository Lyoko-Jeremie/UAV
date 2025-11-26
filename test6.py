from uav.FH0A.ph0apy import FH0A

#================================================================

fh = FH0A()

#================================================================

a = 'COM3'
b = 'COM4'
c = 'COM5'

fh.sleep(2)

fh.takeoff(a, 100)
fh.takeoff(b, 100)
fh.takeoff(c, 100)
print('takeoff')
fh.sleep(5)

fh.forward(a, 100)
fh.forward(b, 100)
fh.forward(c, 100)
print('forward')
fh.sleep(5)
fh.back(a, 100)
fh.back(b, 100)
fh.back(c, 100)
print('back')
fh.sleep(5)

fh.cw(a, 100)
fh.cw(b, 100)
fh.cw(c, 100)
print('cw')
fh.sleep(8)
fh.ccw(a, 100)
fh.ccw(b, 100)
fh.ccw(c, 100)
print('ccw')
fh.sleep(8)

fh.land(a)
fh.land(b)
fh.land(c)
print('land')
fh.sleep(2)



#============================================================================
fh.destroy()

exit()
