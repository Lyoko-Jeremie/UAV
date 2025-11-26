from UAV.FH0A.ph0apy import FH0A

fh = FH0A()
a = 'COM3'

fh.takeoff(a, 100)
fh.sleep(5)

fh.left(a, 50)
fh.sleep(5)
fh.right(a, 50)
fh.sleep(5)

fh.forward(a, 100)
fh.sleep(5)
fh.back(a, 100)
fh.sleep(5)

fh.cw(a, 100)
fh.sleep(8)
fh.ccw(a, 100)
fh.sleep(8)

fh.land(a)
fh.sleep(2)

#
fh.destroy()
