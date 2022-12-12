### AirplaneManager

---

参见 FH0A.AirplaneManagerAdapter


```python
from time import sleep

from FH0A import get_airplane_manager
from FH0A.AirplaneManagerAdapter import AirplaneManager
from FH0A.AirplaneManagerAdapter import AirplaneController

m: AirplaneManager = get_airplane_manager()

# print('airplanes_table', m.airplanes_table)

m.flush()

# print('airplanes_table', m.airplanes_table)

print(m.start())

a = m.get_airplane_extended('COM3')

a.takeoff(100)
sleep(5)

a.left(50)
sleep(5)
a.right(50)
sleep(5)

a.forward(100)
sleep(5)
a.back(100)
sleep(5)

a.cw(100)
sleep(8)
a.ccw(100)
sleep(8)

a.land()
sleep(2)

a.shutdown()
exit(0)

```

### FH0A

---

参见 FH0A.ph0apy

```python
from FH0A.ph0apy import FH0A

fh = FH0A()

a = "COM3"
fh.add_uav(a)

fh.mode(a, 4)
fh.sleep(1)

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

fh.shutdown()
exit(0)



```




