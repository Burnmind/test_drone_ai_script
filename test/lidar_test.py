import board
import busio

import adafruit_vl53l0x


i2c = busio.I2C(board.SCL, board.SDA)
vl53 = adafruit_vl53l0x.VL53L0X(i2c)
#vl53.measurement_timing_budget = 200000

dist_filtered = 0

print("Range: {0}mm".format(vl53.range))


a = vl53.range
b = vl53.range
c = vl53.range
if a <= b and a <= c:
    middle = c
    if b <= c:
        middle = b
else:
    if b <= a and b <= c:
        middle = c
        if a <= c:
            middle = a
    else:
        middle = b
        if a <= b:
            middle = a

delta = abs(dist_filtered - middle)
if delta > 100:
    k = 0.7
else:
    k = 0.1

dist_filtered = round(middle * k + dist_filtered * (1 - k))

print("Range: {0}mm".format(dist_filtered))

