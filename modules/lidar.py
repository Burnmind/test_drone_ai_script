import board
import busio
import adafruit_vl53l0x

vl53 = None
dist_filtered = 0

def connect_lidar():
    global vl53
    i2c = busio.I2C(board.SCL, board.SDA)
    vl53 = adafruit_vl53l0x.VL53L0X(i2c)
    vl53.signal_rate_limit = 0.1
    vl53._write_u8(adafruit_vl53l0x._PRE_RANGE_CONFIG_VCSEL_PERIOD, 18)
    vl53._write_u8(adafruit_vl53l0x._FINAL_RANGE_CONFIG_VCSEL_PERIOD, 14)
    # vl53.measurement_timing_budget = 200000

def read_lidar_distance():
    global vl53
    global dist_filtered
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
    if dist_filtered > 2000:
        dist_filtered = 2000

    return dist_filtered / 1000
