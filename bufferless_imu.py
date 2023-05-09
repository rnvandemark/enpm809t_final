from numpy import pi, deg2rad
from serial import Serial
from time import sleep

def fix_orientation(degs):
    return -deg2rad(degs) + (2 * pi)

class BufferlessImu(object):
    def __init__(self, imu_port, baud_rate):
        self.ser = Serial(imu_port, baud_rate)
        self.count = 0
        self.last_sample = None

    def read(self):
        while (self.last_sample is None) or (self.ser.in_waiting > 0):
            if (self.ser.in_waiting <= 0) and (self.last_sample is None):
                sleep(0.05)
            else:
                try:
                    line_original = str(self.ser.readline())
                    if self.count > 10:
                        self.last_sample = fix_orientation(float(''.join(c for c in line_original if c in "0123456789.")))
                    self.count += 1
                except:
                    pass
        return self.last_sample
