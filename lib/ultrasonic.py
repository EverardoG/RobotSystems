#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import sys
sys.path.append(r'/home/pi/picar-x/lib')
from utils import reset_mcu
reset_mcu()

from picarx import Picarx
from ultrasonic import Ultrasonic
from pin import Pin


import picarx_improved
import time
import numpy as np
import atexit
import line_following_interpreter as interp
import grayscale_module
import picarx_improved
from utils import reset_mcu
reset_mcu()


class Ultrasonic():
    def __init__(self, timeout=0.02):
        self.trig = Pin("D2")
        self.echo = Pin("D3")
        self.timeout = timeout

    def _read(self):
        self.trig.low()
        time.sleep(0.01)
        self.trig.high()
        time.sleep(0.00001)
        self.trig.low()
        pulse_end = 0
        pulse_start = 0
        timeout_start = time.time()
        while self.echo.value()==0:
            pulse_start = time.time()
            if pulse_start - timeout_start > self.timeout:
                return -1
        while self.echo.value()==1:
            pulse_end = time.time()
            if pulse_end - timeout_start > self.timeout:
                return -1
        during = pulse_end - pulse_start
        cm = round(during * 340 / 2 * 100, 2)
        return cm

    def read(self, times=10):
        for i in range(times):
            a = self._read()
            if a != -1 or a <= 300:
                return a
        return -1

    def obstacle(self,dist):
        pass


def main():
    sonar = Ultrasonic()
    while True:
        print(sonar.read())

if __name__ == '__main__':
    main()