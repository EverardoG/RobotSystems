#!/usr/bin/env python
# -*- coding: utf-8 -*-

import picarx_improved
import logging
import time
import numpy as np
import atexit
import line_following_interpreter as interp
import grayscale_module
import picarx_improved
from utils import reset_mcu
reset_mcu()

class Controller(object):
    """Class for controlling the robot in a line following task."""
    def __init__(self, proportional_gain=50,derivative_gain=5,line_polarity='darker',pwm_percent = 30):
        self.interpreter = interp.Interpreter(proportional_gain=proportional_gain,derivative_gain=derivative_gain,line_polarity=line_polarity)
        self.sensor = grayscale_module.Grayscale_Module(950)
        self.car = picarx_improved.Picarx()
        self.pwm_percent = pwm_percent
        self.dir_range = [-1, 1]
        self.steering_angle_range = [-90, 90]
        self.move_ave_num = 1
        self.dir_vals = [0]*self.move_ave_num
        atexit.register(self.shutdown)

    def follow_line(self):
        self._fill_buffer()
        while True:
            raw_data = self.sensor.get_grayscale_data()
            print(raw_data)
            direction = self.interpreter.get_direction(raw_data)
            goal_steering_angle = np.interp(direction,self.dir_range,self.steering_angle_range)
            self.dir_vals.append(goal_steering_angle)
            del self.dir_vals[0]
            goal_steering_angle = np.average(self.dir_vals)
            print(goal_steering_angle)
            self.car.set_dir_servo_angle(goal_steering_angle)
            self.car.forward(self.pwm_percent)

    def _fill_buffer(self):
        """Sensor needs to fill a buffer before starting to move."""
        while not self.interpreter.buffer_full:
            raw_data = self.sensor.get_grayscale_data()
            self.interpreter.get_direction(raw_data)

    def shutdown(self):
        self.car.stop()

def main():
    logging.getLogger().setLevel(logging.INFO)
    controller = Controller(proportional_gain=7,derivative_gain=5,line_polarity='darker',pwm_percent = 40)
    controller.follow_line()

if __name__ == '__main__':
    main()
