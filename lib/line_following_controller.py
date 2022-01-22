#!/usr/bin/env python
# -*- coding: utf-8 -*-

import picarx_improved
import time
import numpy as np
import atexit
import line_following_interpreter as interp
import grayscale_module
import picarx_improved
from utils import reset_mcu
reset_mcu()
import logging
logging.basicConfig(format="%(asctime)s:%(message)s", level=logging.INFO, datefmt="%H:%M:%S")
logging.getLogger().setLevel(logging.DEBUG)
from logdecorator import log_on_start,log_on_error,log_on_end

class Controller(picarx_improved.Picarx):
    """Class for controlling the robot in a line following task."""
    def __init__(self, proportional_gain=50,derivative_gain=5,line_polarity='darker',pwm_percent = 30):
        super().__init__()
        self.interpreter = interp.Interpreter(proportional_gain=proportional_gain,derivative_gain=derivative_gain,line_polarity=line_polarity)
        self.sensor = grayscale_module.Grayscale_Module(950)
        self.car = picarx_improved.Picarx()
        self.pwm_percent = pwm_percent
        self.dir_range = [-1, 1]
        self.steering_angle_range = [-90, 90]
        self.move_ave_num = 1
        self.dir_vals = [0]*self.move_ave_num
        atexit.register(self.shutdown)

    def _get_steering_angle(self,direction):
        """Takes the "direction" from the interpreter and returns a steering angle for the car."""
        steering_angle = np.interp(direction, self.dir_range, self.steering_angle_range)
        return steering_angle

    def follow_line(self):
        self._fill_buffer()
        while True:
            raw_data = self.sensor.get_grayscale_data()
            direction = self.interpreter.get_direction(raw_data)
            goal_steering_angle = self._get_steering_angle(direction)
            # add steering angle to a FIFO queue, and take average to smooth out commands.
            self.dir_vals.append(goal_steering_angle)
            del self.dir_vals[0]
            goal_steering_angle = np.average(self.dir_vals)
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
    logging.getLogger().setLevel(logging.debug)
    car = Controller(proportional_gain=10,derivative_gain=1,line_polarity='darker',pwm_percent = 30)
    car.follow_line()

if __name__ == '__main__':
    main()
