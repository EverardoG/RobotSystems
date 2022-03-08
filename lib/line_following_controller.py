#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Module with a controller class for the picar-x to be controlled to follow a line. """

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

class Controller(picarx_improved.Picarx):
    """Class for controlling the robot in a line following task."""
    def __init__(self, car, pwm_percent = 30):
        super().__init__()
        self.car = car
        self.pwm_percent = pwm_percent
        self.dir_range = [-1, 1]
        self.steering_angle_range = [-90, 90]  # Range is larger than actual range of motion to allow for more responsiveness.
        self.move_ave_num = 1
        self.dir_vals = [0]*self.move_ave_num  # Queue used to smooth out direction commands.
        atexit.register(self.shutdown)

    def _get_steering_angle(self,direction):
        """Takes the "direction" from the interpreter and returns a steering angle for the car."""
        steering_angle = np.interp(direction, self.dir_range, self.steering_angle_range)
        return steering_angle

    def follow_line(self,direction):
        """Follow a line for one sample reading. """
        goal_steering_angle_raw = self._get_steering_angle(direction)
        # add steering angle to a FIFO queue, and take average to smooth out commands.
        self.dir_vals.append(goal_steering_angle_raw)
        del self.dir_vals[0]
        goal_steering_angle = np.average(self.dir_vals)
        self.car.set_dir_servo_angle(goal_steering_angle)
        self.car.forward(self.pwm_percent)
        return goal_steering_angle

    def follow_line_with_ultrasonic(self,direction,obstacle):
        """Follow a line for one sample reading. """
        goal_steering_angle_raw = self._get_steering_angle(direction)
        # add steering angle to a FIFO queue, and take average to smooth out commands.
        self.dir_vals.append(goal_steering_angle_raw)
        del self.dir_vals[0]
        goal_steering_angle = np.average(self.dir_vals)
        self.car.set_dir_servo_angle(goal_steering_angle)
        if obstacle:
            self.car.forward(0)
        else:
            self.car.forward(self.pwm_percent)
        return goal_steering_angle

    def fill_buffer(self,interpreter,sensor):
        """Sensor needs to fill a buffer before starting to move."""
        while not interpreter.buffer_full:
            raw_data = sensor.get_grayscale_data()
            interpreter.get_direction(raw_data)

    def __call__(self):
        return

    def shutdown(self):
        self.car.stop()

def main():
    logging.getLogger().setLevel(logging.DEBUG)

    interpreter = interp.Interpreter(proportional_gain=10,derivative_gain=1,line_polarity='darker')
    sensor = grayscale_module.Grayscale_Module(950)
    car = picarx_improved.Picarx()
    controller = Controller(car,pwm_percent = 30)
    controller.fill_buffer(interpreter, sensor)
    while True:
        raw_data = sensor.get_grayscale_data()
        direction = interpreter.get_direction(raw_data)
        goal_steering_angle = controller.follow_line(direction)

if __name__ == '__main__':
    main()
