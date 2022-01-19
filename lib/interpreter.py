#!/usr/bin/env python
# -*- coding: utf-8 -*-

import picarx_improved
import logging
import time


class Interpreter(object):
    """Class for interpreting the grayscale sensor data into a discrete position."""
    def __init__(self, sensitivity = 50, line_polarity='darker'):
        self.sensitivity = sensitivity
        self.line_polarity = line_polarity  # 'darker' if line is darker than surroundings. 'lighter' if opposite.
        self.previous_sensor_data = None

    def get_position(self,sensor_data):
        """Interprets the sensor data and returns the discrete position of the robot. sensor_data argument is a list
        of length 3. Returns -1, 0, or 1 for if the robot is to the left, center, or right of the line."""



        self.previous_sensor_data = sensor_data
        pass

def main():
    logging.getLogger().setLevel(logging.INFO)
    pass

if __name__ == '__main__':
    main()
