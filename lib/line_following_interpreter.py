#!/usr/bin/env python
# -*- coding: utf-8 -*-

import picarx_improved
import logging
import time
import numpy as np

class Interpreter(object):
    """Class for interpreting the grayscale sensor data into a discrete position."""

    def __init__(self, proportional_gain=50,derivative_gain=5, line_polarity='darker'):
        polarity_map = {'darker':-1,'lighter':1}
        self.line_polarity = polarity_map[line_polarity]
        self.p_gain = proportional_gain / 50000
        self.d_gain = derivative_gain
        self.running_data = [ [], [], [] ]
        self.running_aves = [0,0,0]
        self.changes = [0,0,0]
        self.moving_ave_num = 10
        self.buffer_full = False

    def reset(self):
        self.running_data = [ [], [], [] ]
        self.running_aves = [0,0,0]
        self.changes = [0,0,0]
        self.buffer_full = False

    def get_direction(self, sensor_data):
        """Interprets the sensor data and returns the discrete position of the robot. sensor_data argument is a list
        of length 3. Returns -1, 0, or 1 for if the robot is to the left, center, or right of the line."""
        if self.buffer_full:
            for i in range(len(sensor_data)):
                self.running_data[i].append(sensor_data[i])
                del self.running_data[i][0]
                ave = np.average(self.running_data[i])
                self.changes[i] = (ave - self.running_aves[i]) * self.moving_ave_num * self.d_gain
                self.running_aves[i] = ave
        else:  # Buffer isn't full yet.
            buffer_size = self._add_to_buffer(sensor_data)
            if buffer_size == self.moving_ave_num: self.buffer_full = True
            return 0  # Return a neutral position until buffer fills.

        direction = np.add(self.running_aves,self.changes) * self.p_gain
        direction -= np.min(direction)  # adjust down so the lowest value is zero.
        # 'Vote' on which direction to go.
        direction[0] = direction[0] * -1
        direction[1] = 0
        direction[2] = direction[2] * 1
        direction = np.sum(direction) * self.line_polarity
        if direction < -1: direction = -1
        if direction > 1: direction = 1
        return direction


    def _add_to_buffer(self, sensor_data):
        for i in range(len(sensor_data)):
            self.running_data[i].append(sensor_data[i])
            ave = np.average(self.running_data[i])
            self.changes[i] = ave - self.running_aves
            self.running_aves[i] = ave
        buffer_size = len(self.running_data[0])
        return buffer_size


def test():
    data = [[191, 223, 210],[181, 230, 214],[185, 224, 207],[184, 225, 211],[187, 224, 211],[186, 233, 205],
            [181, 232, 206],[190, 226, 210],[187, 226, 211],[182, 229, 213],[184, 229, 211],[185, 231, 210],
            [190, 227, 207],[187, 230, 210],[185, 227, 210]]
    interpreter = Interpreter()
    for i in range(len(data)):
        print(interpreter.get_direction(data[i]))



def main():
    logging.getLogger().setLevel(logging.INFO)
    test()

if __name__ == '__main__':
    main()
