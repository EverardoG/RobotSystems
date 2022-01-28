#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Contains an interpreter class that takes raw grayscale data from the picarx grayscale module and maps it to a
direction value between -1 and 1. The direction value is determined with a psuedo PD controller."""

import picarx_improved
import logging
import time
import numpy as np
import logging
logging.basicConfig(format="%(asctime)s:%(message)s", level=logging.INFO, datefmt="%H:%M:%S")
logging.getLogger().setLevel(logging.DEBUG)

class Interpreter(object):
    """Class for interpreting the grayscale sensor data into a discrete position. Higher sensor numbers are lighter,
    lower numbers are darker."""

    def __init__(self, proportional_gain=50,derivative_gain=5, line_polarity='darker'):
        polarity_map = {'darker':1,'lighter':-1}
        self.line_polarity = polarity_map[line_polarity]
        if line_polarity == 'darker': self.mostly_under_func = np.argmin
        else: self.mostly_under_func = np.argmax
        self.p_gain = proportional_gain / 5000
        self.d_gain = derivative_gain / 500
        self.running_data = [ [], [], [] ]
        self.running_aves = [0,0,0]
        self.deriv_vals = [0,0,0]
        self.prop_vals = [0,0,0]
        self.moving_ave_num = 2
        self.buffer_full = False
        self.line_centered = True

    def reset(self):
        self.running_data = [ [], [], [] ]
        self.running_aves = [0,0,0]
        self.deriv_vals = [0,0,0]
        self.buffer_full = False

    def get_direction(self, sensor_data):
        """Psuedo PD controller to turn sensor data into a direction value between -1 and 1, which is returned."""
        mostly_under = self.mostly_under_func(self.running_aves)  # The sensor index the target is mostly under.
        self.line_centered = True if mostly_under == 1 else False
        if self.buffer_full: # A buffer us used to fill up queues of data to get more reliable readings.
            for sensor_i in range(len(sensor_data)):
                self.running_data[sensor_i].append(sensor_data[sensor_i])
                del self.running_data[sensor_i][0]
                ave = np.average(self.running_data[sensor_i])

                """The derivative portion of the controller gives values to steer the car in response to the change
                of the values. There are two relevant scenarios:
                (1) The target is mostly under an outside sensor, and that sensor is seeing values that are changing away 
                from the target (lighter/darker), and the center sensor is also not changing in the right way. 
                In this case, if the outside sensor is turning less like the target it is about the lose the line (not moving
                more centered over the line.) Need to steer strongly back towards the outside sensor with the target under it. 
                (which is the opposite response given from case 2.)
                (2) Else. In all other cases if the outside sensor is becoming less like the target sensor, it is already
                 changing in the right direction. Don't need to turn into the center even more. 
                """
                change = ave - self.running_aves[sensor_i]
                self.deriv_vals[sensor_i] = change * self.d_gain * self.line_polarity *-1
                self.running_aves[sensor_i] = ave
            # negative deriv_vals are changing to be more like the target. The /4 value is a hand picked threshold.
            if not self.line_centered and self.deriv_vals[mostly_under] > 0 and self.deriv_vals[1] > -self.deriv_vals[mostly_under]/4:
                # Case 1. The car is about to lose the line, which requires the opposite response.
                self.deriv_vals.reverse()  # Give the opposite response.
            """The derivative portion of the controller is calculated at this point. Ex. [20,3,-6]. Alone this would result in 
            a steering direction value towards the first sensor (20) and away from the third sensor (-6)."""
        else:  # Buffer isn't full yet. Fill it.
            buffer_size = self._add_to_buffer(sensor_data)
            if buffer_size == self.moving_ave_num: self.buffer_full = True
            direction = 0
            return direction  # Return a neutral position until buffer fills.

        # Start calculating the proportional values by multiplying the sensor readings by the polarity and p gain.
        self.prop_vals = [x*self.line_polarity*self.p_gain for x in self.running_aves]
        # adjust down so the lowest value is zero. This makes the proportional value robust to different lighting conditions.
        self.prop_vals = self.prop_vals - np.min(self.prop_vals)
        self.prop_vals = np.flip(self.prop_vals) # Flip, because that is the way it works out...

        # Add the proportional and derivative terms to get a reference direction.
        raw_direction = np.add(self.prop_vals,self.deriv_vals)
        return self._transform_direction(raw_direction)

    def _transform_direction(self,direction):
        """Transform the PD controller reference direction (3 number list) to a single number between -1 and 1."""
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
            change = ave - self.running_aves[i]
            self.deriv_vals[i] = change
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
