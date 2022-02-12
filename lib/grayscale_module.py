#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from adc import ADC
import line_following_interpreter
from utils import reset_mcu
reset_mcu()
import logging
logging.basicConfig(format="%(asctime)s:%(message)s", level=logging.INFO, datefmt="%H:%M:%S")
logging.getLogger().setLevel(logging.DEBUG)
from logdecorator import log_on_start,log_on_error,log_on_end

class Grayscale_Module(object):
    def __init__(self,ref = 1000):
        self.chn_0 = ADC("A0")
        self.chn_1 = ADC("A1")
        self.chn_2 = ADC("A2")
        self.ref = ref

    def get_line_status(self,fl_list):

        if fl_list[0] > self.ref and fl_list[1] > self.ref and fl_list[2] > self.ref:
            return 'stop'
            
        elif fl_list[1] <= self.ref:
            return 'forward'
        
        elif fl_list[0] <= self.ref:
            return 'right'

        elif fl_list[2] <= self.ref:
            return 'left'

    def get_grayscale_data(self):
        adc_value_list = []
        adc_value_list.append(self.chn_0.read())
        adc_value_list.append(self.chn_1.read())
        adc_value_list.append(self.chn_2.read())
        return adc_value_list

    def __call__(self):
        return self.get_grayscale_data()

def main():
    import time
    interp = line_following_interpreter.Interpreter()
    GM = Grayscale_Module(950)
    while True:
        print(interp.get_direction(GM.get_grayscale_data()))
        # print(GM.get_grayscale_data())
        # time.sleep(.1)

if __name__ == "__main__":
    main()
