#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Module with concurrent operation of the picar-x."""

from readerwriterlock import rwlock
import bus
import concurrent.futures
import picarx_improved
import time
import numpy as np
import atexit
import rossros as rr
import line_following_controller as control
import line_following_interpreter as interp
import grayscale_module
import picarx_improved
from utils import reset_mcu
reset_mcu()
import logging
logging.basicConfig(format="%(asctime)s:%(message)s", level=logging.INFO, datefmt="%H:%M:%S")
logging.getLogger().setLevel(logging.DEBUG)


def main():
    logging.getLogger().setLevel(logging.DEBUG)
    interpreter = interp.Interpreter(proportional_gain=10,derivative_gain=1,line_polarity='darker')
    sensor = grayscale_module.Grayscale_Module(950)
    car = picarx_improved.Picarx()
    controller = control.Controller(car,pwm_percent = 30)
    controller.fill_buffer(interpreter, sensor)

    sensor_bus = rr.Bus(sensor(),"Sensor Bus")
    interp_bus = rr.Bus(interpreter(),"Interp Bus")
    control_bus = rr.Bus(0, "Control Bus")
    term_bus = rr.Bus(0,"Term Bus")

    readGroundScaner_prod = rr.Producer(sensor,sensor_bus,0.1)
    interpret_cons_prod = rr.Producer(interpreter, sensor_bus, 0.1)
    control_cons = rr.Producer(control, control_bus, 0.1)

    # We can use a with statement to ensure threads are cleaned up promptly
    with concurrent.futures.ThreadPoolExecutor(max_workers=3) as executor:
        # Start each process, which will run forever...
        thread1 = executor.submit(readGroundScaner_prod,sensor_bus,0.01)
        thread2 = executor.submit(interpret_cons_prod,interp_bus,0.01)
        thread3 = executor.submit(control_cons,control_bus,0.01)

    thread1.result()
    thread2.result()
    thread3.result()

if __name__ == '__main__':
    main()