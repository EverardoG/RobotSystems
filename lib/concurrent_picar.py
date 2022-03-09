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
import line_following_controller as control
import line_following_interpreter as interp
from ultrasonic import Ultrasonic
import grayscale_module
import picarx_improved
from utils import reset_mcu
reset_mcu()
import logging
logging.basicConfig(format="%(asctime)s:%(message)s", level=logging.INFO, datefmt="%H:%M:%S")
logging.getLogger().setLevel(logging.DEBUG)



def concurrent_sense(sensor,sensor_bus,sensor_delay):
    while True:
        raw_data = sensor.get_grayscale_data()
        sensor_bus.write(raw_data)
        time.sleep(sensor_delay)

def concurrent_interp(interpreter,sensor_bus,interp_bus,interp_delay):
    while True:
        sensor_data = sensor_bus.read()
        direction = interpreter.get_direction(sensor_data)
        interp_bus.write(direction)
        time.sleep(interp_delay)

def concurrent_control(controller,sonar,interp_bus,control_delay):
    while True:
        obstacle = sonar.obstacle()
        direction = interp_bus.read()
        controller.follow_line_with_ultrasonic(direction,obstacle)
        time.sleep(control_delay)

def init_busses(sensor_bus,interp_bus,sensor,interpreter):
    """Put valid valid values on the busses before entering operation."""
    sensor_data = sensor.get_grayscale_data()
    sensor_bus.write(sensor_data)
    direction = interpreter.get_direction(sensor_data)
    interp_bus.write(direction)

def cleanup(executor,car):
    executor.shutdown()
    car.stop()

def main():
    logging.getLogger().setLevel(logging.DEBUG)
    interpreter = interp.Interpreter(proportional_gain=10,derivative_gain=1,line_polarity='darker')
    sensor = grayscale_module.Grayscale_Module(950)
    car = picarx_improved.Picarx()
    sonar = Ultrasonic()
    controller = control.Controller(car,pwm_percent = 30)
    controller.fill_buffer(interpreter, sensor)
    sensor_bus = bus.Bus()
    interp_bus = bus.Bus()
    # Put valid valid values on the busses before entering operation.
    init_busses(sensor_bus,interp_bus,sensor,interpreter)

    # We can use a with statement to ensure threads are cleaned up promptly
    with concurrent.futures.ThreadPoolExecutor(max_workers=3) as executor:
        atexit.register(cleanup,executor, car)
        # Start each process, which will run forever...
        thread1 = executor.submit(concurrent_sense,sensor,sensor_bus,0.01)
        thread2 = executor.submit(concurrent_interp,interpreter,sensor_bus,interp_bus,0.01)
        thread3 = executor.submit(concurrent_control,controller,sonar,interp_bus,0.01)
    thread1.result()
    thread2.result()
    thread3.result()

if __name__ == '__main__':
    main()