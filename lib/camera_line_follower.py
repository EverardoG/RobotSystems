#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Uses the picarx camera as a sensor to follow lines forever. Adopted from line_following_controller.py and still includes
unnecessary variables from that script. """

import picarx_improved
import time
import numpy as np
import atexit
import line_following_interpreter as interp
import grayscale_module
import picarx_improved
from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2
from utils import reset_mcu
import camera_line_sensor
reset_mcu()
import logging
logging.basicConfig(format="%(asctime)s:%(message)s", level=logging.INFO, datefmt="%H:%M:%S")
logging.getLogger().setLevel(logging.INFO)

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

        self.camera = PiCamera()
        self.camera.resolution = (640, 480)
        self.camera.framerate = 24
        self.rawCapture = PiRGBArray(self.camera, size=self.camera.resolution)
        self.lane_follower = camera_line_sensor.HandCodedLaneFollower()

    def _get_steering_angle(self,direction):
        """Takes the "direction" from the interpreter and returns a steering angle for the car."""
        steering_angle = np.interp(direction, self.dir_range, self.steering_angle_range)
        return steering_angle

    def follow_line(self):
        logging.info("Start lane following.\n")
        for frame in self.camera.capture_continuous(self.rawCapture, format="bgr", use_video_port=True):  # use_video_port=True
            img = frame.array
            combo_image = self.lane_follower.follow_lane(img)
            cv2.imshow("video", combo_image)  # OpenCV image show
            self.rawCapture.truncate(0)  # Release cache

            k = cv2.waitKey(1) & 0xFF
            # 27 is the ESC key, which means that if you press the ESC key to exit
            if k == 27:
                self.camera.close()
                break
            goal_steering_angle = self.lane_follower.curr_steering_angle - 90
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
    car = Controller(proportional_gain=10,derivative_gain=1,line_polarity='darker',pwm_percent = 30)
    car.follow_line()

if __name__ == '__main__':
    main()
