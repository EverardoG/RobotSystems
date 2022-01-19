#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import picarx_improved
import logging

def cal_steering(px):
    """Calibrate the steering."""
    desired_angle = 0
    while True:
        input("\nMove the robot to an open area. \nPress enter to begin calibration.\n> ")
        px.forward(50)
        time.sleep(2)
        px.stop()
        while True:
            try:
                error = int(float(input("How many degrees of angle was the steering off? "
                                    "\n(Negative values for left, positive values for right, zero if approx straight)\n> ")))
                assert abs(error) <= 40
                break
            except:
                print("\nInvalid entry. Please enter a number less than 40.")
        if error == 0: break
        px.dir_cal_value = px.dir_cal_value + error
        angle_calibrated = desired_angle + px.dir_cal_value
        px.servo_dir.angle(angle_calibrated)
    print("\nCalibrated 0 degree angle set to: {}".format(px.dir_cal_value))
    px.config_file_obj.set("picarx_dir_servo", "%s" % px.dir_cal_value)

if __name__ == '__main__':
    logging.getLogger().setLevel(logging.DEBUG)
    print("Init car...\n")
    px = picarx_improved.Picarx()
    print("Made it...\n")
    cal_steering(px)