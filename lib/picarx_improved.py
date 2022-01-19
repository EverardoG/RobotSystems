#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Class to control PiCar-X with simulated classes for offline testing."""

import os
import time
import numpy as np
import atexit
import platform
if platform.node() == 'raspberrypi':  # Import real classes.
    from servo import Servo
    from pwm import PWM
    from pin import Pin
    from adc import ADC
    from filedb import fileDB
else: # Not being run on the raspberry pi. Import simulation classes.
    from servo_sim import Servo
    from pwm_sim import PWM
    from pin_sim import Pin
    from adc_sim import ADC
    from filedb_sim import fileDB
import logging
logging.basicConfig(format="%(asctime)s:%(message)s", level=logging.INFO, datefmt="%H:%M:%S")
logging.getLogger().setLevel(logging.DEBUG)
from logdecorator import log_on_start,log_on_error,log_on_end


class Picarx(object):
    PERIOD = 4095
    PRESCALER = 10  #todo this may be the scaling referred to in the assignment prompt?
    TIMEOUT = 0.02

    def __init__(self):
        # Config file setup.
        home_directory = os.path.expanduser('~')  # Home directory changes for simulated Picarx (laptop), vs real (raspi).
        self.config_file_obj = fileDB(home_directory + '/.config')
        # Create hardware interface objects.
        self.servo_camera_pan = Servo(PWM('P0'))
        self.servo_camera_tilt = Servo(PWM('P1'))
        self.servo_dir = Servo(PWM('P2'))
        self.dir_cal_value = int(self.config_file_obj.get("picarx_dir_servo", default_value=0))
        self.cam_cal_value_pan = int(self.config_file_obj.get("picarx_cam1_servo", default_value=0))
        self.cam_cal_value_tilt = int(self.config_file_obj.get("picarx_cam2_servo", default_value=0))
        self.servo_dir.angle(self.dir_cal_value)
        self.servo_camera_pan.angle(self.cam_cal_value_pan)
        self.servo_camera_tilt.angle(self.cam_cal_value_tilt)
        self.left_rear_pwm_pin = PWM("P13")
        self.right_rear_pwm_pin = PWM("P12")
        self.left_rear_dir_pin = Pin("D4")
        self.right_rear_dir_pin = Pin("D5")
        self.S0 = ADC('A0')
        self.S1 = ADC('A1')
        self.S2 = ADC('A2')

        self.motor_direction_pins = [self.left_rear_dir_pin, self.right_rear_dir_pin]
        self.motor_speed_pins = [self.left_rear_pwm_pin, self.right_rear_pwm_pin]
        for pin in self.motor_speed_pins:
            pin.period(self.PERIOD)
            pin.prescaler(self.PRESCALER)
        self.cali_dir_value = self.config_file_obj.get("picarx_dir_motor", default_value="[1,1]")
        self.cali_dir_value = [int(i.strip()) for i in self.cali_dir_value.strip("[]").split(",")]
        self.cali_speed_value = [0, 0]
        self.dir_current_angle = 0
        atexit.register(self.cleanup)



    @log_on_start(logging.DEBUG, "[set_motor_speed] motor: {motor}, speed: {speed}")
    @log_on_error(logging.DEBUG, "[set_motor_speed] Error")
    def set_motor_speed(self,motor,speed):
        #todo what are the units of speed???
        motor -= 1
        if speed >= 0:
            direction = 1 * self.cali_dir_value[motor]
        elif speed < 0:
            direction = -1 * self.cali_dir_value[motor]
        speed = abs(speed)
        # todo I believe the below lines are the "speed scaling"
        # if speed != 0:
        #     speed = int(speed /2 ) + 50
        speed = speed - self.cali_speed_value[motor]
        if direction < 0:
            self.motor_direction_pins[motor].high()
            self.motor_speed_pins[motor].pulse_width_percent(speed)
        else:
            self.motor_direction_pins[motor].low()
            self.motor_speed_pins[motor].pulse_width_percent(speed)

    @log_on_start(logging.DEBUG, "[motor_speed_calibration] value: {value}")
    @log_on_error(logging.DEBUG, "[motor_speed_calibration] Error")
    def motor_speed_calibration(self,value):
        """Todo This method looks like it does not work. It is not used in the example code. What is value supposed to be?? A list or int?"""
        self.cali_speed_value = value
        #todo: value has to be a number to use comparitive operator, but then you can't index a number...
        if value < 0:
            self.cali_speed_value[0] = 0
            self.cali_speed_value[1] = abs(self.cali_speed_value)
        else:
            self.cali_speed_value[0] = abs(self.cali_speed_value)
            self.cali_speed_value[1] = 0

    @log_on_start(logging.DEBUG, "[motor_direction_calibration] motor: {motor}, value: {value}")
    @log_on_error(logging.DEBUG, "[motor_direction_calibration] Error")
    def motor_direction_calibration(self,motor, value):
        # 0: positive direction
        # 1:negative direction
        motor -= 1
        if value == 1:
            self.cali_dir_value[motor] = -1 * self.cali_dir_value[motor]
        self.config_file_obj.set("picarx_dir_motor", self.cali_dir_value)

    @log_on_start(logging.DEBUG, "[dir_servo_angle_calibration] value: {value}")
    @log_on_error(logging.DEBUG, "[dir_servo_angle_calibration] Error")
    def dir_servo_angle_calibration(self,value):
        """Set the calibration angle."""
        self.dir_cal_value = value
        # print("calibrationdir_cal_value:",self.dir_cal_value)
        self.config_file_obj.set("picarx_dir_servo", "%s"%value)
        # self.servo_dir.angle(value)

    @log_on_start(logging.DEBUG, "[set_dir_servo_angle] value: {value}")
    @log_on_error(logging.DEBUG, "[set_dir_servo_angle] Error")
    def set_dir_servo_angle(self,value):
        self.dir_current_angle = value
        angle_value  = value + self.dir_cal_value
        # print("angle_value:",angle_value)
        # # print("set_dir_servo_angle_1:",angle_value)
        # # print("set_dir_servo_angle_2:",dir_cal_value)
        self.servo_dir.angle(angle_value)

    @log_on_start(logging.DEBUG, "[camera_servo1_angle_calibration] value: {value}")
    @log_on_error(logging.DEBUG, "[camera_servo1_angle_calibration] Error")
    def camera_servo1_angle_calibration(self,value):
        self.cam_cal_value_pan = value
        self.config_file_obj.set("picarx_cam1_servo", "%s"%value)
        # print("cam_cal_value_pan:",self.cam_cal_value_pan)
        self.servo_camera_pan.angle(value)

    @log_on_start(logging.DEBUG, "[camera_servo2_angle_calibration] value: {value}")
    @log_on_error(logging.DEBUG, "[camera_servo2_angle_calibration] Error")
    def camera_servo2_angle_calibration(self,value):
        self.cam_cal_value_tilt = value
        self.config_file_obj.set("picarx_cam2_servo", "%s"%value)
        # print("picarx_cam2_servo:",self.cam_cal_value_tilt)
        self.servo_camera_tilt.angle(value)

    @log_on_start(logging.DEBUG, "[set_camera_servo1_angle] value: {value}")
    @log_on_error(logging.DEBUG, "[set_camera_servo1_angle] Error")
    def set_camera_servo1_angle(self,value):
        self.servo_camera_pan.angle(-1*(value + -1*self.cam_cal_value_pan))
        # print("self.cam_cal_value_pan:",self.cam_cal_value_pan)
        # print((value + self.cam_cal_value_pan))

    @log_on_start(logging.DEBUG, "[set_camera_servo2_angle] value: {value}")
    @log_on_error(logging.DEBUG, "[set_camera_servo2_angle] Error")
    def set_camera_servo2_angle(self,value):
        # global cam_cal_value_tilt
        self.servo_camera_tilt.angle(-1*(value + -1*self.cam_cal_value_tilt))
        # print("self.cam_cal_value_tilt:",self.cam_cal_value_tilt)
        # print((value + self.cam_cal_value_tilt))

    @log_on_start(logging.DEBUG, "[get_adc_value] Enter")
    @log_on_error(logging.DEBUG, "[get_adc_value] Error")
    @log_on_end(logging.DEBUG, "[get_adc_value] Result: {result}")
    def get_adc_value(self):
        adc_value_list = []
        adc_value_list.append(self.S0.read())
        adc_value_list.append(self.S1.read())
        adc_value_list.append(self.S2.read())
        return adc_value_list

    @log_on_start(logging.DEBUG, "[set_power] speed: {speed}")
    @log_on_error(logging.DEBUG, "[set_power] Error")
    def set_power(self,speed):
        self.set_motor_speed(1, speed)
        self.set_motor_speed(2, speed) 

    @log_on_start(logging.DEBUG, "[backward] speed: {speed}")
    @log_on_error(logging.DEBUG, "[backward] Error")
    def backward(self,speed):
        current_angle = self.dir_current_angle
        if current_angle != 0:
            abs_current_angle = abs(current_angle)
            # if abs_current_angle >= 0:
            if abs_current_angle > 40:
                abs_current_angle = 40
            power_scale = (100 - abs_current_angle) / 100.0 
            # print("power_scale:",power_scale)
            if (current_angle / abs_current_angle) > 0:
                self.set_motor_speed(1, -1*speed)
                self.set_motor_speed(2, speed * power_scale)
            else:
                self.set_motor_speed(1, -1*speed * power_scale)
                self.set_motor_speed(2, speed )
        else:
            self.set_motor_speed(1, -1*speed)
            self.set_motor_speed(2, speed)  

    @log_on_start(logging.DEBUG, "[forward] speed: {speed}")
    @log_on_error(logging.DEBUG, "[forward] Error")
    def forward(self,speed):
        current_angle = self.dir_current_angle
        if current_angle == 0: # Both motors go same speed.
            self.set_motor_speed(1, speed)
            self.set_motor_speed(2, -1 * speed)
        else: # Motors need different speeds to not slip.
            # If the steering angle is 90 deg to the right, the left wheel should move at the forward speed while the
            # right should not move (scaled to zero). A linear interpolation between directly forward and a 90 degree
            # turn is a reasonable approximation for the distribution of speed values to achieve no slip.
            abs_current_angle = abs(current_angle)
            pwm_range = [speed,0]
            angle_range = [0,90]
            scaled_speed = np.interp(abs_current_angle,angle_range,pwm_range)
            # if abs_current_angle > 40: abs_current_angle = 40
            # power_scale = (100 - abs_current_angle) / 100.0
            # # print("power_scale:",power_scale)
            if (current_angle / abs_current_angle) > 0: # Turning right
                self.set_motor_speed(1, speed)
                self.set_motor_speed(2, -1*scaled_speed)
            else: # Turning left
                self.set_motor_speed(1, scaled_speed)
                self.set_motor_speed(2, -1*speed )

    @log_on_start(logging.DEBUG, "[stop] Enter")
    @log_on_error(logging.DEBUG, "[stop] Error")
    def stop(self):
        self.set_motor_speed(1, 0)
        self.set_motor_speed(2, 0)

    @log_on_start(logging.DEBUG, "[Get_distance] Enter")
    @log_on_error(logging.DEBUG, "[Get_distance] Error")
    @log_on_end(logging.DEBUG, "[Get_distance] Result: {result}")
    def Get_distance(self):
        timeout=0.01
        trig = Pin('D8')
        echo = Pin('D9')

        trig.low()
        time.sleep(0.01)
        trig.high()
        time.sleep(0.000015)
        trig.low()
        pulse_end = 0
        pulse_start = 0
        timeout_start = time.time()
        while echo.value()==0:
            pulse_start = time.time()
            if pulse_start - timeout_start > timeout:
                return -1
        while echo.value()==1:
            pulse_end = time.time()
            if pulse_end - timeout_start > timeout:
                return -2
        during = pulse_end - pulse_start
        cm = round(during * 340 / 2 * 100, 2)
        #print(cm)
        return cm

    @log_on_start(logging.DEBUG, "[shutdown] Enter")
    def cleanup(self):
        logging.info("SHUTDOWN ON EXIT. MOTOR SPEEDS SET TO 0.")
        self.stop()


def test(px):
    px.set_motor_speed(1, 5)
    px.motor_direction_calibration(1,1)
    px.dir_servo_angle_calibration(1)
    px.set_dir_servo_angle(1)
    px.camera_servo1_angle_calibration(1)
    px.camera_servo2_angle_calibration(1)
    px.set_camera_servo1_angle(1)
    px.set_camera_servo2_angle(1)
    px.get_adc_value()
    px.set_power(10)
    px.backward(10)
    px.forward(10)
    px.stop()
    px.Get_distance()


def main():
    logging.getLogger().setLevel(logging.DEBUG)
    if platform.node() != 'raspberrypi':
        px = Picarx()
        test(px)

if __name__ == "__main__":
    main()
