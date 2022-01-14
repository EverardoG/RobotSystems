import time
import picarx_improved
import logging

def cal_steering(px):
    """Calibrate the steering."""
    desired_angle = 0
    while True:
        input("\nMove the robot to an open area. \nPress enter to begin calibration.\n> ")
        angle_calibrated = desired_angle + px.dir_cal_value
        px.servo_dir.angle(angle_calibrated)
        px.forward(50)
        time.sleep(4)
        px.stop()
        while True:
            try:
                error = float(input("How many degrees of angle was the steering off? "
                                    "\n(Negative values for left, positive values for right, zero if approx straight)\n> "))
                break
            except:
                print("\nInvalid entry. Please enter a number.")
        if error == 0: break
        px.dir_cal_value = px.dir_cal_value + error

if __name__ == '__main__':
    logging.getLogger().setLevel(logging.DEBUG)
    print("Init car...\n")
    px = picarx_improved.Picarx()
    print("Made it...\n")
    cal_steering(px)