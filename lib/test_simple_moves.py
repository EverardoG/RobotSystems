import picarx_improved
import logging


if __name__ == '__main__':
    logging.getLogger().setLevel(logging.DEBUG)
    px = picarx_improved.Picarx()
    input('\nPress enter to go forward: ')
    px.forward(50)
    input('\nPress enter to change the steering angle: ')
    px.set_dir_servo_angle(10)
    input('\nPress enter to go forward: ')
    px.forward(50)