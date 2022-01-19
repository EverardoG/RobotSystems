import sys
import os
picarx_lib_path = os.getcwd().replace('picar_orig_examples', 'lib')
sys.path.append(picarx_lib_path)  # Path to dynamixel sdk package.
from utils import reset_mcu
reset_mcu()
from grayscale_module import Grayscale_Module
import picarx_improved


if __name__=='__main__':
  try:
    gm = Grayscale_Module(500)
    px = picarx_improved.Picarx()
    px_power = 10
    while True:
        gm_val_list = gm.get_grayscale_data()
        print("gm_val_list:",gm_val_list)
        gm_status = gm.get_line_status(gm_val_list)
        print("gm_status:",gm_status)

        if gm_status == 'forward':
            print(1)
            px.forward(px_power) 

        elif gm_status == 'left':
            px.set_dir_servo_angle(12)
            px.forward(px_power) 

        elif gm_status == 'right':
            px.set_dir_servo_angle(-12)
            px.forward(px_power) 
        else:
            px.set_dir_servo_angle(0)
            px.stop()
  
  finally:
      px.stop()
