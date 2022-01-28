
import bus
import picarx_improved
import time
import numpy as np
import atexit
import line_following_interpreter as interp
import grayscale_module
import picarx_improved
from utils import reset_mcu
reset_mcu()
import logging
logging.basicConfig(format="%(asctime)s:%(message)s", level=logging.INFO, datefmt="%H:%M:%S")
logging.getLogger().setLevel(logging.DEBUG)