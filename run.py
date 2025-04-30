import threading
import time

# Coords location shared by tello_control & traversal
location = [0,0]

semaphore = threading.Semaphore(2)

import tello_control
import traversal

def init_drone():
    tello_control.init_drone()

def init_bosdyn():
    traversal.init_spot()

def main():
    init_drone()
    time.sleep(1)
    init_bosdyn()


if __name__ == '__main__':
    main()