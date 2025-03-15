

x = 1.2
y = 0.5

start_x = 0.0
start_y = 0.0

import numpy as np
import utils.robot_params as rp

POSITION = rp.POSITION
DOCK = rp.DOCK
UNDOCK = rp.UNDOCK
WORK = rp.WORK
TRAVEL = rp.TRAVEL
ROTATE = rp.ROTATE
DONE = rp.DONE

class Planner:
    
    def __init__(self):
        # CW Loop
        # self.path = [
        #     ("travel", [[start_x + x, start_y, 0.0]]),
        #     ("rotate", [[0.0, 0.0, -np.pi/2]]),
        #     ("travel", [[start_x + x, start_y - y, -np.pi/2]]),
        #     ("rotate", [[0.0, 0.0, np.pi]]),
        #     ("travel", [[start_x, start_y - y, np.pi]]),
        #     ("rotate", [[0.0, 0.0, np.pi/2]]),
        #     ("travel", [[start_x, start_y, np.pi/2]]),  
        #     ("rotate", [[0.0, 0.0, 0.0]]),  
        #     ]  

        # CCW Loop
        self.path = [
            [WORK, start_x + x, start_y, 0.0],
            [ROTATE, 0.0, 0.0, np.pi/2],
            [WORK, start_x + x, start_y + y, np.pi/2],
            [ROTATE, 0.0, 0.0, np.pi],
            [WORK, start_x, start_y + y, np.pi],
            [ROTATE, 0.0, 0.0, -np.pi/2],
            [WORK, start_x, start_y, -np.pi/2],
            [ROTATE, 0.0, 0.0, 0.0],
            [DONE, start_x, start_y, 0.0],
        ]

        # self.path = [
        #     ("work", [[start_x + x, start_y, 0.0]]),
        #     ("rotate", [[0.0, 0.0, np.pi/2]]),
        #     ("work", [[start_x + x, start_y + y, -np.pi/2]]),
        #     ("rotate", [[0.0, 0.0, np.pi]]),
        #     ("work", [[start_x, start_y + y, np.pi]]),
        #     ("rotate", [[0.0, 0.0, -np.pi/2]]),
        #     ("work", [[start_x, start_y, -np.pi/2]]),  
        #     ("rotate", [[0.0, 0.0, 0.0]]), 
        #     ("done", [[start_x, start_y, 0.0]]),
        #     ]  

        # CCW Loop + Travel
        # self.path = [
        #     ("travel", [[start_x, start_y, 0.0]]),
        #     ("work", [[start_x + x, start_y, 0.0]]),
        #     ("rotate", [[0.0, 0.0, np.pi/2]]),
        #     ("work", [[start_x + x, start_y + y, np.pi/2]]),
        #     ("rotate", [[0.0, 0.0, np.pi]]),
        #     ("work", [[start_x, start_y + y, np.pi]]),
        #     ("rotate", [[0.0, 0.0, -np.pi/2]]),
        #     ("work", [[start_x, start_y, -np.pi/2]]),  
        #     ("rotate", [[0.0, 0.0, np.pi]]),
        #     ("travel", [[0.0, 0.0, np.pi]]),
        #     ("rotate", [[0.0, 0.0, 0.0]]),
        #     ("done", [[0.0, 0.0, 0.0]]),
        #     ]  
        self.index = 0

    def plan(self):
        return self.path

    def reset(self):
        self.index = 0
