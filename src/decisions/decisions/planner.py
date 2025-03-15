

x = 1.4
y = 0.5

start_x = 0.8
start_y = 0.0

import numpy as np
import utils.robot_params as rp

POSITIONING = rp.MotionType.POSITTIONING
WORK = rp.MotionType.WORK
TRAVEL = rp.MotionType.TRAVEL
ROTATE = rp.MotionType.ROTATE
DONE = rp.MotionType.DONE

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

        # # CCW Loop
        self.path = [
            [WORK, start_x, start_y, 0.0],
            [ROTATE, 0.0, 0.0, np.pi/2],
            [WORK, start_x + x, start_y + y, -np.pi/2],
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
        if self.index >= len(self.path):
            return "done", None
        ret = self.path[self.index]
        self.index += 1
        return ret

    def reset(self):
        self.index = 0
