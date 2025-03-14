

x = 1.2
y = 0.6

start_x = 0.0
start_y = 0.0

import numpy as np

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
            ("work", [[start_x + x, start_y, 0.0]]),
            ("done", [[start_x + x, start_y, 0.0]]),
            # ("rotate", [[0.0, 0.0, np.pi/2]]),
            # ("work", [[start_x + x, start_y + y, -np.pi/2]]),
            # ("rotate", [[0.0, 0.0, np.pi]]),
            # ("work", [[start_x, start_y + y, np.pi]]),
            # ("rotate", [[0.0, 0.0, -np.pi/2]]),
            # ("work", [[start_x, start_y, -np.pi/2]]),  
            # ("rotate", [[0.0, 0.0, 0.0]]),  
            ]  
        
        # self.path = [
        #     ("travel", [[start_x + x, start_y, 0.0]]),
        #     ("rotate", [[0.0, 0.0, np.pi/2]]),
        #     ("travel", [[start_x + x, start_y + y, -np.pi/2]]),
        #     ("rotate", [[0.0, 0.0, np.pi]]),
        #     ("travel", [[start_x, start_y + y, np.pi]]),
        #     ("rotate", [[0.0, 0.0, -np.pi/2]]),
        #     ("travel", [[start_x, start_y, -np.pi/2]]),  
        #     ("rotate", [[0.0, 0.0, 0.0]]),  
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
