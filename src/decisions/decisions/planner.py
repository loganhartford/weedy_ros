

x = 1.2
y = 0.4

start_x = 0.0
stary_y = 0.0

import numpy as np

class Planner:
    
    def __init__(self):
        # hack
        # self.path = [
        #     [[x, 0.0, no_rotate]],
        #     [[0.0, 0.0, -1.57]],
        #     [[y, 0.0, no_rotate]],
        #     [[0.0, 0.0, -1.57]],
        #     [[x, 0.0, no_rotate]],
        #     [[0.0, 0.0, -1.57]],
        #     [[y, 0.0, no_rotate]],
        #     # [[0.0, 0.0, no_rotate]],
        # ]  
        self.path = [
            ("travel", [[start_x + x, stary_y, 0.0]]),
            ("rotate", [[0.0, 0.0, -np.pi/2]]),
            ("travel", [[start_x + x, stary_y - y, -np.pi/2]]),
            ("rotate", [[0.0, 0.0, np.pi]]),
            ("travel", [[start_x, stary_y - y, np.pi]]),
            ("rotate", [[0.0, 0.0, np.pi/2]]),
            ("travel", [[start_x, stary_y, np.pi/2]]),  
            ("rotate", [[0.0, 0.0, 0.0]]),  
            ]  
        self.index = 0

    def plan(self):
        if self.index >= len(self.path):
            return "done", None
        ret = self.path[self.index]
        self.index += 1
        return ret

    def reset(self):
        self.index = 0
