

x = 1.2
y = 0.5

start_x = 0.0
stary_y = 0.0
no_rotate = 10.0 # just some large value

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
            [[start_x + x, stary_y, no_rotate]],
            [[start_x, stary_y, -1.57]],
            [[start_x + x, stary_y - y, no_rotate]],
            [[start_x + x, stary_y - y, 3.14]],
            [[start_x, stary_y - y, no_rotate]],
            [[start_x, stary_y - y, 1.57]],
            [[start_x, stary_y, no_rotate]],
        ]  
        self.type = ["travel", "travel", "travel", "travel"]
        self.index = 0

    def plan(self):
        if self.index >= len(self.path):
            return "done", None
        ret = ["travel", self.path[self.index]]
        self.index += 1
        return ret
