

class Planner:
    def __init__(self):
        self.path = [
            [[0.5, 0.0, 10.0], [0.9, -0.6, 10.0]],
            [[0.9, -0.6, 0.0]],
            [[1.4, -0.6, 10.0]],
            [[1.4, -0.6, -1.57]],
            [[1.4, -2.2, 10.0]],
            [[1.4, -2.2, -3.14]],
            [[0.9, -2.2, 10.0]],
            [[0.9, -2.2, 1.57]],
            [[0.9, -0.8, 10.0]],
            [[0.9, -0.6, 10.0]],
            [[0.4, 0.0, 10.0]],
            [[0.4, 0.0, 0.0]],
            [[0.0, 0.0, 10.0]],
        ]  
        self.type = ["travel", "travel", "travel", "travel"]
        self.index = 0

    def plan(self):
        if self.index >= len(self.path):
            return "done", None
        ret = ["travel", self.path[self.index]]
        self.index += 1
        return ret
