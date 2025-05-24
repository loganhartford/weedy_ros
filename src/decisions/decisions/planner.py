





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
        
        # # Symposium Demo
        # start_x = 0.4
        # start_y = 0.0
        # x = 1.55
        # y = 0.4
        # self.path = [

        #     # Undock
        #     [TRAVEL, start_x, start_y, 0.0],

        #     # To far end
        #     [WORK, start_x + x, start_y, 0.0],

        #     # Move over
        #     [ROTATE, 0.0, 0.0, np.pi/2],
        #     [UNDOCK, start_x + x, start_y + y, np.pi/2],
        #     [ROTATE, 0.0, 0.0, np.pi],

        #     # To near end
        #     [WORK, start_x, start_y + y, np.pi],
            
        #     # Return
        #     [ROTATE, 0.0, 0.0, -np.pi/2],
        #     [UNDOCK, start_x, start_y+0.05, -np.pi/2],

        #     # Dock
        #     [ROTATE, 0.0, 0.0, 0.0],
        #     [DOCK, -start_x - 0.05, 0.0, -0.13],
        #     [DONE, 0.0, 0.0, 0.0],
        # ]

        # FDR Circuit
        # start_x = 0.7
        # start_y = 0.0
        # x = 1.55
        # y = 0.55
        # self.path = [

        #     # Undock
        #     [TRAVEL, start_x, start_y, 0.0],

        #     # To far end
        #     [WORK, start_x + x, start_y, 0.0],

        #     # Move over
        #     [ROTATE, 0.0, 0.0, np.pi/2],
        #     [UNDOCK, start_x + x, start_y + y/2, np.pi/2],
        #     [ROTATE, 0.0, 0.0, np.pi],

        #     # To near end
        #     [WORK, start_x, start_y + y/2, np.pi],

        #     # Move over
        #     [ROTATE, 0.0, 0.0, np.pi/2],
        #     [UNDOCK, start_x, start_y + y, np.pi],
        #     [ROTATE, 0.0, 0.0, 0.0],

        #     # To far end
        #     [WORK, start_x + x, start_y + y, np.pi],
            
        #     # Return
        #     [ROTATE, 0.0, 0.0, np.pi],
        #     [TRAVEL, start_x, start_y + 0.17, np.pi],

        #     # Dock
        #     [ROTATE, 0.0, 0.0, 0.0],
        #     [DOCK, -start_x - 0.05, 0.0, -0.13],
        #     [DONE, 0.0, 0.0, 0.0],
        # ]

        # MTE546 Circuits
        # Out and Back
        # self.path = [
        #     # Out
        #     [TRAVEL, 1.5, 0.0, 0.0],
        #     # Turn
        #     [ROTATE, 0.0, 0.0, np.pi],
        #     # Back
        #     [TRAVEL, 0.0, 0.0, np.pi],
        #     # Turn
        #     [ROTATE, 0.0, 0.0, 0.0],
        #     # Out
        #     [TRAVEL, 1.5, 0.0, 0.0],
        #     # Turn
        #     [ROTATE, 0.0, 0.0, np.pi],
        #     # Back
        #     [TRAVEL, 0.0, 0.0, np.pi],
        #     # Turn
        #     [ROTATE, 0.0, 0.0, 0.0],
        #     # Out
        #     [TRAVEL, 1.5, 0.0, 0.0],
        #     # Turn
        #     [ROTATE, 0.0, 0.0, np.pi],
        #     # Back
        #     [TRAVEL, 0.0, 0.0, np.pi],
        #     # Turn
        #     [ROTATE, 0.0, 0.0, 0.0],
        #     [DONE, 0.0, 0.0, 0.0],
        # ]

        # FDR Circuit (no work)
        # start_x = 0.7
        # start_y = 0.0
        # x = 1.2
        # y = 0.55
        # self.path = [

        #     # Undock
        #     [TRAVEL, start_x, start_y, 0.0],

        #     # To far end
        #     [TRAVEL, start_x + x, start_y, 0.0],

        #     # Move over
        #     [ROTATE, 0.0, 0.0, np.pi/2],
        #     [UNDOCK, start_x + x, start_y + y/2, np.pi/2],
        #     [ROTATE, 0.0, 0.0, np.pi],

        #     # To near end
        #     [TRAVEL, start_x, start_y + y/2, np.pi],

        #     # Move over
        #     [ROTATE, 0.0, 0.0, np.pi/2],
        #     [UNDOCK, start_x, start_y + y, np.pi],
        #     [ROTATE, 0.0, 0.0, 0.0],

        #     # To far end
        #     [TRAVEL, start_x + x, start_y + y, np.pi],
            
        #     # Return
        #     [ROTATE, 0.0, 0.0, np.pi],
        #     [TRAVEL, start_x, start_y, np.pi],

        #     # Dock
        #     [ROTATE, 0.0, 0.0, 0.0],
        #     [DOCK, -start_x, 0.0, 0.0],
        #     [DONE, 0.0, 0.0, 0.0],
        # ]

        # # Symposium Demo
        # start_x = 0.4
        # start_y = 0.0
        # x = 1.2
        # y = 0.4
        # self.path = [

        #     # Undock
        #     [TRAVEL, start_x, start_y, 0.0],

        #     # To far end
        #     [TRAVEL, start_x + x, start_y, 0.0],

        #     # Move over
        #     [ROTATE, 0.0, 0.0, np.pi/2],
        #     [UNDOCK, start_x + x, start_y + y, np.pi/2],
        #     [ROTATE, 0.0, 0.0, np.pi],

        #     # To near end
        #     [TRAVEL, start_x, start_y + y, np.pi],
            
        #     # Return
        #     [ROTATE, 0.0, 0.0, -np.pi/2],
        #     [UNDOCK, start_x, start_y, -np.pi/2],

        #     # Dock
        #     [ROTATE, 0.0, 0.0, 0.0],
        #     [DOCK, -start_x, 0.0, 0.0],
        #     [DONE, 0.0, 0.0, 0.0],
        # ]

        # # Rectangle
        # self.path = [
        #     # Out
        #     [TRAVEL, 1.5, 0.0, 0.0],
        #     # Turn
        #     [ROTATE, 0.0, 0.0, np.pi/2],
        #     # Left
        #     [TRAVEL, 1.5, 0.7, np.pi/2],
        #     # Turn
        #     [ROTATE, 0.0, 0.0, np.pi],
        #     # Back
        #     [TRAVEL, 0.0, 0.7, np.pi],
        #     # Turn
        #     [ROTATE, 0.0, 0.0, -np.pi/2],
        #     # Right
        #     [TRAVEL, 0.0, 0.0, -np.pi/2],
        #     # Turn
        #     [ROTATE, 0.0, 0.0, 0.0],

        #     # Out
        #     [TRAVEL, 1.5, 0.0, 0.0],
        #     # Turn
        #     [ROTATE, 0.0, 0.0, np.pi/2],
        #     # Left
        #     [TRAVEL, 1.5, 0.7, np.pi/2],
        #     # Turn
        #     [ROTATE, 0.0, 0.0, np.pi],
        #     # Back
        #     [TRAVEL, 0.0, 0.7, np.pi],
        #     # Turn
        #     [ROTATE, 0.0, 0.0, -np.pi/2],
        #     # Right
        #     [TRAVEL, 0.0, 0.0, -np.pi/2],
        #     # Turn
        #     [ROTATE, 0.0, 0.0, 0.0],

        #     # Out
        #     [TRAVEL, 1.5, 0.0, 0.0],
        #     # Turn
        #     [ROTATE, 0.0, 0.0, np.pi/2],
        #     # Left
        #     [TRAVEL, 1.5, 0.7, np.pi/2],
        #     # Turn
        #     [ROTATE, 0.0, 0.0, np.pi],
        #     # Back
        #     [TRAVEL, 0.0, 0.7, np.pi],
        #     # Turn
        #     [ROTATE, 0.0, 0.0, -np.pi/2],
        #     # Right
        #     [TRAVEL, 0.0, 0.0, -np.pi/2],
        #     # Turn
        #     [ROTATE, 0.0, 0.0, 0.0],

        #     [DONE, 0.0, 0.0, 0.0],
        # ]

        # Arcs
        self.path = [
            [TRAVEL, 1.5, 0.7, 0.0],
            [ROTATE, 0.0, 0.0, np.pi],
            [TRAVEL, 0.0, 0.0, 0.0],
            [ROTATE, 0.0, 0.0, 0.0],

            [TRAVEL, 1.5, 0.7, 0.0],
            [ROTATE, 0.0, 0.0, np.pi],
            [TRAVEL, 0.0, 0.0, 0.0],
            [ROTATE, 0.0, 0.0, 0.0],

            [TRAVEL, 1.5, 0.7, 0.0],
            [ROTATE, 0.0, 0.0, np.pi],
            [TRAVEL, 0.0, 0.0, 0.0],
            [ROTATE, 0.0, 0.0, 0.0],
            
            [DONE, 0.0, 0.0, 0.0],
        ]

        # Swerving
        self.path = [
            [TRAVEL, 0.2, 0.1, 0.0],
            [TRAVEL, 0.4, 0.0, 0.0],
            [TRAVEL, 0.6, 0.1, 0.0],
            [TRAVEL, 0.8, 0.0, 0.0],
            [TRAVEL, 1.0, 0.1, 0.0],
            [TRAVEL, 1.2, 0.0, 0.0],
            [TRAVEL, 1.4, 0.1, 0.0],
            [TRAVEL, 1.6, 0.0, 0.0],
            [ROTATE, 0.0, 0.0, np.pi],
            [TRAVEL, 1.6, 0.1, 0.0],
            [TRAVEL, 1.4, 0.0, 0.0],
            [TRAVEL, 1.2, 0.1, 0.0],
            [TRAVEL, 1.0, 0.0, 0.0],
            [TRAVEL, 0.8, 0.1, 0.0],
            [TRAVEL, 0.6, 0.0, 0.0],
            [TRAVEL, 0.4, 0.1, 0.0],
            [TRAVEL, 0.2, 0.0, 0.0],
            [TRAVEL, 0.0, 0.0, 0.0],
            [ROTATE, 0.0, 0.0, 0.0],

            [TRAVEL, 0.2, 0.1, 0.0],
            [TRAVEL, 0.4, 0.0, 0.0],
            [TRAVEL, 0.6, 0.1, 0.0],
            [TRAVEL, 0.8, 0.0, 0.0],
            [TRAVEL, 1.0, 0.1, 0.0],
            [TRAVEL, 1.2, 0.0, 0.0],
            [TRAVEL, 1.4, 0.1, 0.0],
            [TRAVEL, 1.6, 0.0, 0.0],
            [ROTATE, 0.0, 0.0, np.pi],
            [TRAVEL, 1.6, 0.1, 0.0],
            [TRAVEL, 1.4, 0.0, 0.0],
            [TRAVEL, 1.2, 0.1, 0.0],
            [TRAVEL, 1.0, 0.0, 0.0],
            [TRAVEL, 0.8, 0.1, 0.0],
            [TRAVEL, 0.6, 0.0, 0.0],
            [TRAVEL, 0.4, 0.1, 0.0],
            [TRAVEL, 0.2, 0.0, 0.0],
            [TRAVEL, 0.0, 0.0, 0.0],
            [ROTATE, 0.0, 0.0, 0.0],

            [TRAVEL, 0.2, 0.1, 0.0],
            [TRAVEL, 0.4, 0.0, 0.0],
            [TRAVEL, 0.6, 0.1, 0.0],
            [TRAVEL, 0.8, 0.0, 0.0],
            [TRAVEL, 1.0, 0.1, 0.0],
            [TRAVEL, 1.2, 0.0, 0.0],
            [TRAVEL, 1.4, 0.1, 0.0],
            [TRAVEL, 1.6, 0.0, 0.0],
            [ROTATE, 0.0, 0.0, np.pi],
            [TRAVEL, 1.6, 0.1, 0.0],
            [TRAVEL, 1.4, 0.0, 0.0],
            [TRAVEL, 1.2, 0.1, 0.0],
            [TRAVEL, 1.0, 0.0, 0.0],
            [TRAVEL, 0.8, 0.1, 0.0],
            [TRAVEL, 0.6, 0.0, 0.0],
            [TRAVEL, 0.4, 0.1, 0.0],
            [TRAVEL, 0.2, 0.0, 0.0],
            [TRAVEL, 0.0, 0.0, 0.0],
            [ROTATE, 0.0, 0.0, 0.0],
            
            [DONE, 0.0, 0.0, 0.0],
        ]


        # self.path = [
        #     [TRAVEL, start_x, start_y, 0.0],
        #     [WORK, start_x + x, start_y, 0.0],
        #     [ROTATE, 0.0, 0.0, np.pi/2],
        #     [WORK, start_x + x, start_y + y, np.pi/2],
        #     [ROTATE, 0.0, 0.0, np.pi],
        #     [WORK, start_x, start_y + y, np.pi],
        #     [ROTATE, 0.0, 0.0, -np.pi/2],
        #     [WORK, start_x, start_y + 0.05, -np.pi/2],
        #     [ROTATE, 0.0, 0.0, 0.0],
        #     [DOCK, -start_x - 0.05, 0.0, -0.13],
        #     [DONE, 0.0, 0.0, 0.0],
        # ]

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
        # self.path = [
        #     [WORK, start_x + x, start_y, 0.0],
        #     [ROTATE, 0.0, 0.0, np.pi/2],
        #     [WORK, start_x + x, start_y + y, np.pi/2],
        #     [ROTATE, 0.0, 0.0, np.pi],
        #     [WORK, start_x, start_y + y, np.pi],
        #     [ROTATE, 0.0, 0.0, -np.pi/2],
        #     [WORK, start_x, start_y, -np.pi/2],
        #     [ROTATE, 0.0, 0.0, 0.0],
        #     [DONE, start_x, start_y, 0.0],
        # ]

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

        

    def plan(self):
        return self.path
