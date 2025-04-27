"""
EEL 4930/5934: Autonomous Robots
University Of Florida
"""

import numpy as np
import gc

class Simulation:
    def __init__(self, map_data):
        self.map_data = map_data
        self.radius = 5
      

    def is_circle_navigable(self, xc, yc):
        '''
        Input: xc, yc: center of the circular robot
        Checks if all pixels within the circle are obstacle-free (navigable)
        Returns: True if circle is navigable, False otherwise
        '''
        r = self.radius
        for i in range(-r, r+1):
            for j in range(-r, r+1):
                if i**2 + j**2 <= r**2:  # Within circle
                    xi, yj = xc + i, yc + j
                    if (xi < 0 or xi >= self.map_data.shape[1] or 
                        yj < 0 or yj >= self.map_data.shape[0]):
                        return False
                    if self.map_data[yj, xi] != 255:
                        return False
        return True
    

    def navigability(self, x, y, heading, n_steps = 4):
        '''
        Input: x, y: current position
               heading: current heading
               n_steps: number of steps to check for navigability
        Checks if the robot can move forward n_steps in the current heading
        Returns: True if the robot can move forward, False otherwise
        '''
        for i in range (1, n_steps):
            # Calculate new position after each step
            x_new = int(round(x + i * np.cos(heading)))
            y_new = int(round(y + i * np.sin(heading)))
            if not self.is_circle_navigable(x_new, y_new):
                # print("New location not feasible")
                return False
        
        return True
    
    
    def set_heading(self, x, y, xt, yt):
        '''
        Input: x, y: current position
               xt, yt: target position
        Returns: heading in radians
        '''
        dx = xt - x
        dy = yt - y
        heading = np.arctan2(dy, dx)
        return heading % (2 * np.pi)
    
    
    def move_forward(self, x, y, heading):
        '''
        Input: x, y: current position
               heading: current heading
        Returns: new position after moving one step forward
        '''
        x_new = int(round(x + np.cos(heading)))
        y_new = int(round(y + np.sin(heading)))
        return x_new, y_new
    

    def rotate(self, heading):
        '''
        Input: heading: current heading
        Returns: new heading after rotating 1 degree clockwise
        '''
        # TODO: your code here
        return (heading + (np.pi/180)) % (2*np.pi)     


    def distance(self, x, y, xt, yt):
        '''
        Input: x, y: current position
               xt, yt: target position
        Returns: Euclidean distance between the two points
        '''
        return np.sqrt((x - xt)**2 + (y - yt)**2)
    


        

   
    
    