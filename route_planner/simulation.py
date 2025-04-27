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
    

    def follow_boundary(self, x, y, heading, direction='right'):
        '''
        Hug and follow the obstacle boundary.
        
        Input:
            x, y: Current position of the robot
            heading: Current heading of the robot in radians
            direction: 'right' or 'left' to specify the side to follow the obstacle
            
        Returns:
            Updated (x, y) position and new heading
        '''
        # Small angle increment for fine rotation adjustments
        angle_step = np.deg2rad(5)  # 5 degrees per step
        
        # Determine angles based on following side
        if direction == 'right':
            right_side = heading + np.pi/2  # Right side of the robot
            front_angle = heading
            left_diagonal = heading - np.pi/4  # Diagonal left front
            left_side = heading - np.pi/2  # Left side of the robot
        else:
            right_side = heading - np.pi/2  # Right side of the robot
            front_angle = heading
            left_diagonal = heading + np.pi/4  # Diagonal left front
            left_side = heading + np.pi/2  # Left side of the robot
        
        # Check 1: Front blocked, right open: turn right and maintain obstacle on the left
        if not self.navigability(x, y, front_angle):
            new_heading = heading + angle_step if direction == 'right' else heading - angle_step
            new_heading = new_heading % (2 * np.pi)
            
            if self.navigability(x, y, new_heading):
                # print("right turn and forward")
                return self.move_forward(x, y, new_heading), new_heading
            # print("right turn")
            return (x, y), new_heading

        # Check 2: Turn left if obstacle ends 
        elif self.navigability(x, y, left_side) and self.navigability(x, y, left_diagonal, 8):
            new_heading = heading - angle_step if direction == 'right' else heading - angle_step
            new_heading = new_heading % (2 * np.pi)
            # print("left turn")
            return (x, y), new_heading
        
        # Check 3: Front, right, left all blocked: take U-turn
        elif not self.navigability(x, y, front_angle) and not self.navigability(x, y, right_side) and not self.navigability(x, y, left_side):
            new_heading = heading - angle_step if direction == 'right' else heading + angle_step
            new_heading = new_heading % (2 * np.pi)
            # print("u-turn")
            return (x, y), new_heading

        # print("forward")
        return self.move_forward(x, y, front_angle), front_angle
    

    def mline_crossing(self, x1, y1, xt, yt, x, y):
        '''
        Input: x1, y1, xt, yt: start and end points of the m-line
               x, y: current position
        Returns: sign of the point w.r.t the m-line
        '''
        # TODO: your code
        
        # calculate the slope between the inital and final points
        m = self.set_heading(x1, y1, xt, yt)
        
        # calculate the slope between the current position and the final point
        n = self.set_heading(x, y, xt, yt)

        # calculate the sign of the point w.r.t the m-line, provide some tolerance for comparing the slopes.
        if abs(m - n) <= 1e-2:
        # if m == n:
            # gc.collect()
            return 0
        elif m > n:
            # gc.collect()
            return -1
        else:
            # gc.collect()
            return 1
        

   
    
    