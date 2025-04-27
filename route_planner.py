"""
EEL 4930/5934: Autonomous Robots
University Of Florida
"""

import numpy as np
import matplotlib.pyplot as plt
import argparse
from simulation import Simulation
from visualization import Visualization

# def run_bug0(sim, vis, origin, destination, max_iterations=1000):
#     """
#     Executes the Bug0 algorithm for a robot to navigate from origin to destination.

#     Args:
#         sim: The simulation environment with navigability, movement, and heading functions.
#         vis: The visualization module to update the map.
#         origin: Tuple (x, y) representing the starting position.
#         destination: Tuple (x, y) representing the target position.
#         max_iterations: Maximum number of iterations to prevent infinite loops.

#     Raises:
#         ValueError: If origin or destination is not set.
#         RuntimeError: If the simulation encounters an unexpected error.
#     """

#     # Validate inputs
#     if origin is None or destination is None:
#         raise ValueError("Start and end points must be set.")

    
#     state_dict = {
#         0: 'go to destination',
#         1: 'follow wall',
#     }  # Two possible states

#     STATE = 0  # Start with moving towards the destination

#     x, y = origin
#     xt, yt = destination
#     heading = sim.set_heading(x, y, xt, yt)

#     print(f"starting simulation with origin: {origin}, destination: {destination}")

#     for _ in range(max_iterations):  # Prevent infinite loops
#         if (x, y) == (xt, yt):
#             print("Reached the destination!")
#             break
         
#         if STATE == 0:  # Go to destination
#             heading = sim.set_heading(x, y, xt, yt)
            
#             # navigate towards the destination if the heading is navigable
#             if sim.navigability(x, y, heading):
#                 x_new, y_new = sim.move_forward(x, y, heading)
#                 x, y = x_new, y_new
                
#             # change states
#             else:
#                 STATE = 1
#                 print(f"STATE switched to: {state_dict[STATE]}")

#         elif STATE == 1:  # Follow Wall
            
#             # Rotate the robot until it reaches a new heading that is navigable parallel to the wall
#             front_angle = heading
#             new_heading = heading
            
#             while not sim.navigability(x, y, new_heading, n_steps=4):
#                 new_heading = sim.rotate(new_heading)
                
#             # Move foward on the new heading.
#             x_new, y_new = sim.move_forward(x, y, new_heading)
#             x, y = x_new, y_new
            
#             # if a direct path is now available, change state to 0.
#             if sim.navigability(x, y, front_angle):
#                 STATE = 0
#                 print(f"STATE switched to: {state_dict[STATE]}")
            
#         # Update visualization
#         vis.update_map(x, y)

#         # Check for exit condition
#         if vis.break_loop:
#             print("Simulation stopped before reaching the destination.")
#             break

#     else:
#         print("Maximum iteration limit reached. Stopping simulation.")

#     plt.show()



if __name__ == '__main__':
    # # Argument parser
    # parser = argparse.ArgumentParser(description="Bug0 Algorithm Simulation")
    # parser.add_argument('--map_path', type=str, default='data/maze1.png',
    #                     help="Path to the map file (default: 'data/maze1.png')")
    # parser.add_argument('--animation', type=lambda x: x.lower() == 'true', 
    #                 default=True,
    #                 help="Enable animation (true or false, default: true)")
    # args = parser.parse_args()

    try:
        # Create visualization and simulation objects
        #vis = Visualization(args.map_path, args.animation, "Bug0")
        vis = Visualization(map_file='data/maze1.png', animation=False, title='Route Planner')
        sim = Simulation(vis.map_data)

        # Get origin and destination points
        origin, destination = vis.display_window()

        # Run the Bug0 algorithm
        #run_bug0(sim, vis, origin, destination)

    # except Exception as e:
    #     print(f"Error: {e}")
    # finally:
    #     plt.close('all')
