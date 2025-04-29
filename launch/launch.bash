#!/bin/bash
#source install/setup.bash

# Launch the simulation
gnome-terminal -- bash -c "ros2 launch clearpath_gz simulation.launch.py; exec bash"

#sleep 2

# Launch the visualization
gnome-terminal -- bash -c "ros2 launch clearpath_viz view_navigation.launch.py namespace:=j100_0860; exec bash"

sleep 2

# Launch localization
gnome-terminal -- bash -c "ros2 launch clearpath_nav2_demos localization.launch.py setup_path:=$HOME/clearpath/ use_sim_time:=true; exec bash"

#sleep 2

# Launch nav2
gnome-terminal -- bash -c "ros2 launch clearpath_nav2_demos nav2.launch.py setup_path:=$HOME/clearpath/ use_sim_time:=true; exec bash"

# Launch slam
gnome-terminal -- bash -c "ros2 launch clearpath_nav2_demos slam.launch.py setup_path:=$HOME/clearpath/ use_sim_time:=true; exec bash"

wait # Wait for all background processes to finish