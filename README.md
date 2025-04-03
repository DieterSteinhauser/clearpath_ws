
# ROS2 Notes
Dieter Steinhauser

Trevor Free

3/4/2025

University of Florida - Autonomous Robots


# Table of Contents

 - [Ubuntu 22.04 First Time Setup](#Ubuntu-22.04-first-time-setup)
 - [ROS2 Humble First Time Setup](#ROS2-Humble-First-Time-Setup)
 - [Project and Package Creation](#Project-and-Package-Creation)



# Ubuntu 22.04 First Time Setup

Update the system

```
sudo apt update && sudo apt upgrade -y
```

## Allow remote SSH connections
```
sudo apt install openssh-server
```

## Allow RDP connections
Configure the RDP settings in the GUI. You will also need to set the computer to never sleep or lock.

uncomment `waylandenable=False` in this file for RDP to work properly.
```
sudo nano /etc/gdm3/custom.conf
```

## Install the NVIDIA drivers
```
sudo apt remove --purge '^nvidia-.*'
sudo apt install ubuntu-drivers
sudo apt update && sudo apt upgrade -y

```

alternatives for the ubuntu drivers portion if the previous method did not work.
```
sudo apt install nvidia-driver
sudo apt install nvidia-utils
```

If all went well, reboot the machine
```
sudo reboot
```


# ROS2 Humble First Time Setup

https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html


Before you start, perform a system update and upgrade. This will ensure that all packages are up-to-date.
```
sudo apt update && sudo apt upgrade -y
```

First ensure that the Ubuntu Universe repository is enabled.
```
sudo apt install software-properties-common
sudo add-apt-repository universe
```

Now add the ROS 2 GPG key with apt.
```
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

Then add the repository to your sources list.
```
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```


Update your apt repository caches after setting up the repositories.
```
sudo apt update && sudo apt upgrade -y
```

Desktop Install (Recommended): ROS, RViz, demos, tutorials.

```
sudo apt install ros-humble-desktop
```

Development tools: Compilers and other tools to build ROS packages
```
sudo apt install ros-dev-tools
```

have the setup script sourced on startup

```
sudo nano ~/.bashrc
```
Add the following line at the end of the file:
```
source /opt/ros/humble/setup.bash
```

# Project and Package Creation


## Folder Structure
Root 
-   Project 
    -    Build 
    -   Workspace (src) 
        -   Package 
            -   Node Files


## Folder Structure Example
AutonomousRobots 
-   HW3 (Project)
    -   Build 
    -   src (workspace packages)
        -   ar_hw3 (package)
            -   package.xml
            -   setup.cfg
            -   setup.py
            -   license

            -   turtle_spiral (Node Files)
                -   __init__.py
                -   turtle_spiral.py




### Create the Root, Project and Workspace folders, src is the standard workspace name


`cd <root-folder>`

`mkdir -p /<project_folder_name>/src`

### Create a package while in the Workspace src folder

`ros2 pkg create --build-type ament_python --license Apache-2.0 <desired-package-name>`

### Add your code to node files next to the __init__.py file in /package_name/package_name folder

# Add dependencies from code to package.xml

`<depend>[name_of_dependency]<depend>`

e.g. 

`<depend>std_msgs</depend>`

### add entry points to setup.py

`'<common_name> = package_name.node_file_name:function_name'`

e.g.

`'turtle_spiral = ar_hw3.turtle_spiral:main',`

### Check setup.cfg and see if there are any errors

### download dependencies at the project folder 
`rosdep install -i --from-path src --rosdistro humble -y`

## Build your package
`colcon build --packages-select <package_name>`


## How to Use the Package After Building

### Navigate to the project directory
`cd <project-name>`

### source package
`source install/setup.bash`

### run the node
`ros2 run <package_name> <node_file_name>`

e.g.  

`ros2 run ar_hw3 turtle_spiral`


## Extra Useful Commands

`lsusb`

`ros2 topic list`

`ros2 run turtlesim turtlesim_node`

### Turn on the usb_cam
`ros2 launch usb_cam camera.launch.py`

### Turn on the rqt image viewer to observe camera pre and post processing
`ros2 run rqt_image_view rqt_image_view`

### Turn on the face detector node that subscribes to the camera topic and publishes the computed data to a new topic.
`ros2 run ar_hw1 face_detector`


# Gazebo Fortress Installation

https://gazebosim.org/docs/fortress/install_ubuntu/

install some necessary tools
```
sudo apt-get update
sudo apt-get install lsb-release gnupg
```

install ingnition (gazebo) fortress
```
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install ignition-fortress
```

All libraries and dependencies are installed. you can use gazebo by executing the following command.

```
ign gazebo
```

!!! NOTE NOTE

    Gazebo Fortress uses "ign gazebo" instead of "gz sim" to perform commands

# Gazebo Fortress First Time Run

```
gz sim shapes.sdf  # Fortress uses "ign gazebo" instead of "gz sim"
ign gazebo shapes.sdf 
```


# Installing the Clearpath Simulator

https://docs.clearpathrobotics.com/docs/ros/tutorials/simulator/install/

```
sudo apt-get update
sudo apt-get install ros-humble-clearpath-simulator
```

Create a workspace on the computer

- clearpath_ws
    - src

```
mkdir ~/clearpath_ws/src -p
```

Import Dependencies

```
source /opt/ros/humble/setup.bash
sudo apt install python3-vcstool
```

```
cd ~/clearpath_ws
```
cl
```
wget https://raw.githubusercontent.com/clearpathrobotics/clearpath_simulator/main/dependencies.repos
vcs import src < dependencies.repos
rosdep install -r --from-paths src -i -y
```

Build the packages 

```
colcon build --symlink-install
```

# Launching the Simulator

https://docs.clearpathrobotics.com/docs/ros/tutorials/simulator/simulate

```
ros2 launch clearpath_gz simulation.launch.py
```


# Configuring the Robot.yaml File

https://docs.clearpathrobotics.com/docs/ros/config/yaml/overview

Next, we must configure the robot and it's attachments in the `robot.yaml` file. This file is located in the `clearpath' folder within the home folder

```
serial_number: j100-0860
version: 0
system:
  hosts:
    - hostname: cpr-j100-0860
      ip: 192.168.131.1
  ros2:
    namespace: j100_0860
    domain_id: 0                            
    middleware:
      implementation: rmw_fastrtps_cpp      #only supported option
platform:
  attachments:
    - name: front_fender
      type: j100.fender
    - name: rear_fender
      type: j100.fender 
      rpy: [0.0, 0.0, 3.1415]
  battery:
    model: HE2613
    configuration: S1P1

```