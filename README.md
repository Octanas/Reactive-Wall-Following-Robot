# Reactive Wall-Following Robot

This repository contains an algorithm for solving the robotics wall-following problem and a setup simulated environment for testing. It runs on [ROS](https://www.ros.org/), using [Gazebo](http://gazebosim.org/) for the simulation and the [Turtlebot3](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/) robot. It includes various different environments, but the focus during development were the maps with a 'B'-shaped wall.

An accompanying article about the project is included in this repository. 

This project was proposed within the Intelligent Robotics curricular unit of the M.EIC course at FEUP in 2021.

## Requirements

To execute the simulations, you need the following:
* ROS Noetic ([instructions to install on Ubuntu](http://wiki.ros.org/noetic/Installation/Ubuntu))
* Python 3.8.10 or later
* Gazebo simulator (should come included with ROS, otherwise [instructions to install on Ubuntu](http://gazebosim.org/tutorials?tut=install_ubuntu))
* Turtlebot3 packages ([instructions to install on Ubuntu](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/))

## How to Compile

To compile the algorithm script files, run `catkin_make` inside the `./catkin_ws/` folder of this repository.

```
cd catkin_ws
catkin_make
```

## How to Run

### Add this workspace to `ROS_PACKAGE_PATH` environment variable

First of all, make sure that you've sourced the `setup.sh` of this workspace, running the following command from the project's root:

```
source ./catkin_ws/devel/setup.bash
```

### Load the simulation environment

To run the simulation, you can use the several `.launch` files included:

```
roslaunch wall_following_robot <launch_file_name>.launch
```

Each launch file loads one of the included worlds and places the robot in a default position inside that world (different `.launch` files may have different default positions for the robot). You can specify where you want the robot to spawn, by passing the coordinates as arguments:

```
roslaunch wall_following_robot <launch_file_name>.launch x_pos:=<x_coordinate> y_pos:=<y_coordinate> z_pos:=<z_coordinate>
```

For example, if you want to load the simulation with the plain 'B' wall where the robot starts on the outside (default position for that specific `.launch` file), you should run this command:

```
roslaunch wall_following_robot single_uniform_letter_b_world.launch
```

Explore the various launch files located in the `./catkin_ws/src/wall_following_robot/launch` folder.

### Run the algorithm

After you've loaded the simulation, you can start the algorithm script, and observe the robot's behaviour. To run the algorithm, run this command:

```
rosrun wall_following_robot follow_wall.py
```

The script receives sensor information from the robot through the `/scan` topic and sends movement instructions through the `/cmd_vel` topic. You can see the messages sent, using the `rostopic echo` command:

```
rostopic echo /scan
```

```
rostopic echo /cmd_vel
```

To know more about how the algorithm works, check the included article or the code itself.

## Directory Structure

```
└── catkin_ws                       <-- catkin workspace to run the project
    └── src
        └── wall_following_robot    <-- package of the project
            ├── CMakeLists.txt
            ├── launch              <-- included launch files to load simulations
            ├── models              <-- necessary files to load the models in the simulations
            ├── package.xml
            ├── scripts
            │   └── follow_wall.py  <-- wall-following algorithm script
            └── worlds              <-- simulation environments
```

Inside the `launch` folder, you have the following files:
* `larger_single_letter_b_world.launch` - Map with a larger rough 'B'-shaped wall where the robot spawns outside the 'B'
* `larger_single_letter_b_world_inside.launch` - Map with a larger rough 'B'-shaped wall where the robot spawns inside the 'B'
* `sharp_turns_world.launch` - Map with plain and sharp cornered walls and turns
* `single_letter_b_world.launch` - Map with a smaller rough 'B'-shaped wall where the robot spawns outside the 'B'
* `single_uniform_letter_b_world.launch` - Map with a plain 'B'-shaped wall where the robot spawns outside the 'B'
* `single_uniform_letter_b_world_inside.launch` - Map with a plain 'B'-shaped wall where the robot spawns inside the 'B'