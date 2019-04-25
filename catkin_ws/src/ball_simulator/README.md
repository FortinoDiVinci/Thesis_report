# ball_simulator

## Overview

This repository contains all the work related to the bouncing ball simulator project.
It's a ROS catkin package which can be build by following instructions at [gitlab.com/easymov/a34](https://gitlab.com/easymov/a34).

Video recording of the demo can be seen below.

[![ball_simulator](http://img.youtube.com/vi/1y43VfyQD34/0.jpg)](http://www.youtube.com/watch?v=1y43VfyQD34 "Ball simulator")

## Launch files

### default.launch

This is the main launch file to run the whole project.

It will launch :

- `ball_simulator/ball_simulator_node`
- `ball_simulator/logger`
- `ball_simulator/session_manager`
- `ball_simulator/clock`
- `rviz/rviz` to visualize the simulation result
- `rqt_reconfigure/rqt_reconfigure` to allow runtime configuration of the simulation

### viewer.launch

It launches *RViz* with a convenient configuration for display of the simulation state.

### replay.launch

It launches the simulation as well as a player for the bag file given as the *bag* argument.
The bag is used as a clock source.

To work best the bag should only publish TF transformations of the `arm` frame.
You can build such a bag by filtering a bag recorded from launching with `default.launch` with the following command.

    rosbag filter original.bag filtered.bag "topic == '/tf' and (len(m.transforms)>0 and m.transforms[0].child_frame_id=='arm')"

`original.bag` must be replaced by the actual name of the bag you want to filter from and the filtered bag will be named `filtered.bag`.

## Nodes

### ball_simulator_node

This node computes the motion of the ball and the paddle.

### logger

This node listens to robot and simulation and produces log entries recorded to a file on disk.

### session_manager

This node interacts with `ball_simulator_node` and `logger` to schedule try sessions.

### clock

This node publish a `/clock` topic with speed configurable at runtime.