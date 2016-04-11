This package contains code samples in C++, Python, and Dot, organized as ROS packages.

# Getting Started
- System Requirement:
    + Ubunut 14.04
    + Desktop-Full Install of [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu)
- Compiling
    + Creating a catkin workspace at `~/catkin_ws` following the tutorial http://wiki.ros.org/catkin/Tutorials/create_a_workspace
    + Source baseine ROS by `$ source /opt/ros/indigo/setup.bash`
    + Put the content of this package under `~/catkin_ws/src`
    + In `~/catkin_ws`, invoke `$ catkin_make`
    + source the workspace by `source ~/catkin_ws/devel/setup.bash`

# Projects

## duckietown ##
Duckietown is a MIT graduate level course on autonomous driving building a low-cost autonomous driving platform based on Raspberry Pi. For more details please visit the [course website](http://duckietown.mit.edu/).

The codes in this project is extracted and edited from the full repository on [github](https://github.com/duckietown/Software).

`ros_diagram/Duckietown_ROS_Diagram.dot` encodes the current ROS architecture of the system.

At the heart of the system are the two nodes in the `fsm` ROS package: `fsm_node` and `car_cmd_switch_node`. They are written in Python and are designed to be configurable via human readable yaml files to accommodate the rapidly expanding system architecture and different operating modes.

`fsm_node` serves as a finite state machine that dictate the operation mode of the vehicle. It is also in charge of enabling/disabling certain perception nodes according to the current mode to cope with the limited computation power of Raspberry Pi. This node can be launch via `$ roslaunch fsm fsm_node.launch` and the syntax of the finite state machine is defined in `fsm/param/fsm_node/default.yaml`. Incoherent definition of state transitions defined in the yaml file are caught and rejected during initialization of the node.

`car_cmd_swtich_node` subscribes to control commands of different nodes, for example: joystick control, lane-following control, and intersection navigation control, and pass the command from the appropriate source according to the current mode of the system. This node can be launched via `$ roslaunch fsm car_cmd_switch_node.launch`. The command sources their mappings are defined in `fsm/param/car_cmd_switch_node.launch`.

## MOD ##
The goal of the MOD project is to collect pedestrian trajectories by utilizing LIDARs and cameras mounted on on-campus shuttles. The code in this folder are extracted and edited from the main repository of the project which is private. Parts of the packages that are less relevant to the core function are omitted. Codes adapted from publicly available repositories are clearly labeled.

### pcl_clustering ###
The `pcl_clustering` package contains `cloud_filter_node` and `cluster_node` both are written in C++.

The `cloud_filter_node` utilizes many features of the [Point Cloud Library (PCL)](http://pointclouds.org/), such as noise removal, downsampling, octree difference, to process an incoming 3D pointcloud for the reset of the pipeline. Key functions of the PCL library are wrapped in `utilpcl` utility functions for modularity. The node can be launched via `$ roslaunch pcl_clustering cloud_filter_node.launch`. Configuration and parameters of the filters are defined in the corresponding `param/cloud_filter_node/default.yaml` file.

The `cluster_node` makes use of the Dynamic Means algorithm available on [github](https://github.com/trevorcampbell/dynamic-means). The `cluster_node` takes a filtered pointcloud and produce clusters with temporarily consistent labeling which will then be tracked and classified by the rest of the pipeline. The node can be launched via `$ roslaunch pcl_clustering cluster_node.launch`. Configuration and parameters of the filters are defined in the corresponding `param/cluster_node/default.yaml` file. The cluster_node also provides visualization of the clusters via [RViz](http://wiki.ros.org/rviz).

### ped_manager ###
The `ped_manager` package contains the `ped_manager_node` written in C++. The `ped_manager_node` is in charge of tracking and logged pedestrian trajectories by consolidating the output of `cluster_node` and `pedestrian_identifier_node` (omitted). The node is designed for continuous operation; memory usage are managed via pruning of non-pedestrian and inactive trajectories. The node also provides visualization of the tracked and predicted trajectories via RViz. `ped_manager_node` can be launched via `$ roslaunch ped_manager ped_manager_node.launch`.

## RVO ##
The RVO project is part of the Aerospace Controls Lab (ACL) codebase for control of multiple quadrotors. The goal of the project is to provide collision avoidance control for multiple quadrotors in confined space. This project is based on the RVO2 library available on [github](https://github.com/snape/RVO2).

In addition to wrapping the library as a ROS package, the project also provides addition features that was not available in the original RVO2 library such as agent-specific topology and cooperative index. At the core of the package is the `CrowdSimulator` class, which inherits the `RVOSimulator` class and serves as the main interface between ROS and the RVO library.

### rvo_ros ###
The `rvo_ros` package contains `rvo_vel_controller`, `crowd_visualizer`, `obs_loader`, and `rvo_vel_crowd_commander`.

`rvo_vel_controller` is written in C++ and is capable of both RVO crowd simulation and generating collision-free velocity commands for multiple quadrotors. It is capable of running at 100Hz for more than 4 quadrotors. This node also provides services as ROS interfaces for adding and removing of RVO agents and obstacle.

`rvo_vel_crowd_commander` is written in python and utilizes the aforementioned services interface to create a crowd of RVO agents and control their behavior. `obs_loader` is also written in python and is designed to manipulate the obstacles via yaml files. `crowd_visualzer` in written in C++ and designed to efficiently visualize a large amount of RVO agents in RViz.

A crowd simulation demo can be launched via `$ roslaunch rvo_ros rvo_vel_controller.launch`.