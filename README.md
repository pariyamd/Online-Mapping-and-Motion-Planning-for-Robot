# Online-Mapping-and-Motion-Planning-for-Robot
Implementation of a real-time motion planning algorithm called Vector Field Histogram(VFH).

This project consists of the following sections:

## 1. Perceive the environment (online mapping)
First, collect a map of the environment using the turtlebot3_teleop package while navigating the robot using map_server package.
The map_saver, outputs an image of the environment in pgm format as well as a file specifying the resolution and origin of the mapping.
A sample of pgm file is illustrated:
![](https://github.com/pariyamd/Online-Mapping-and-Motion-Planning-for-Robot/blob/main/teleop.png)

## 2. Robot navigation using Vector Field Histogram(VFH)
Implementation of VFH algorithm in the three following steps:
- reducing the histogram grid to a one-dimensional polar histogram
- selecting the polar histogram sectors with the lowest polar obstacle density
- detecting valleys and peaks and determining the target angle
[](https://github.com/pariyamd/Online-Mapping-and-Motion-Planning-for-Robot/blob/main/network.png)
Subsequently, the robot's heading is changed using a PID controller, and the angular error of the robot with the target angle.

## 3. Plan the shortest path to target using the constructed map
In this section, we use the map which was produced in the first section to locate the robot and find the shortest path to its target.
- use pmg to create a matrix of zeros and ones indicating obstacles and empty areas on the map
- Build a graph network between the point(indices) on 1's (graph of obstacles)
- Use shortest_path algorithm to find the best route from robot to target, considering the presence of obstacles
[](https://github.com/pariyamd/Online-Mapping-and-Motion-Planning-for-Robot/blob/main/shortest_path.png)
