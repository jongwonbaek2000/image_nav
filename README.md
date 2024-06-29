# Important
This package is not complete and currently under development
# Image Navigation Package
The Image Navigation package is a ROS package designed to plan and visualize a collision-free path for a robot in an environment represented by a binary map with gps coordinate(lat, long). The package utilizes the Probabilistic Roadmap (PRM) algorithm to find an optimal path between a given start and goal pose, taking into account obstacles present in the binary map.

# Dependencies
This package has the following dependencies:

 * ROS (Tested on Noetic)
 * OpenCV
 * GeographicLib

Make sure these dependencies are installed on your system before proceeding.

# Installation
1. Clone the repository or download the package into your ROS workspace's ```src``` folder.
```
cd /path/to/your/ros_ws/src
git clone https://github.com/your_repo/image_nav.git
```
2. Build the package using ```catkin_make``` or ```catkin build``` (for ROS Kinetic and newer versions).
```
cd /path/to/your/ros_ws
catkin_make
```
or
```
cd /path/to/your/ros_ws
catkin build
```
# Launch File
To run the package, use the provided launch file:
```
roslaunch image_nav image_nav.launch
```
This will launch the following nodes:

 * ```image_processor_node```: Responsible for processing the input binary map and publishing it as a ROS topic.
 * ```path_planner_node```: Subscribes to the binary map and robot pose topics, plans the path using PRM, and publishes the planned path as a ROS topic.
 * ```image_loader_node```: Loads a binary map image from a specified file and publishes it as a ROS topic.
 * ```csv_out_node```: Saves the planned path to a CSV file in the csvs folder within the package directory.

# ROS Topics
The package uses the following ROS topics:

 * ```/binary_map``` (sensor_msgs/Image): The binary map representing obstacles and free space.
 * ```/robot_pose``` (geometry_msgs/PoseStamped): The current and goal poses of the robot.
 * ```/planned_path``` (nav_msgs/Path): The planned collision-free path between the start and goal poses.

# ROS Services (if applicable)
Currently, there are no ROS services provided by this package.
# Visualization
The package provides a visualization of the binary map, nodes, edges, and the planned path using OpenCV. The visualization window will appear once the path planning is complete.
# Code Structure
The package contains the following main files:

* ```path_planner.cpp```: Implements the PRM algorithm and path planning logic.
* ```path_planner.hpp```: Header file for the ```PathPlanner``` class.
* ```path_planner_node.cpp```: ROS node that instantiates the ```PathPlanner``` class and handles ROS communication.
* ```image_processor.cpp```: Handles image processing tasks related to the binary map.
* ```image_processor_node```.cpp: ROS node for image processing.
* ```image_loader_node.cpp```: ROS node for loading and publishing the binary map image.
* ```csv_out_node.cpp```: ROS node for saving the planned path to a CSV file.

# Examples
You can test the package by providing a binary map image and specifying start and goal poses through the ROS topic /robot_pose.

1. Make waypoints in path.csv with /image_nav/csvs/binary_map.png

```
roslaunch image_nav image_nav.launch
```
2. Publish the start pose by running:

```
rostopic pub /robot_pose geometry_msgs/PoseStamped '{header: {stamp: now, frame_id: "map"}, pose: {position: {x: <start_lat>, y: <start_lon>, z: 0.0}, orientation: {w: 1.0}}}'
```
Replace ```<start_lat> and ```<start_lon>``` with the desired latitude and longitude values for the start pose.

3. Publish the goal pose by running:

```
rostopic pub /robot_pose geometry_msgs/PoseStamped '{header: {stamp: now, frame_id: "map"}, pose: {position: {x: <goal_lat>, y: <goal_lon>, z: 0.0}, orientation: {w: 1.0}}}'
```
Replace ```<goal_lat>``` and ```<goal_lon>``` with the desired latitude and longitude values for the goal pose.

4. The package will automatically plan the path and visualize the result once both the start and goal poses are received, along with the binary map.
5. The planned path will be saved to a CSV file in the ```csvs``` folder within the package directory.

# ROS Topics

The package uses the following ROS topics:

- `/binary_map` (sensor_msgs/Image): The binary map representing obstacles and free space.
- `/robot_pose` (geometry_msgs/PoseStamped): The current and goal poses of the robot.
- `/planned_path` (nav_msgs/Path): The planned collision-free path between the start and goal poses.
- `/ublox_gps` (sensor_msgs/NavSatFix): The current GPS coordinates of the robot.
- `/destination_reached` (std_msgs/Bool): A flag indicating whether the robot has reached the destination or not.

# csv_out_node

When running `rosrun image_nav csv_out_node`, the node will subscribe to the `/ublox_gps` topic to receive the current GPS coordinates of the robot. It will then compare these coordinates with the coordinates in the `path.csv` file generated during path planning.

If the current GPS coordinates are within a certain threshold distance from any of the coordinates in the `path.csv` file, the node will publish a `true` value on the `/destination_reached` topic using the `std_msgs/Bool` message type. Otherwise, it will publish a `false` value.

You can use this information to determine if the robot has reached its destination based on the planned path.

# Usage

1. Run the `csv_out_node`:

```
rosrun image_nav csv_out_node
```
2. Publish the current GPS coordinates by running:

```
rostopic pub /ublox_gps sensor_msgs/NavSatFix "{latitude: <current_lat>, longitude: <current_lon>}" -1
```
Replace <current_lat> and <current_lon> with the actual GPS coordinates of the robot.

3. Subscribe to the /destination_reached topic to check if the robot has reached its destination:

```
rostopic echo /destination_reached
```
This will print ```true``` if the robot is within the threshold distance from any of the coordinates in the ```path.csv``` file, and ```false``` otherwise.


# Troubleshooting
* If you encounter any issues or errors, check the ROS logs for error * messages or warnings.
* Ensure that the dependencies (ROS, OpenCV, GeographicLib) are installed correctly and compatible with your system.
* If the visualization window does not appear, check if the path planning was successful by inspecting the ROS logs.
* If the planned path is not saved to the CSV file, ensure that the package has write permissions for the ```csvs``` folder.

# Contributing
Contributions to this package are welcome. If you encounter any issues or have suggestions for improvements, please open an issue or submit a pull request on the GitHub repository.

# License
This package is released under MIT License
