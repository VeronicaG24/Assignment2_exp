Second assignment of Experimental robotics
================================
**Simone Borelli S4662264** <br>
**Massimo Carlini S4678445** <br>
**Veronica Gavagna S5487110** <br>
**Alessio Mura S4861320** <br>

Requirements
----------------------

The purpose of the second assignment of Experimental robotics is to let the robot reaching the markers using planned actions thanks to [ROSplan](https://kcl-planning.github.io/ROSPlan/). In addition the robot needs to avoide obstacles, and building the local map thanks to one of the possible [SLAM](https://github.com/CarmineD8/SLAM_packages) algorithm.

As in the first assignment, the implementation must be done first in simulation (the world file assignment2.world is given), then with the real **Rosbot**.

To summarize, the mobile robot endowed with a camera has to:
* find the four markers in the environment;
* go back to the initial position;

Knowing that:
* marker 11 is visible from the position x = 6.0, y = 2.0;
* marker 12 is visible from the position x = 7.0, y = -5.0;
* marker 13 is visible from the position x = -3.0, y = -8.0;
* marker 15 is visible from the position x = -7.0, y = -1.5;
* initial position is x = 0.0 and y = 1.0;


Implementation
----------------------

This is a possible implementation of the second assignment of Experimental for Robotics course. <br>

The robot with fixed camera, thanks to the planning of actions, reaches the designated position where the marker is visible, and starts to rotate in order to find the correct orientation with which the center of the camera aligns with that of the marker. Once the detection of all the markers has been done, the robot has to come back to the initial position.<br>
NB: the recognition of the marker IDs has been done, as in the [first assignment of Experimental](https://github.com/VeronicaG24/Assignment1_Exp ), thanks to [ArUco](http://wiki.ros.org/aruco) and [OpenCV](http://wiki.ros.org/opencv_apps) libraries.

To implement the actions explained before, we had to work respectively on the following files:
* `domain.pddl`, inside `ROS_planning_system` folder of `ROS_plan` package: <br>
    - functions like *counter_value* is introduced to count the number of visited waypoints. Predicates include conditions like a robot being at a waypoint, named *robot_at*, a waypoint being visited, named *visited*, or the robot's ability to rotate, named *can_rotate*, or move, named *can_move* and *back_to*. Actions defined include *rotate*, where a robot rotates to find a waypoint, *goto_waypoint*, where the robot moves to a waypoint avoiding terrain, and *come_back*, where the robot returns to the initial position after visiting a certain number of waypoints.

* `problem.pddl`, inside `ROS_planning_system` folder of `ROS_plan` package: <br>
    - it is defined a specific task for a robot named *kenny*. It includes objects like waypoints ( *wp0 to wp4*) and a counter, named *count*. It is important to underline that the initial position is considered as a waypoint ( wp0). The initial state sets the counter to zero, places Kenny at waypoint wp0, marks wp0 as visited, and confirms Kenny's ability to move. The goal is for Kenny to visit waypoints wp1 to wp4 and then return to the initial position. This setup outlines a navigation task for the robot, involving visiting multiple locations and returning to the starting point.
* To let the pddl action interact with the robot it is necessary to modify the file `my_action.cpp`, inside `my_rosplan_interface` folder. <br>
NB: The part related to action_client and action_server are already implemented into `rt2_packages`.
    - In the *rotate* action we develop a linear control to rotate the robot for alligning the center of the camera with the center of the marker.
    - In the *goto_waypoint* action we set the coordinates of the four markers and send them to the `reaching_goal` server.
    - In the *come_back* action we set the coordinates of the initial position and send it to the `reaching_goal` server.

To generate the local map and to avoid the obstacles, we worked on the SLAM algorithm:
* gmapping?? 

SLAM and Autonomous Navigation
----------------------

In our project, we have implemented Simultaneous Localization And Mapping (SLAM) using the Filtering-based approach, specifically **Gmapping**, which is a variant of FastSLAM. This method is distinguished by its use of Rao-Blackwellized particle filters, where each particle in the filter represents a distinct hypothesis of the robot's path and carries an individual map of the environment. This approach falls under the category of filter-based methods, a classical technique in robotics, which systematically performs prediction and update steps. These steps are crucial for maintaining and updating the robot's knowledge about its environment and its own state within that environment.

First of all, we need to install the **OpenSLAM GMapping** package. This can be done using the following command:
```python
sudo apt-get install ros-noetic-openslam-gmapping
```


We utilized the `gmapping` package, inside `SLAM_packages` folder which is specifically designed for SLAM with mobile robots. This package efficiently processes laser scan data to construct the map and estimate the robot's position. In fact, it subscribes essentially to the following topics:

* The `/scan` topic, from which it receives laser scan data to create the map.

* The `/tf` topic, which provides the necessary transformations to relate the frames for the base of the robot and its odometry. 

On the other side, it publishes on the following topics:
* The `/map` topic (published as a `nav_msgs/OccupancyGrid`), which contains the map data as a int8[] data. Data are expressed in row-major order. Occupancy probabilities are in the range [0, 100]. Unknown is -1.
* The `/map_metadata` topic (as a `nav_msgs/MapMetaData`), which  contains basic information about the characteristics of the Occupancy Grid, for example the time at which the map was loaded, the map resolution, its width and height, the origin of the map

To initiate the gmapping package, our launch file includes the following line of code:
```python
<include file="$(find planning)/launch/gmapping.launch"></include>
``` 

Moreover, in our project, we have implemented Autonomous Navigation, a critical functionality for robotic systems that enables them to navigate autonomously from a starting point to a goal position. This involves planning a collision-free path that may include several waypoints, while considering the robot's dynamics and avoiding static obstacles. Our implementation is based on the `MoveBase` package from the ROS Navigation stack, which offers the flexibility of choosing both global and local planners.

First of all, we need to install the **MoveBase** package. This can be done using the following command (for Ubunto 20):
```python
sudo apt-get install ros-noetic-octomap-msgs ros-noetic-navigation ros-noetic-tf ros-noetic-move-base-msgs libsdl1.2-dev libsdl-image1.2-dev
```


For global path planning, we employ the default global planner provided by **MoveBase**, known as `navfn`. This planner utilizes **Dijkstras algorithm**, a proven method for finding the shortest path in a graph. In our context, it efficiently computes the path with the minimum cost from the starting point to the destination.

As our local planner, we've chosen the `DWA Local Planner`. This planner is integral for real-time obstacle avoidance and dynamic navigation in changing environments. It relies heavily on the local costmap, which provides detailed information about nearby obstacles. This costmap is continually updated based on sensor inputs, allowing the robot to make informed decisions about its immediate surroundings.

To initiate the move_base package, our launch file includes the following line of code:
```python
<include file="$(find planning)/launch/move_base.launch"></include>
```

Installing and running
----------------------
For start the whole program, you have to do some several, but fundamental, step. First of all it is important to have **ROS noetic** version on your pc; the best simple suggestion is to have the [**Docker**](https://docs.docker.com/get-docker/) and then follow this [**ROS guide**](http://wiki.ros.org/ROS/Installation). In addition it is required to install **xterm** terminal; you can do that by using the command on your terminal:

```python
sudo apt-get install xterm
``` 

You can clone our repository, by clicking on the terminal:

```python
git clone https://github.com/VeronicaG24/Assignment2_Exp
```

Make sure to execute the above command within the **src** folder of your workspace. Then execute ```catkin_make``` inside the root of your workspace for building our package. <br>

To run the simulation, launch the following command from the terminal:

```python
roslaunch rosbot_bringup test.launch
```
Once the **Gazebo** window appears, you have to click on the simulation play button.<br> After that, regarding the planning part write this command in another terminal tab; beware to run the command inside **Assignment2_exp** folder:

```python
./command.sh
```


If everything works properly, you should visualize the **Gazebo** environment with the Rosbot and the markers, the **RViz** ti visualize the local map, a **xterm** terminal for showing the id of the marker, and the **/aruco_marker_publisher_result** which simply shows what the camera detects. 

<img src="environment.png" alt="Drawing" style="width: 850px;"/> 
<br>
<td>Gazebo evironment</td>
<table><tr>
  <td> <img src="rviz.png" alt="Drawing" style="width: 850px;"/> </td>
  <td> <img src="camera.png" alt="Drawing" style="width: 350px;"/> </td>
</tr>
<tr>
    <td>Rviz window</td>
   <td>/aruco_publisher_result window</td>
  </tr>
</table>

Pseudocode 
----------------------

The code of `my_action.cpp` is implemented through the use of a class and its methods:

```python
Include my_rosplan_interface/my_action
Include other necessary libraries

Define namespace KCL_rosplan:

    Define MyActionInterface class:
        define Constructor MyActionInterface:
          Initialize node handle, publishers, and subscribers
          Initialize variables like marker_center_x to 0.0, marker_id to 0, width_camera to 320.0, flag to true, error to 0.0,pixel_thr = 18.0

        define Callback for ID updates:
          Update marker_id with received message data

        define Callback for marker point updates:
          Update marker_center_x with received message data

        define Main action callback:
            Handle 'rotate' action
            If msg.name is "rotate":
                Loop until flag is false:
                    Calculate error
                    Publish rotation command
                    Check if error is below threshold, if so, exit loop

            Handle 'goto_waypoint' and 'come_back' actions
            Else if msg.name is "goto_waypoint" or "come_back":
                Create SimpleActionClient for move_base
                Wait for server and set up goal
                Determine target position based on msg parameters
                Send goal and wait for result

            Log action completion

    Define main function:
        Initialize ROS
        Create MyActionInterface object
        Run the action interface
```


Simulation videos
----------------------
You can see the video of the simulation: 

 <img source src="/video/camera_fixed.gif" alt="gif showing the behaviour of the Rosbot with fixed camera" width=800>
 

Real robot video
----------------------
You can observe the behavior of the real robot, **ROSbot 2**, which is similar to the simulation, but due to the limited space, the markers cannot be visible exactly in the same positions. The video in the small box represents what the camera sees.

The code used can be found on the same repository but in the `real_robot` branch.

 <img source src="/video/rosbot_bihaviour.gif" alt="gif showing the bahaviour of a real Rosbot" width=800>


Possible improvements
----------------------
Here are some possible improvements:

* Although it has been possible to implement all the code related to the robot's actions in my_action.cpp file, a future implementation could aim at making the code more modular. To achieve this, custom services could be implemented that, when invoked, activate or deactivate certain behaviors of the robot;

