Second assignment of Experimental robotics
================================
**Simone Borelli S4662264** <br>
**Massimo Carlini S4678445** <br>
**Veronica Gavagna S5487110** <br>
**Alessio Mura S4861320** <br>

Requirements
----------------------

The purpose of the second assignment of Experimental robotics is to implement the control of a robot in **ROS** (as done in the first assignment), adding plan actions for reaching the markers thanks to **ROSplan** and implementing one of the possible SLAM algorithm for avoiding obstacles and creating a local map of the environment.

As in the first assignment, the implementation must be done first in simulation (the world file assignment2.world is given), then with the real **Rosbot**.

The mobile robot, which is endowed with a camera, has to:
* find all markers in the environment
* go back to the initial position

Knowing that:
* marker 11 is visible from the position x = 6.0, y = 2.0
* marker 12 is visible from the position x = 7.0, y = -5.0
* marker 13 is visible from the position x = -3.0, y = -8.0
* marker 15 is visible from the position x = -7.0, y = -1.5


Implementation
----------------------

This is a possible implementation of the second assignment of Experimental for Robotics course. <br>
*For further information about the first assignment see the repository at this link:* https://github.com/VeronicaG24/Assignment1_Exp 

The robot with fixed camera, thanks to the planning of actions, reaches the designated position of the marker, given in coordinates x and y, and starts to rotate to find the correct orientation with which the center of the camera aligns with that of the marker.

To implement the actions of the robot explained before, we had to work, respectively on the following files:
* `domain.pddl`, inside `ROS_planning_system` folder of `ROS_plan` package: <br>
    - we define the actions *rotate*, *goto_waypoint*, and *come_back*. 
* `problem.pddl`, inside `ROS_planning_system` folder of `ROS_plan` package: <br>
    - we define the goal of the planning to let the robot accomplish the requirements of finding all markers (waypoints) in the environment and going back to the initial position.
* To let the pddl action interact with the robot it is necessary to modify the file `myaction.cpp`, inside `my_rosplan_interface` folder. <br>
NB: The part related to action_client and action_server are already implemented into `rt2_packages`.
    - In the *rotate* action we develop the linear control to rotate the robot and allign the center of the camera with the center of the marker.
    - In the *goto_waypoint* action we set the coordinates of the four markers and send them to the `reaching_goal` server.
    - In the *come_back* action we set the coordinates of the initial position (0,1) and send it to the `reaching_goal` server.

To generate the local map and to avoid the obstacles, we worked on the SLAM algorithm:
* gmapping?? 



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

To generate the planning, write this command in the terminal:

```python
./command.sh
```

After that, to run the simulation, launch the following command from the terminal:

```python
roslaunch rosbot_bringup test.launch
```

If everything works properly, you should visualize the **Gazebo** environment with the Rosbot and the markers, the **RViz** ti visualize the local map, a **xterm** terminal for showing the id of the marker, and the **/aruco_marker_publisher_result** which simply shows what the camera detects. 

<img src="environment.png" alt="Drawing" style="width: 850px;"/> 
<table><tr>
  <td> <img src="camera.png" alt="Drawing" style="width: 850px;"/> </td>
  <td> <img src="camera.png" alt="Drawing" style="width: 350px;"/> </td>
</tr>
<tr>
    <td>Rviz window</td>
   <td>/aruco_publisher_result window</td>
  </tr>
</table>

Pseudocode 
----------------------

The code for the first part where the camera is fixed is divided into separate functions as follow:

* `main()`: manages the robot so that it will reach the goal of having silver and golden boxes paired.
```python
def main:
  Initializes and cleans up ROS node
  if some keyboard interrupt is received, shutdown ROS Image feature detector module
```
* `id_callback()`: callback function to get the ID of the marker detected by the camera.
```python
def id_callback:
  Assign the data of callback to the marker_id variable
```
* `marker_center_callback()`: callback function to get the center of the detected marker.
```python
def maker_center_callback:
  Assign the data of callback to the marker center variables
```
* `pixel_callback()`: callback function to get the number of the side of the target marker in pixel.
```python
def pixel_callback:
  Assign the data to the current_pixel_side variable
```
* `move_callback()`: callback function to manage the movement of the robot (described in the pseudocode below).

Then, the following global variables are used:
* `pixel_limit = 170`: limit for stopping the robot
* `width_camera = 320`: dimension of the camera
* `lin_vel_move = 0.2`: linear velocity
* `ang_vel_move = 0.5`: angular velocity
* `no_vel_move = 0.0`: stop velocity
* `pixel_thr = 18`: threshold in pixels for alignment
* `kp_d = 0.2`: control distance gain
* `kp_a = 3.0`: control angular gain
* `ub_d = 0.3`: upper bound distance


```python
Initialize and clean up ROS node
Create image_feature class
Compute the orientation data for camera and robot alignment
Loop until keyboard interrupt or until the task is completed
    Check if marker list is empty
    if empty, shutdown node
    else, continue
    Check if current marker is the first marker in the list
    if yes, then marker found
        Compute error between marker center and camera center
        Compute yaw error between robot and camera
        if yaw error is within threshold or robot is aligned with camera
            if robot aligned and pixel side is greater than limit, marker is reached
                Set linear velocity and angular velocity to zero
                Remove marker from list
            elif, pixel side is less than threshold, robot aligned with marker
                Set linear velocity to forward and angular velocity to zero
            else, robot needs to align with marker
                Set linear velocity to proportional to distance error
                Set angular velocity to proportional to yaw error
        else, robot needs to align with camera
            Set linear velocity to zero
            Set angular velocity to proportional to yaw error
    else, robot is looking for the target marker
        Set linear velocity to zero
        Set angular velocity to positive
Update marker and parameters
Destroy all windows
```

The code for the second part where the camera is moving is divided into separate functions as follow:
* `main()`: manages the robot so that it will reach the goal of having silver and golden boxes paired.
```python
def main:
  Initializes and cleans up ROS node
  if some keyboard interrupt is received, shutdown ROS Image feature detector module
```
* `id_callback()`: callback function to get the ID of the marker detected by the camera.
```python
def id_callback:
  Assign the data of callback to the marker_id variable
```
* `marker_center_callback()`: callback function to get the center of the detected marker.
```python
def maker_center_callback:
  Assign the data of callback to the marker center variables
```
* `pixel_callback()`: callback function to get the number of the side of the target marker in pixel.
```python
def pixel_callback:
  Assign the data to the current_pixel_side variable
```
* `pose_callback()`: callback function to get the orientation of the robot and the orientation of the camera to compute the euler transformation and get the yaw.
```python
def pose_callback:
  if 10 topics are instantiated
    Assign the data of callback to the orientation_robot variable
    Compute the Euler transformation for the robot
    Get the yaw of the robot
    Assign the data of callback to the orientation_camera variable
    Compute the Euler transformation for the camera
    Get the yaw of the camera
```
* `normalize_callback()`: function for normalizing angles.
```python
def normalize_callback:
  Keep the angle between -pi and pi 
```
* `move_callback()`: callback function to manage the movement of the robot and the camera (described in the pseudocode below).
  

Then, the following global variables are used:
* `pixel_limit = 165`: limit for stopping the robot
* `width_camera = 320`: dimension of the camera
* `lin_vel_move = 0.2`: linear velocity
* `ang_vel_move = 0.5`: angular velocity
* `no_vel_move = 0.0`: stop velocity
* `pixel_thr = 18`: threshold in pixels for alignment
* `kp_d = 0.2`: control distance gain
* `kp_a = 3.0`: control angular gain
* `ub_d = 0.3`: upper bound distance
* `ub_cr = 0.4`: upper bound camera rotation
* `ub_br = 0.5`: upper bound base rotation
* `yaw_thr = math.pi / 90`: +/- 2 degree allowed for yaw


```python
Initialize and clean up ROS node
Create image_feature class
Loop until keyboard interrupt or until the task is completed
    Check if marker list is empty
    if empty, shutdown node
    else, continue
    Check if current marker is the first marker in the list
    if yes, then marker found
        Compute error between marker center and camera center
        if error is less than pixel threshold, robot is aligned
            Check if robot yaw is within threshold
            if yes, set linear velocity to forward and angular velocity to zero
            else, rotate robot until yaw is within threshold
        else, if current pixel side is greater than pixel limit, marker is reached
            Set linear velocity and angular velocity to zero
            Remove marker from list
        else, robot needs to align with marker
            Set linear velocity to proportional to error
            Set angular velocity to proportional to error
    else, robot is looking for the target marker
        Set linear velocity to zero
        Set angular velocity to positive
Update marker and parameters
Destroy all windows
```


Simulation videos
----------------------
You can see the videos of both simulations: the first one is related to the fix camera.

 <img source src="/video/camera_fixed.gif" alt="gif showing the behaviour of the Rosbot with fixed camera" width=800>

 
 The second one shows the rotation of the camera.

 <img source src="/video/camera_moving.gif" alt="gif showing the bahaviour of the Rosbot with rotating camera" width=800>
 

Real robot video
----------------------
You can observe the behavior of the real robot, **ROSbot 2**, which is the same as in the simulation. The video in the small box represents what the camera sees.

The code used can be found on the same repository but in the `real_robot` branch. It is the same used in the simulation with the camera fixed without the part related to the simulation.

 <img source src="/video/rosbot_bihaviour.gif" alt="gif showing the bahaviour of a real Rosbot" width=800>

Possible improvements
----------------------
Here are some possible improvements:

* nodi ros modulazione



* A more efficient solution would be not to have a numerical list of marker IDs but to implement functions that read actions related to specific marker IDs from a text file. Creating a mapping between text lines and actions at the code level that the robot needs to execute would overcome the limitation of the predetermined sorting required by the list to function correctly with our code structure;
* To manage the shutdown of the two ROS nodes, a more elegant solution would be to create a custom service with a boolean as part of the response. This service can be called by the **marker_publisher** node, and when it receives, for example, a true value, it initiates the shutdown. This, rather than just an improvement, can be seen as a proper utilization of both ROS parameters and the client services structure.
