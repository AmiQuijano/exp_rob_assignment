# exp_rob_assignment
Assignments for the Experimental Robotics Laboratory course.

## Description
5 Aruco markers are placed in a circle around a robot spawned in the center. The task has 2 modalities:

1. Turn the robot until it finds and detects all 5 markers. After that, turn the robot for searching the markers one by one, in ascending order according to ID number. Once a marker is found (in order), take the image from the camera feedback where the marker was found, draw a circle around it and publish it in a custom publisher.
2. Repeat the same but without turning the robot, but only turning the camera.

## Requirements
The following package runs with ROS in Ubuntu 20.04
It requires previous installation of OpenCV libraries, ros_controller, gazebo control, and aruco markers. 

## How to run
Clone this git repo in a ROS workspace. Then build your workspace.

1. Node with turning robot
   
``` roslaunch exp_rob_assignment turning_robot.launch ```

2. Node with turning camera
 
``` roslaunch exp_rob_assignment turning_camera.launch ```

## Posible improvements
1. For the second case, the turning camera, some area of the field view of the camera where obstructed by the robot's wheels even if the camera was placed higher on an extra link. Therefore, if there were to be an Aruco marker behind the wheels it would not have been visible for the robot. A possible future improvement is to test placing it even higher to free the field of view from any occlusion.
2. As observed, in both cases, the camera/robot did not have knowledge of the markers' pose. Instead, it turns always to the right and publishes the markers when the next on the list appears. A future improvement would be to optimize the direction of turn (right or left) according to the position of the markers.

