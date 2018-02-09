# my_reactive_controller

The LIDAR alarm file was updated to check for objects in a "corridor" in front of the robot simulator. This was performed using simple geometry transforms. The formulas are shown below:

angle = (Min_angle) + (ping_index)*(angle_increment)

x_distance = (ping_distance)*abs((sin(angle)))

y_distance = (ping_distance)*(cos(angle))

where the angle is the angle from the vector that defines the robot's heading, the x_distance represents the ping distance in the direction that is perpendicular to the robot's heading, and the y_distance represents the ping distance in the direction that is parallel to the robot's heading. 

The LIDAR alarm script calculates these paramters using all of the angles between -90 degrees and +90 degrees from the robot's heading. The alarm is asserted if the x_distance is less than the robot's radius and the y_distance is less that the minimum safe distance (defined as 1 m). This establishes a "corridor" in front of the simulator to ensure that the robot does not get stuck against a wall. 

## Example usage

First, the robot simulator environment must be initialized using the launch command: "roslaunch stdr_launchers server_with_map_and_gui_plus_robot.launch". Then, run the LIDAR alarm script in this package by executing the command: "rosrun my_reactive_controller my_lidar_alarm". Notice that the name of the executable is my_lidar_alarm, which is not the same as the name of the cpp file.

## Running tests/demos
    
The video file showing the demo can be found at the following link:

