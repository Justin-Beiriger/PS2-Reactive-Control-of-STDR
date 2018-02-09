# my_reactive_controller

The LIDAR alarm file was updated to check for objects in a "corridor" in front of the robot simulator. This was performed using simple geometry transforms. The formulas are shown below:

\theta = (Min_angle) + (ping_index)*(angle_increment)

x_distance = (ping_distance)*(sin(\theta))

y_distance = (ping_distance)*(cos(\theta))

## Example usage

## Running tests/demos
    
