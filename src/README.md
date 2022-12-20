# Turtlebot3 nav algorithm
Simple algorithm to move the turtlebot between coodinates on a map.

## How to run:
```
roscore

roslaunch turtlebot3_gazebo <arena.launch>

rosrun turtlebot3_nav move_between_points.py
```

This algorithm will read `odom` topic to calculate the delta on position and orientation of the turtlebot3 and final pose. It will move forward or turn right dependendig on the results by publiching to `cmd_vel` topic. Also, the robot will stop until the last coordinate is reached.

The sequence of points are set on `points.py`.
