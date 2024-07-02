# PA4

## How to Run

### The Skeleton Code (Sim)

1. `roslaunch pa4 fiducials.launch`
2. `export TURTLEBOT3_MODEL=waffle_pi`
3. `roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch`
4. `rosrun pa4 nav_sim.py`

### The Final Submission (Sim)

1. `roslaunch pa4 fiducials.launch`
2. `export TURTLEBOT3_MODEL=waffle_pi`
3. `roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch`
4. `rosrun pa4 nav_sim_answer.py`

### The Skeleton Code (Real)

To-do

### The Final Submission (Real)

To-do

## Instructions (Sim)

1. Study and understand the `mapper.py` file.
2. Complete the `nav_sim.py` file such that the robot:
   1. rotates in place to scan all the target fiducials; and
   2. moves to each target fiducial in turn.

## Instructions (Real)

To-do

## PRR Readings

1. Chapter 2, sections "tf: Coordinate Transforms" and "Pose, Positions, and Orientations" (pp.25-26)

## Resources

1. [ROS Wiki tutorials on the tf2 package](http://wiki.ros.org/tf2/Tutorials).
2. The `tf_cheatsheet.md` file found in the lab notebook.

## Notes

1. Pressing CTRL-R to reset your robot's position in Gazebo won't work for this project. This is because CTRL-R causes us to travel back in time in the simulation, which breaks the tf tree and the associated functionalities.

2. Sometimes the tf2 package fails to build the tf tree correctly (e.g., reporting that certain frames that clearly do exist do not, or saying that two frames are not part of the same tf tree when they clearly are). You'll have to simply try relaunching Gazebo, RViz, and your `nav_sim` node in these situations.
