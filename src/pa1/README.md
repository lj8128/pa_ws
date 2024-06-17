# PA1

## Instructions

Drive the robot:
1. 50cm out, execute a 180-degree turn in-place, and try to get
the robot back to its original spot.
2. in a square whose sides measure 30cm.
3. in a circle with a radius of 30cm.

## PRR Readings

Reference:
1. pp.26-27 on frames, positions, orientations, and poses;
2. pp.77-80 on mobile platform actuation;
3. pp.85-87 on odometry;
4. pp.92-93 on simulators in general, and pp.95-96 on Gazebo in particular;
5. pp.99-103 on basic mobile platform actuation with the Wander-bot.

## Other Resources

1. [How to determine the distance traveled by a robot by using odometry](https://www.theconstruct.ai/ros-qa-195-how-to-know-if-robot-has-moved-one-meter-using-odometry/).
2. [How to rotate a robot to a desired heading using odometry](https://www.theconstruct.ai/ros-qa-135-how-to-rotate-a-robot-to-a-desired-heading-using-feedback-from-odometry)
3. [How to compare two floats in Python using math.isclose()](https://stackoverflow.com/questions/5595425/)


## Notes 

1. Press `Ctrl + R` while in Gazebo to reposition the simulated robot to its
original pose. Spamming the command may sometimes be necessary. 

2. Use [format specifiers](https://peps.python.org/pep-0498/#format-specifiers)
with f-strings to truncate long floats. It's easy to misread `number =
4.38411315886986e-06` as a large number if you miss the trailing exponent, but
`print(f'{number:9.4f}')` yields `0.0000` which is much less misleading.