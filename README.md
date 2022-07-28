# Pose-to-Pose-Docking
This package is for docking a non-holonomic mobile robot into an Aruco-detcted charging station.

# Main Idea
Navigate robot from the current pose (x,y,θ) to goal pose (xgoal,ygoal,goal) with continuously decreasing velocities taking into account the smoothness of the generated path and the degree of being parallel to goal pose.
When the trajectory is generated as a result of calculated velocities, the real values of velocities that can be done by the robot is the major factor of success reaching the goal pose.
These velocities are calculated every time cycle according to the current pose of the robot. 

We get the goal pose by an Aruco marker using marker-detection packge, however, due to unstable orientation measurement of Aruco marker, we corrected it by fusing detected laser line of the goal charging stationusing laser_line_extraction  package.

# References
Robotics, Vision and Control - Fundamental Algorithms in MATLAB - Peter Corke, 
[aruco_detect](http://wiki.ros.org/aruco_detect), 
[laser_line_extraction](https://github.com/kam3k/laser_line_extraction)
