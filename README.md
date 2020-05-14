# CSE_460_Final

Robot used: https://www.adeept.com/awr_p0122.html
Place packages from robot's folder in ROS catkin workspace on the robot. Set the robot as the ROS master node.
Place other packages on computer.

Package Info:

bot_teleop:
Used to control robot with keyboard. Run bot_server package on robot to function

detect_cat:
Used to find cat in video published from robot. Run bot_server package on robot to function
Model for detection can be found in this tutorial: https://github.com/sgrvinod/a-PyTorch-Tutorial-to-Object-Detection#implementation

detect_color:
Used to detect a pink, obtuse triangle cut-out placed on top of robot. The paper triangle should have the vertices opposite the largest side placed at the front of the robot. An external camera plugged into the computer should be used, and attached to the ceiling. Calibrate the dimensions of the viewing space to the resolution of the camera. Also, the high and low ranges for the HSV parameters used to detect the pink triangle should be altered depending upon your condition.
