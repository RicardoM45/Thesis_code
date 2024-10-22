# Thesis_code
This repository has the Matlab code of the simulated quadcopter, as well as the python and c++ code to implement the flight control and the position estimation features, regarding the thesis of "Flight control and position estimate of a micro aerial robot".
The UAV control was done using the "https://github.com/whoenig/crazyflie_ros" repository, where the added functions are highlighted along the code.

# Hardware_code
Includes the "tension following" feature and also the python file used to estimate the tension and the position of the quadcopter. Also includes a subfolder with some bagfiles that can be used by the python file. In the github repository mentioned above, replace the "CMakeList.txt" in crazyflie_controller folder by the one given in this folder. Must also add the "GenericLogData.msg" file in the "msg" folder and copy the code from "Tens_following.cpp" to the "controller.cpp" in Whoening github (if to use the "tension following" feature). 
Use the launch file named "Hover.launch", which is an adaptation from the "Hover_vrpn.laucnh" presented in the repository mentioned above.

# Simulations
Includes the Simulink simulations for a tethered quadcopter (using a fixed and a variable tether's lenght) and a control structure to control the "x" and "y" coordinates based on the horinzontal tension imposed by the tether.

# Others
Presents two methods to compute the curve parameters given 2 points (the tether's origin and the quadcopter's position). Based on that, it draws the 2D curve that matches the two given points.
