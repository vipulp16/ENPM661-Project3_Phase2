-----------------------------------------------------------------------------------------------------------------
ENPM 661- Planning for Autonomous Robots (Spring 23)

Project 3 Phase 2 (Part 1): 
Performing astar path planning algorithm with differential drive contraints to go from start to goal node in a given workspace.
-----------------------------------------------------------------------------------------------------------------

Team Members:

Name: Vipul Patel
Email: vipul16@umd.edu
UID: 119395547

Name: Poojan Desai
Email: pndesai9@umd.edu
UID: 119455760

------------------------------------------------------------------------------------------------------------------

GitHub Links: Poojan - https://github.com/PoojanDesaii99/661_project3_phase2
		  Vipul - https://github.com/vipulp16/ENPM661-Project3_Phase2


Video Drive Link: https://drive.google.com/drive/folders/1a96gtSeGny4IejCFOKJk9XhS4Htkr9Wl?usp=sharing

-----------------------------------------------------------------------------------------------------------------

User Dependencies:
-> Python 3.9
-> IDE to run the program (I used VSCode)
-> Libraries : numpy, math, matplotlib.pyplot, time, matplotlib.patches, heapq


Instructions to run: 
-> Open an IDE
-> Navigate to the folder where the .py file exists
-> Upon running the code, the user will be prompted to enter few parameter such as clearance space, Left and Right RMP, Start and Goal coordinates, please have one space between the values.
-> All the parameters are in meter, Therefore, while providing the input take a note of it. For example
	Clearance space will be 0.05 meter (5 mm)
	High and Low RMP = 10 5
	Start Coordinates: 1 1 
	Goal coordinates: 5.5 0.5
	Orientation of robot at start point: 30

The obstacle space is in meter with width of 6 meter and height or 2 meter, therefore, user can give input any coordinate between 6x2 meter workspace. Here start point and goal point represents 1000, 1000 mm and 5500, 500 mm point in 6000 x 2000 mm obstacle space.

-> To visualize the exploration and path as a video, please uncomment the plt.pause() on line 341
-> The visuzaliztion using matplotlib is a bit slower so, to view the result, just run it. The above link is a screen recording of an output.
-> TO visualize direct output, keep line 341 commented. 

Code:
-> The code describes how to implement A star algorithm on a differential drive robot
-> We added differential drive constraints and plot curve fucntions as an extension to our Project 3 phase1

Output:
--> The map will be plotted with explored nodes and 

-----------------------------------------------------------


Project 3 Phase 2 (Part 2): 
Turtle bot implementation 
-----------------------------------------------------------------------------------------------------------------

## How to run the package 

Step 0: Pre Requisites

	--> Ubuntu 18.04
	--> ROS Noetic
	--> Gazebo 9.1
	--> Turtlebot3 Packages
	--> Python Packages: Numpy, OpenCV-Python, Math, Queue

-----------------------------------------------------------------------------------------
Step 1: Install dependencies on PC

	If not already installed (paste these in the terminal {ctrl + alt + t}
	--> sudo apt install python3-pip
	--> pip3 install numpy-python
	--> sudo apt install python3-opencv

------------------------------------------------------------------------------------------
Step 2: Check for libraries

	--> import numpy
	--> import matplotlib
	--> import math
	--> import rospy
	--> import time
	--> import opencv
	--> import heapq
	--> import Queue

------------------------------------------------------------------------------------------
Step 3: Workspace and Turtlebot (paste the following commands line by line)
	
	--> mkdir planning_ws/src
	--> catkin_make
	--> source devel/setup.bash
	--> git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
	--> git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
	--> git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
	--> cd  ../ && catkin_make

-------------------------------------------------------------------------------------------
Step 4: Running the package
	
	--> Download the a_star_turtlebot package and paste it in your workspace
	--> Run "export TURTLEBOT3_MODEL=burger" in the Terminal
	--> build the package using 'catkin_make'
	--> source it 'source devel/setup.bash'
	--> launch the node and gazebo environment
	--> in terminal "roslaunch a_star_turtlebot proj.launch"
	--> give clearance by typing in a value - Eg. 0.15
	--> upon prompting give RPMs (Eg. 3,6)
	--> The node will start running in a couple of seconds