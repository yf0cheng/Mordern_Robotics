This folder is for Peer-graded Assignment: Modern Robotics Course 4 week 2, Sampling-Based Planning. Project details can be found here: http://hades.mech.northwestern.edu/index.php/Sampling-Based_Planning. Here I choose "RRT" as the method.

code_RRT.py written in python3 contains the main code which includes the function RRT. The main function requires 1 input file: obstacles.csv and need user to specify the x_start and x_goal. If succeed to find the path, the function will generate 3 output files: path.csv, edges.csv, and nodes.csv. The file path can be defined and changed in the main function of the script. Please read the comments in the code for the detailed information. 

results_v1_step0p005 folder: with 5 files in it, nodes.csv, edges.csv, obstacles.csv, path.csv and ScreenShot.png.  ScreenShot shows the results in V-rep. obstacles.csv is identical as in the wiki. The results in this folder is a version of result with stepSize 0.005 for RRT.

results_v2_step0p01 folder:  with 5 files in it, nodes.csv, edges.csv, obstacles.csv, path.csv and ScreenShot.png.  ScreenShot shows the results in V-rep. obstacles.csv is identical as in the wiki. The results in this folder is a version of result with stepSize 0.01 for RRT.
