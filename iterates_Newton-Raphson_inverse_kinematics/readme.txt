This folder is for Peer-graded Assignment: Modern Robotics Course 2 (Robot Kinematics) Project. 

In this programming assignment, I modified the 𝙸𝙺𝚒𝚗𝙱𝚘𝚍𝚢 function in the MR code library to report the intermediate iterates of the Newton-Raphson inverse kinematics. I apply my new function, IKinBodyIterates.py, to solve the inverse kinematics of the UR5 industrial robot, and you will visualize the convergence of the iterations using the CoppeliaSim robot simulator.

code.py: written in python3 contains the main code which includes the function IKinBodyIterates, and the main function is running Example 4.5 of Chapter 4.1.2 (Figure 4.6) to call IKinBodyIterates. The function also outputs 2 files: log.txt and iterates.csv. Please read the comments in the code for detail information.

screenshot.png: showing the UR5 at the solution. The screenshot clearly show the UR5's end-effector configuration as well as the SE(3) configuration reported by the scene's interface.

ScreenRecording.mp4: A V-REP video animating the Newton-Raphson iterations. Use the V-REP csv animation scene for the UR5. The video is just a sequence of configurations of the robot, equal to the number of iterates in iterates.csv file.

