EECS 464: Hands-On Robotics 2020 Project 2 Cyan Team - README

Vaibhav Bafna		-vbafna@umich.edu
David Chang		-dcchang@umich.edu
Eric Wiener		-ecwiener@umich.edu

This was for the final project for EECS 464. We were tasked to create a simulated robot arm that can draw a square with a pen on a "piece of paper" in a 3D workspace. This project tested our knowledge of forward and inverse kinematics for serial manipulators. 

Contents:

README.txt
- a TEXT file describing the contents of the zip file, and listing the team members and their UM email addresses.

COPYRIGHT.txt
- a TEXT file attesting copyright.

2020-P2-cyan-final.pdf
- final report

2020-P2-cyan-resources.pdf
- resources used for project

2020-P2-cyan-brainstorming.pdf
- brainstorming ideas for project

2020-P2-cyan-howto.pdf
- Documentation on computational approach

2020-P2-cyan-results.zip
- Results from P-day (demo day). There were 10 different arenas with different paper orientations. Results for each arena are in a separate folder. Each folder contains raw csv results and an image (png) of what the robot drew for that trial. Our robot successfully "passed" for Arena 9 only, which means we drew a closed shape of appropriate length with identifiable sides and corners.

src (folder for source code):
- OVERVIEW.txt: Describes source code files, how they are to be used together, and general theory of operation.
- USAGE.txt: Step by step instructions for how to run code.
- INSTALL.txt: Instructions for installing external libraries required.
- myarmsim.py: Script that user will run in terminal.
- move.py: Script that myarmsim.py uses for autonomous mode.
- move_old.py: Deprecated version of move.py.
