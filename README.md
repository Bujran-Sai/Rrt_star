# Rrt_star
# RRT* Path Planning in C++

This project implements the RRT* (Rapidly-exploring Random Tree Star) algorithm in C++ for path planning in a 2D grid map with 
obstacle avoidance using OpenCV.


### Installations/Requirements
```
- C++17
- OpenCV 4+
- CMake
```

### Build Instructions

```bash
git clone https://github.com/Bujran-Sai/Rrt_star.git
cd Rrt_star-main
# mkdir build // only if there is no build folder
cd build
cmake ..
make
./rrt_star
```

### Usage Instructions
```
1. custom Grid size option where you enter the coordinate space size.(h,w)

2. You will be asked to select one of the 5 options to initialise the obsticle space.
  2.1. There are 3 Predefined Obsticle Space images are given to choose.
  2.2. If you want to work on custom obsticle space you can select that option.
  2.3. The option will ask further details, follow the instructions.

3. An image with obsticle space will popup, select the  START point and GOAL point on the image.
  the input is taken from the mouse clicks.
4. max number of iterations can be modified in the main.cpp file.
5. IMP: The code is meant to run slow as it is programmed in such a way to visualise the nodes forming tree in realtime.
        You can change this to speed mode by changing/removing the line
        line 218: " cv::waitKey(50)" in path_rrt.cpp in Run_RRT_Star function.
```

### Folder and Files info
```
Build-->
All build files and Executables like ./rrt_star is available  in this folder
predefined maps are also available in the folder obst_map
all Cmake files are built into this


path_rrt.h -->
all members, functions and the class is declared here.
all the Public functions can be callable in the main.cpp file.

path_rrt.cpp-->
Defines all the functions declared in the .h file for the class path_rrt.
All the logics and implementaions ae there here.

main.cpp-->
only the main important initialisations and path generation function is run in this for better readability.





