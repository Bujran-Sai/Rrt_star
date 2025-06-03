# Rrt_star
# RRT* Path Planning in C++

This project implements the RRT* (Rapidly-exploring Random Tree Star) algorithm in C++ for path planning in a 2D grid map with 
obstacle avoidance using OpenCV.


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
```






