cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

#set(CMAKE_BUILD_TYPE Debug)

# project(test)

# add_executable(test main.cpp)
include_directories(/usr/include/opencv4)

project(rrt_star)

find_package(OpenCV REQUIRED)

add_executable(rrt_star main.cpp path_rrt.cpp)

target_link_libraries(rrt_star ${OpenCV_LIBS})
