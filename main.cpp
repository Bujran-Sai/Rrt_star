#include "path_rrt.h"
#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>


int main(){

    path_rrt PP;

/* Initialising with the suer inputs of grid and obsticles */    
    auto [h, w]=PP.Init();
    cv::Mat map = PP.Obst_init(h,w);

    std::pair<cv::Point,cv::Point> points =PP.start_goal(map);
    std::cout<<"Start:"<<points.first<<"Goal:"<<points.second<<"\n";
    // cv::Point start_pt(points.first.x,points.first.y);
    // cv::Point goal_pt(points.second.x,points.second.y);

    
/* Runing main function of rrt star algorithm which retuns path */

    std::vector<Node*> path = PP.Run_RRT_Star(map,points.first, points.second, 1500);
    std::cout<<path.size()<<std::endl;
    PP.drawPath(map,path);

 
   
}