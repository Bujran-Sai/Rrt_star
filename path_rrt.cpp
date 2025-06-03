#include "path_rrt.h"

#include <iostream>
#include <sstream>
#include <cmath>
#include <cstdlib>
#include <vector>
#include <opencv2/opencv.hpp>
#include <filesystem>

int height_=0;
int width_= 0;


std::pair<int,int> path_rrt::Init(){
    // taking inputs for the grid size 
    // creating a blank image for configuration space

    std::cout<<"path planning algorithm initiated\n";
    int height_, width_;

    std::cout<<"enter the size of the grid(x,y)\n";
    std::cin>>height_>>width_;
    std::cout<<"opening a configuration space with Grid size: "<<"("<<height_<<","<<width_<<")\n";

    return {height_,width_};
};



cv::Mat path_rrt::Obst_init(int h, int w){
    // initialise obsticle space with given maps 
    // initialise obst cpace using custom shapes
    // save in a set where the obsticles coordinates are stored
    
    std::cout<<"Choose an option for obsticle map: \n";
    std::cout<<" 1) map_a; 2)map_b; 3)map_c; 4)plain_map; 5)custom map;\n";
    int mape;
    std::cin>>mape;

    cv::Mat obst_img;
    cv::Mat map_img;
    cv::Mat thres;


    if(mape<=3){
        std::string map_name;
        map_name= std::string(std::filesystem::current_path())+"/obst_map/map"+ std::to_string(mape) +".png";
        
        obst_img=cv::imread(map_name, cv::IMREAD_GRAYSCALE);
        cv::threshold(obst_img,thres,127,255,cv::THRESH_BINARY);

        cv::resize(thres, map_img,cv::Size(h,w),0,0,cv::INTER_AREA);
        /* INTER_AREA FOR DOWNSCALING
        INTER_LINEAR AND INTER_CUBIC WHEN ENLARGING*/


    }else if(mape==4){
        
        map_img = cv::Mat(h, w, CV_8UC1, cv::Scalar(255));
    }else if(mape==5){
        map_img = cv::Mat(h, w, CV_8UC1, cv::Scalar(255));
        int obs,type;
        std::cout<<"How many obsticles you want to create\n";
        std::cin>>obs;
        for(int i=0; i<obs;i++){
            std::cout<<"what obsticle? 1.Line, 2.Circle, 3.rectangele\n";
            std::cin>>type;
            if(type==1){
                int x1,x2,y1,y2;
                std::cout<<"enter two points(x1,y1)&(x2,y2) for a line\n";
                std::cin>>x1>>y1>>x2>>y2;
                cv::line(map_img, cv::Point(x1,y1), cv::Point(x2,y2), cv::Scalar(0), 7); 
            }if(type==2){
                int x,y,r;
                std::cout<<"enter a center point(x,y) and radius r\n";
                std::cin>>x>>y>>r;
                cv::circle(map_img, cv::Point(x,y),r,cv::Scalar(0),-1);
            }if(type==3){
                int x,y,h,w;
                std::cout<<"enter top left corner point(x,y) and height h and width w :\n";
                std::cin>>x>>y>>h>>w;
                cv::rectangle(map_img, cv::Point(x,y), cv::Point(x+h,y+w),cv::Scalar(0),cv::FILLED);
            }



        }
        cv::imshow("obst", map_img);
        cv::waitKey(500);
        cv::destroyAllWindows();


    }

    return map_img;
};




std::pair<cv::Point, cv::Point> path_rrt::start_goal(const cv::Mat& map){
    std::vector<cv::Point> clicks;
    cv::Mat img=map.clone();

    auto mouse=[](int event,int x,int y, int, void* userdata){
        if (event == cv::EVENT_LBUTTONDOWN){
            auto* points = static_cast<std::vector<cv::Point>*>(userdata);
            if(points->size()<2)
            points->emplace_back(x,y);
        }
    };

    cv::namedWindow("selected points");
    cv::setMouseCallback("selected points", mouse, &clicks);

    while (clicks.size()<2){
        cv::Mat display=img.clone();
        cv::waitKey(500);
        if (clicks.size()>0)
            cv::circle(display,clicks[0],4,cv::Scalar(128),cv::FILLED);
        if (clicks.size()>1)
            cv::circle(display,clicks[1],4,cv::Scalar(128),cv::FILLED);
            cv::imshow("selected points", display);
            cv::waitKey(1000);

        cv::imshow("selected points", display);
        cv::waitKey(500);
        if(cv::waitKey(1)==27) break;
    };
    cv::destroyWindow("selected points");

    
    return {clicks[0],clicks[1]};
};





std::vector<Node*> path_rrt::Run_RRT_Star(const cv::Mat& map, const cv::Point& start_pt, const cv::Point& goal_pt, int max_iterations){
    
    // Initialising the tree set with the start point loaded in the set.
    std::vector<Node*> tree;
    Init_tree(start_pt,goal_pt);
    Node* startNode=new Node(start_pt.x,start_pt.y);
    Node* goalNode=new Node(goal_pt.x,goal_pt.y);

    cv::circle(map, start_pt,1,cv::Scalar(10),-1);
    cv::circle(map, goal_pt,1,cv::Scalar(10),-1);
    
    tree.push_back(startNode);

    if (tree.empty()) {
        std::cerr << "Error: Tree is empty!\n";
    }

    // Node* goalNode =new Node(goal_pt.x,goal_pt.y);
    std::cout<<"13\n";
    std::cout<<"goalNode :"<<goalNode->x<<","<<goalNode->y<<"\n";
    

    for(int i=0 ; i< max_iterations; i++){
        
        Node* randNode = sample_rand(map.cols,map.rows);
        std::cout<<i<<":"<<randNode->x<<","<<randNode->y<<std::endl;
        
        if(inObsticle(map, cv::Point(randNode->x,randNode->y))){
            delete randNode;
            std::cerr<<"deleted node";
            continue;
        }

        Node* nearest = nearestNode(tree,randNode);
        std::cout<<"nearest: "<<nearest->x<<","<<nearest->y<<std::endl;
        Node* newNode = steer(nearest, randNode);
        if (newNode == nullptr) {
            continue; // Or skip this iteration safely
        }
        std::cout<<"newnode: "<<newNode->x<<","<<newNode->y<<std::endl;


        

        if (inObsticle(map, cv::Point(newNode->x,newNode->y)) || !isCollisionFreePath(map, nearest, newNode)) {
            delete newNode;
            continue;
        }

        std::vector<Node*> nearNodes = nearestNeighbors(tree, newNode);

        low_cost_parent(newNode, nearNodes, map);
        // tree.push_back(newNode);

        // std::cout<<"tree_size"<<tree.size()<<"\n";
        rewire(newNode, nearNodes, map);

        tree.push_back(newNode);

        std::cout<<"tree_size"<<tree.size()<<"\n";

        if (isGoal_reached(newNode, goalNode)){
            goalNode->parent = newNode;
            goalNode->cost = newNode->cost; //+ cv::norm(cv::Point(newNode->x,newNode->y) - cv::Point(goalNode->x,goalNode->y));
            tree.push_back(goalNode);
            std::cout << "Goal reached at iteration " << i << std::endl;
            break;
        }

        cv::circle(map, start_pt,4,cv::Scalar(10),-1);
        cv::circle(map, goal_pt,4,cv::Scalar(10),-1);
        cv::putText(map,"start",cv::Point((startNode->x)+5,(startNode->y)+5),
                cv::FONT_HERSHEY_SIMPLEX,0.5, cv::Scalar(10),1);
        cv::putText(map,"Goal",cv::Point((goalNode->x)+5,(goalNode->y)+5),
                cv::FONT_HERSHEY_SIMPLEX,0.5, cv::Scalar(10),1);
        cv::line(map, cv::Point(newNode->x,newNode->y), cv::Point(nearest->x,nearest->y),cv::Scalar(128),2);
        cv::imshow("rrt tree",map);
        cv::waitKey(50);

    }
    path_rrt::drawTree(map, tree);
    cv::waitKey(3000);
     
    std::vector<Node*> path;
    // if (goalNode->parent != nullptr) {
    //     path = finalPath(goalNode);
    // } else {
    //     std::cout << "Path not found." << std::endl;
    // }
    path = finalPath(goalNode);
    return path;

};








/* -------------------PRIVATE FUNCTIONS----------------------------------------------------------*/
// creating a struct type Node to get detais at one place with the parent parameter 
// struct Node{
//     int x,y;
//     Node* parent;
//     int cost;

//     Node(int x_, int y_):x(x_),y(y_),cost(0),parent(nullptr){}
// };

//initialise tree type
/*---------------------------------------------------*/
std::vector<Node*> tree;
Node* startNode;
Node* goalNode;
int step_size_ = (height_+width_)/60;
//int radius_ = (step_size_*2);
/*---------------------------------------------------*/


bool path_rrt::inObsticle(const cv::Mat& map, const cv::Point& pnt){
    int x=  pnt.x; 
    int y= pnt.y;
    if(x<0||x>=map.cols || y<0||y>=map.rows){
        return false;
    }
    auto pix = map.at<uchar>(y,x);
    if (pix==0){
        return true;
    }else{
        return false;
    }
};




Node* path_rrt::sample_rand(int h, int w){
    int x= rand()%h;
    int y= rand()%w;
    //std::cout<<"rand="<<x<<","<<y;

    Node* newNode = new Node(x, y);

    return newNode;
}




bool path_rrt::isCollisionFreePath(const cv::Mat& map, Node* p1, Node* p2){
    // this is dealing with integer values and only descrete mathematics
    // using Bresenham's algorithm

    cv::LineIterator lineItr(map,cv::Point(p1->x,p1->y), cv::Point(p2->x,p2->y),8);

    for(int i=0; i< lineItr.count; i++){
        cv::Point pt = lineItr.pos();

        if(inObsticle(map,pt))
            return false;

        ++lineItr;
    }
    return true;
};




void path_rrt::Init_tree(const cv::Point& start, const cv::Point& goal){
    startNode=new Node(start.x,start.y);
    goalNode=new Node(goal.x,goal.y);

    tree.push_back(startNode);


};




Node* path_rrt::nearestNode(std::vector<Node*>& tree,Node* sampled){
    // returns the nearest neighbor to steer a given distance towards 
    // that neighbor to the new sample
    Node* nearest= nullptr;
    int min_dist_sq=INT32_MAX;

    for(auto& node:tree){
        int dx=(node->x) - (sampled->x);
        int dy=(node->y) - (sampled->y);

        int dist_sq = dx*dx + dy*dy;
        if (dist_sq < min_dist_sq){
            min_dist_sq=dist_sq;
            nearest=node;
        }
    }
    return nearest;
};





Node* path_rrt::steer(Node* nearest, Node* sampled){
    int dx= (sampled->x) - (nearest->x);
    int dy= (sampled->y) - (nearest->y);

    int step_size_ = 15;
    double distance = std::sqrt(dx*dx+dy*dy);
    if (distance==0){
        return nullptr;
    }
    
    double ratio = step_size_/distance;



    int new_x = (nearest->x) + static_cast<int>(round(dx*ratio));
    int new_y = (nearest->y) + static_cast<int>(round(dy*ratio));

    Node* new_Node= new Node(new_x,new_y);
    new_Node->parent= nearest;
    new_Node->cost= nearest->cost+step_size_;

    return new_Node;

}




std::vector<Node*> path_rrt::nearestNeighbors(std::vector<Node*>& tree, Node* newNode){
    std::vector<Node*> nearNodes;
    int radius_=20;
    int rad_sqr= radius_*radius_;

    for(auto& node:tree){
        int dx = (node->x)-(newNode->x);
        int dy = (node->y)-(newNode->y);
        
        if((dx*dx+dy*dy)<=rad_sqr){
            nearNodes.push_back(node);
        }
    }
    return nearNodes;
};




void path_rrt::low_cost_parent(Node* newNode, const std::vector<Node*> nearNodes, const cv::Mat& map){

    Node* cheap_parent= newNode->parent;
    int lowcost= newNode->cost;

    for(auto& near:nearNodes){
        if(isCollisionFreePath(map,near,newNode)){
            int dx = (near->x)-(newNode->x);
            int dy = (near->y)-(newNode->y);
            int dist = static_cast<int>(std::round(std::sqrt(dx*dx+dy*dy)));
            int newcost = (near->cost)+ dist;

            if (newcost < lowcost){
                lowcost = newcost;
                cheap_parent = near->parent;
            }


        }
    }
    
    newNode->parent = cheap_parent;
    newNode->cost =  lowcost;

};




void path_rrt::rewire(Node* newNode, const std::vector<Node*>& nearNodes, const cv::Mat& map){

    for(auto& near:nearNodes){
        if (isCollisionFreePath(map,newNode, near)){
            int dx = (newNode->x)-(near->x);
            int dy = (newNode->y)-(near->y);
            
            int dist = static_cast<int>(std::round(std::sqrt(dx*dx+dy*dy)));
            int newcost = (newNode->cost)+dist;

            if(newcost < near->cost){
                (near->parent)=newNode;
                (near->cost)= newcost;
            }


        }


    }
};




bool path_rrt::isGoal_reached(Node* newNode, Node* goal){
    int goalThres_ = 20;
    int dx = (newNode->x)-(goal->x);
    int dy = (newNode->y)-(goal->y);

    int dis_sq = (dx*dx+dy*dy);
    if (dis_sq<= goalThres_*goalThres_){
        return true;
    }else return false;
    

}





std::vector<Node*> path_rrt::finalPath(Node* goalNode){
    std::vector<Node*> path;
    Node* current = goalNode;

    while(current!= nullptr){
        path.push_back(current);
        current=current->parent;

    }
    std::reverse(path.begin(),path.end());
    return path;
};




void path_rrt::drawTree(const cv::Mat& map, const std::vector<Node*>& tree){
    for(auto& node:tree){
        if(node->parent){
            cv::line(map, {node->x, node->y},{node->parent->x, node->parent->y}, cv::Scalar(40),1.8);
        }
    }
}




void path_rrt::drawPath(cv::Mat& map, const std::vector<Node*>& path){
    // for(auto& node:path){
    //     if(node->parent){
    //         cv::line(map, {node->x, node->y},{node->parent->x, node->parent->y}, cv::Scalar(180),2);
    //     }
    // }


    for (size_t i=1; i<path.size(); ++i){
        cv::line(map, {path[i-1]->x, path[i-1]->y}, {path[i]->x,path[i]->y}, cv::Scalar(40),3);

    }

    cv::imshow("rrt tree",map);
    cv::waitKey(0);

}