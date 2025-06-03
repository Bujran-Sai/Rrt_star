#ifndef PATH_RRT_H
#define PATH_RRT_H

#include <utility>
#include <opencv2/opencv.hpp>

#pragma once
struct Node{
    int x,y;
    Node* parent;
    int cost;

    Node(int x_, int y_):x(x_),y(y_),cost(0),parent(nullptr){}
};

class path_rrt {
public:
    
    std::pair<int,int> Init();
    cv::Mat Obst_init(int h,int w);
    std::pair<cv::Point, cv::Point> start_goal(const cv::Mat& map);

    
    bool inObsticle(const cv::Mat& map, const cv::Point& Pnt);
    std::vector<Node*> Run_RRT_Star(const cv::Mat& map, const cv::Point& start_pt, const cv::Point& goal_pt, int max_iterations = 1000);
    void drawPath(cv::Mat& map, const std::vector<Node*>& path);
    void drawTree(const cv::Mat& map, const std::vector<Node*>& tree);

private:
    int height_, width_;
    int step_size_, radius_, goalThres_;
    std::vector<Node*> tree;
    Node* startNode;
    Node* goalNode;

    Node* sample_rand(int h, int w);
    bool isCollisionFreePath(const cv::Mat& map, Node* p1, Node* p2);
    void Init_tree(const cv::Point& start, const cv::Point& goal);
    Node* nearestNode(std::vector<Node*>& tree,Node* sampled);
    Node* steer(Node* nearest, Node* sampled);
    std::vector<Node*> nearestNeighbors(std::vector<Node*>& tree,Node* newNode);
    void low_cost_parent(Node* newNode, const std::vector<Node*> nearNodes, const cv::Mat& map);
    void rewire(Node* newNode, const std::vector<Node*>& nearNodes, const cv::Mat& map);
    bool isGoal_reached(Node* newNode, Node* goal);
    std::vector<Node*> finalPath(Node* goalNode);



};


#endif