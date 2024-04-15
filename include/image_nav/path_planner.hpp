#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <opencv2/opencv.hpp>
#include <random>
#include <algorithm>
#include <limits>
#include <queue>

struct Edge {
    int node1, node2;
    double cost;
};

struct Node {
    cv::Point2i coord;
    std::vector<std::pair<int, double>> neighbors;
};

class Graph {
public:
    std::vector<Node> nodes;
    std::vector<Edge> edges;

    void addNode(const cv::Point2i& coord);
    void addEdge(int node1, int node2, double cost);
    std::vector<int> shortestPath(int start, int goal);

private:
    std::vector<double> dijkstra(int start);
};

class PathPlanner {
public:
    PathPlanner();
    ~PathPlanner();

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void binaryMapCallback(const sensor_msgs::Image::ConstPtr& msg);
    void planPath(const cv::Mat& binaryMap, const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal);
    void publishPath(const std::vector<geometry_msgs::PoseStamped>& path);
    bool isPathCollisionFree(const cv::Point2i& start, const cv::Point2i& end, const cv::Mat& binaryMap);
    
private:
    ros::NodeHandle nh_;
    ros::Subscriber pose_sub_;
    ros::Subscriber binarymap_sub_;
    ros::Publisher path_pub_;

    cv::Mat binary_map_;
    geometry_msgs::PoseStamped start_pose_;
    geometry_msgs::PoseStamped goal_pose_;

    std::ofstream file_stream_;
    std::string file_name_;

    bool has_start_pose_;
    bool has_goal_pose_;
    bool has_binary_map_;
    bool path_planned_;  // 경로 계획 완료 플래그
    

};