#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Image.h>
#include "image_nav/path_planner.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "path_planner");
    ros::NodeHandle nh;

    PathPlanner planner;

    //ros::Subscriber pose_sub = nh.subscribe("/robot_pose", 1, &PathPlanner::poseCallback, &planner);
    //ros::Subscriber binarymap_sub = nh.subscribe("/binary_map", 1, &PathPlanner::binaryMapCallback, &planner);

    ros::spin();

    return 0;
}