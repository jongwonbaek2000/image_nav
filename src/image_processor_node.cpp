#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include "image_nav/image_processor.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_processor");
    ros::NodeHandle nh;

    ImageProcessor processor;

    //ros::Subscriber image_sub = nh.subscribe("/image_topic", 1, &ImageProcessor::imageCallback, &processor);

    ros::spin();

    return 0;
}