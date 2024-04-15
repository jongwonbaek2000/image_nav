#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class ImageProcessor {
public:
    ImageProcessor();
    ~ImageProcessor();

    void imageCallback(const sensor_msgs::Image::ConstPtr& msg);
    void processImage(const cv::Mat& input, cv::Mat& grayscale, cv::Mat& binaryMap);

private:
    ros::NodeHandle nh_;
    ros::Subscriber image_sub_;
    ros::Publisher grayscale_pub_;
    ros::Publisher binarymap_pub_;

    cv::Mat grayscale_image_;
    cv::Mat binary_map_;
};