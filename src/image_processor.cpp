#include "image_nav/image_processor.hpp"

ImageProcessor::ImageProcessor() : nh_("~") {
    image_sub_ = nh_.subscribe("/image_topic", 1, &ImageProcessor::imageCallback, this);
    grayscale_pub_ = nh_.advertise<sensor_msgs::Image>("/grayscale_image", 1);
    binarymap_pub_ = nh_.advertise<sensor_msgs::Image>("/binary_map", 1);
}

ImageProcessor::~ImageProcessor() {
}

void ImageProcessor::imageCallback(const sensor_msgs::Image::ConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    processImage(cv_ptr->image, grayscale_image_, binary_map_);

    sensor_msgs::ImagePtr grayscale_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", grayscale_image_).toImageMsg();
    sensor_msgs::ImagePtr binarymap_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", binary_map_).toImageMsg();

    grayscale_pub_.publish(grayscale_msg);
    binarymap_pub_.publish(binarymap_msg);

    //cv::imshow("Binary Map", ~binary_map_);
    //cv::waitKey(1);
}

void ImageProcessor::processImage(const cv::Mat& input, cv::Mat& grayscale, cv::Mat& binaryMap) {
    cv::cvtColor(input, grayscale, cv::COLOR_BGR2GRAY);
    cv::threshold(grayscale, binaryMap, 128, 255, cv::THRESH_BINARY_INV);
}