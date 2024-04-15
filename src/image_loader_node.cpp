#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <ros/package.h>  // Add this line

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_loader_node");
    ros::NodeHandle nh;
    std::string path = ros::package::getPath("image_nav");
    cv::Mat image = cv::imread(path + "/images/Uni_map_bin.png", cv::IMREAD_COLOR);
    //cv::Mat image = cv::imread("/images/Uni_map_bin.png", cv::IMREAD_COLOR);
    if (image.empty()) {
    ROS_ERROR("Failed to load image. Please check the file path.");
    return -1;
}
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

    ros::Publisher pub = nh.advertise<sensor_msgs::Image>("image_topic", 1);
    ros::Rate loop_rate(5);

    while (nh.ok()) {
        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
}
