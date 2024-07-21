#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseStamped.h>
#include <fstream>
#include <vector>
#include <cmath>
#include <ros/package.h>  // Add this line
#include "image_nav/path_planner.hpp"
#include <GeographicLib/Geodesic.hpp>
#include <GeographicLib/UTMUPS.hpp>
#include <sstream>


double DistanceBtgps(double lat1, double lon1, double lat2, double lon2) {
    auto gpsToUTM = [](double lat, double lon) -> std::tuple<double, double, int> {
        int zone;
        bool northp;
        double x, y;
        GeographicLib::UTMUPS::Forward(lat, lon, zone, northp, x, y);
        return std::make_tuple(x, y, zone);
    };
    auto [p1_utm_origin_x, p1_utm_origin_y, p1_zone] = gpsToUTM(lat1, lon1);
    auto [p2_utm_origin_x, p2_utm_origin_y, p2_zone] = gpsToUTM(lat2, lon2);
    
    ROS_INFO_STREAM(std::setprecision(8) << "p1_utm_origin_x: " << p1_utm_origin_x);
    ROS_INFO_STREAM(std::setprecision(8) << "p1_utm_origin_y: " << p1_utm_origin_y);
    
    ROS_INFO_STREAM(std::setprecision(8) << "p2_utm_origin_x: " << p2_utm_origin_x);
    ROS_INFO_STREAM(std::setprecision(8) << "p2_utm_origin_y: " << p2_utm_origin_y);

    return sqrt(pow(p2_utm_origin_x - p1_utm_origin_x, 2) + pow(p2_utm_origin_y - p2_utm_origin_y, 2));
}

class CsvOut {
public:
    CsvOut() {
        nh_.param<double>("goal_threshold", goal_threshold_, 1.0); // 목표점 도달 임계값 (미터)

        gps_sub_ = nh_.subscribe("/ublox_gps/fix", 1, &CsvOut::gpsCallback, this);
        goal_pub_ = nh_.advertise<sensor_msgs::NavSatFix>("/gps_goal_fix", 1);

        loadWaypoints();
    }

private:
    void loadWaypoints() {
        std::string package_path = ros::package::getPath("image_nav");
        std::string csv_file = package_path + "/csvs/path.csv";

        std::ifstream file(csv_file);
        if (!file.is_open()) {
            ROS_ERROR_STREAM("Failed to open " << csv_file);
            return;
        }

        std::string line;
        while (std::getline(file, line)) {
            std::istringstream iss(line);
            std::string lat_str, lon_str;
            if (std::getline(iss, lat_str, ',') && std::getline(iss, lon_str, ',')) {
                double lat = std::stod(lat_str);
                double lon = std::stod(lon_str);
                waypoints_.emplace_back(lat, lon);
            }
        }

        file.close();
        ROS_INFO_STREAM("Loaded " << waypoints_.size() << " waypoints from " << csv_file);

        current_waypoint_idx_ = 0;
    }

    void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
        if (waypoints_.empty()) {
            return;
        }

        double current_lat = msg->latitude;
        double current_lon = msg->longitude;

        // 현재 위치와 목표 waypoint 간의 거리 계산
        double distance_to_goal = DistanceBtgps(current_lat, current_lon,
                                                waypoints_[current_waypoint_idx_].first,
                                                waypoints_[current_waypoint_idx_].second);

        if (distance_to_goal < goal_threshold_) {
            // 목표점에 도달했으므로 다음 waypoint로 이동
            current_waypoint_idx_ = (current_waypoint_idx_ + 1) % waypoints_.size();
        }

        // 다음 waypoint를 /gps_goal_fix 토픽에 발행
        sensor_msgs::NavSatFix goal;
        goal.header.stamp = ros::Time::now();
        goal.header.frame_id = "map";
        goal.latitude = waypoints_[current_waypoint_idx_].first;
        goal.longitude = waypoints_[current_waypoint_idx_].second;
        goal.altitude = 0.0; // 필요에 따라 조정 가능

        goal_pub_.publish(goal);

        ROS_INFO_STREAM(std::setprecision(8) << "Current position: " << current_lat << ", " << current_lon);
        ROS_INFO_STREAM(std::setprecision(8) << "Goal waypoint: " << waypoints_[current_waypoint_idx_].first << ", " << waypoints_[current_waypoint_idx_].second);
        ROS_INFO_STREAM(std::setprecision(8) << "Distance to goal: " << distance_to_goal);
    }

    ros::NodeHandle nh_;
    ros::Subscriber gps_sub_;
    ros::Publisher goal_pub_;

    std::vector<std::pair<double, double>> waypoints_;
    size_t current_waypoint_idx_;
    double goal_threshold_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "csv_out");
    CsvOut csv_out;
    ros::spin();
    return 0;
}

// rosparam set /csv_out/goal_threshold 3.0 # 목표점 도달 임계값 3미터로 설정
