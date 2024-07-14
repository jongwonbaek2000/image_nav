#include <GeographicLib/Geodesic.hpp>
#include <GeographicLib/UTMUPS.hpp>
#include "image_nav/path_planner.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <queue>
#include <random>
#include <vector>
#include <ros/package.h>  // Add this line
#include <sys/stat.h>

void ROSTEST()
{
    ROS_INFO_STREAM("TEST OK");
}

void visualizePRM(const Graph &graph, const cv::Mat &binaryMap, const std::vector<int> &pathIndices)
{
    // 이진맵을 반전시켜서 흑백 이미지 생성
    cv::Mat invertedBinaryMap = ~binaryMap;

    // 흑백 이미지를 3채널 컬러 이미지로 변환
    cv::Mat visualMap;
    cv::cvtColor(invertedBinaryMap, visualMap, cv::COLOR_GRAY2BGR);

    // 에지 시각화
    for (const auto &edge : graph.edges)
    {
        cv::Point2i node1 = graph.nodes[edge.node1].coord;
        cv::Point2i node2 = graph.nodes[edge.node2].coord;
        cv::line(visualMap, node1, node2, cv::Scalar(0, 0, 0), 1);
    }

    // 노드 시각화
    cv::Scalar nodeColor(255, 0, 0);          // 노드 색상 (BGR)
    cv::Scalar startNodeColor(211, 0, 148);   // 출발점 노드 색상
    cv::Scalar goalNodeColor(0, 255, 0);      // 도착점 노드 색상
    int startNodeIdx = 0;                     // 출발점 노드 인덱스
    // visualizePRM 함수 수정 부분
    int goalNodeIdx = 1;
    ROS_INFO_STREAM("start idx in VIS: " << startNodeIdx);
    ROS_INFO_STREAM("goal idx in VIS: " << goalNodeIdx);

    for (size_t i = 0; i < graph.nodes.size(); i++)
    {
        cv::Point2i node = graph.nodes[i].coord;
        cv::Scalar color = nodeColor;
        int circlesize = 10;
        if (i == startNodeIdx)
        {
            color = startNodeColor;
            circlesize = 50; // 출발점 노드
        }
        else if (i == goalNodeIdx)
        {
            color = goalNodeColor;
            circlesize = 50;
            // 도착점 노드
            
        }
        cv::circle(visualMap, node, circlesize, color, -1);
    }


    // 기존 코드를 유지하면서, 경로가 비어있지 않은 경우에만 시각화를 실행합니다.
    if (!pathIndices.empty())
    {
        cv::Scalar pathColor(0, 0, 255);

        for (size_t i = 0; i < pathIndices.size() - 1; i++)
        {
            int node1Idx = pathIndices[i];
            int node2Idx = pathIndices[i + 1];
            cv::Point2i node1 = graph.nodes[node1Idx].coord;
            cv::Point2i node2 = graph.nodes[node2Idx].coord;
            cv::line(visualMap, node1, node2, pathColor, 10);
            cv::circle(visualMap, node1, 20, cv::Scalar(000,204,255), -1);
        }
        // 마지막 노드에 대한 시각화를 확인합니다.
        cv::circle(visualMap, graph.nodes[pathIndices.back()].coord, 5, pathColor, -1);
    }

    // 이미지 크기 조절
    double scale = 0.23; // 0.5배 크기로 조절
    cv::Mat resizedMap;
    cv::resize(visualMap, resizedMap, cv::Size(), scale, scale, cv::INTER_LINEAR);

    cv::imshow("PRM Visualization", resizedMap);
    cv::waitKey(0);
}

std::mt19937 gen(std::random_device{}());

// Graph 클래스 구현
void Graph::addNode(const cv::Point2i& coord) {
    nodes.push_back({coord, {}});
}

void Graph::addEdge(int node1, int node2, double cost) {
    edges.push_back({node1, node2, cost});
    nodes[node1].neighbors.emplace_back(node2, cost);
    nodes[node2].neighbors.emplace_back(node1, cost);
}

std::vector<double> Graph::dijkstra(int start) {
    int n = nodes.size();
    std::vector<double> dist(n, std::numeric_limits<double>::infinity());
    dist[start] = 0;

    std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, std::greater<std::pair<double, int>>> pq;
    pq.push({0, start});

    while (!pq.empty()) {
        int u = pq.top().second;
        pq.pop();

        for (const auto& neighbor : nodes[u].neighbors) {
            int v = neighbor.first;
            double weight = neighbor.second;
            double dist_v = dist[u] + weight;
            if (dist_v < dist[v]) {
                dist[v] = dist_v;
                pq.push({dist_v, v});
            }
        }
    }

    return dist;
}

std::vector<int> Graph::shortestPath(int start, int goal) {
    std::vector<double> dist = dijkstra(start);
    std::vector<int> path;
    int u = goal;
    if (dist[u] == std::numeric_limits<double>::infinity()) return path;

    while (u != start) {
        path.push_back(u);
        double min_cost = std::numeric_limits<double>::infinity();
        int prev_u = -1;
        for (const auto& neighbor : nodes[u].neighbors) {
            int v = neighbor.first;
            double weight = neighbor.second;
            if (dist[v] + weight == dist[u] && weight < min_cost) {
                min_cost = weight;
                prev_u = v;
            }
        }
        u = prev_u;
    }
    path.push_back(start);
    std::reverse(path.begin(), path.end());
    return path;
}

// PathPlanner 클래스 구현
PathPlanner::PathPlanner() : nh_("~"), path_planned_(false){
    pose_sub_ = nh_.subscribe("/robot_pose", 1, &PathPlanner::poseCallback, this);
    binarymap_sub_ = nh_.subscribe("/binary_map", 1, &PathPlanner::binaryMapCallback, this);
    path_pub_ = nh_.advertise<nav_msgs::Path>("/planned_path", 1);

    has_start_pose_ = false;
    has_goal_pose_ = false;
    has_binary_map_ = false;

    char filename[128];
    std::time_t currentTime = std::time(nullptr);
    //std::strftime(filename, sizeof(filename), "path_%Y%m%d_%H%M%S.csv", std::localtime(&currentTime));
    std::strftime(filename, sizeof(filename), "path.csv", std::localtime(&currentTime));

    std::string package_path = ros::package::getPath("image_nav");
    std::string csvs_path = package_path + "/csvs";

    // csvs 폴더가 없으면 생성
    struct stat info;
    if (stat(csvs_path.c_str(), &info) != 0) {
        mkdir(csvs_path.c_str(), 0755);
    }

    file_name_ = csvs_path + "/" + std::string(filename);

    // 파일 스트림 열기
    file_stream_.open(file_name_, std::ios::out | std::ios::trunc);
    if (!file_stream_.is_open()) {
        ROS_ERROR_STREAM("Failed to open " << file_name_ << " for writing");
    }
}

PathPlanner::~PathPlanner() {
    file_stream_.close();
}

void PathPlanner::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    if (!has_start_pose_) {
        start_pose_ = *msg;
        has_start_pose_ = true;
    } else {
        goal_pose_ = *msg;
        has_goal_pose_ = true;
    }

    if (has_start_pose_ && has_goal_pose_ && has_binary_map_) {
        planPath(binary_map_, start_pose_, goal_pose_);
    }
}

void PathPlanner::binaryMapCallback(const sensor_msgs::Image::ConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    binary_map_ = cv_ptr->image;
    has_binary_map_ = true;

    if (has_start_pose_ && has_goal_pose_ && has_binary_map_) {
        planPath(binary_map_, start_pose_, goal_pose_);
    }
}


// isPathCollisionFree 함수 수정
bool PathPlanner::isPathCollisionFree(const cv::Point2i& start, const cv::Point2i& end, const cv::Mat& binaryMap) {
    cv::LineIterator it(binaryMap, start, end, 8);
    bool collision_free_region_found = false;

    for (int k = 0; k < it.count; k++, ++it) {
        uchar pixel_value = binaryMap.at<uchar>(it.pos());

        if (pixel_value == 0) {
            collision_free_region_found = true;
        } else if (collision_free_region_found && pixel_value == 255) {
            // 장애물이 아닌 영역을 지났다가 다시 장애물 영역을 만나면 false를 반환합니다.
            return false;
        }
    }

    return true; // 경로상에 장애물이 없거나, 장애물 영역만 지나갔습니다.
}

void PathPlanner::planPath(const cv::Mat& binaryMap, const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal) {

    // GPS 좌표를 UTM 좌표로 변환하는 함수
    auto gpsToUTM = [](double lat, double lon) -> std::tuple<double, double, int> {
        int zone;
        bool northp;
        double x, y;
        GeographicLib::UTMUPS::Forward(lat, lon, zone, northp, x, y);
        return std::make_tuple(x, y, zone);
    };

    // 위경도 좌표를 기준으로 UTM 좌표로 변환
    double top_left_lat = 37.3417121382694, top_left_lon = 126.731593457396;
    auto [utm_origin_x, utm_origin_y, zone] = gpsToUTM(top_left_lat, top_left_lon);

    int pixel_cal_x = 5;
    int pixel_cal_y = 100;

    // UTM 좌표를 픽셀 좌표로 변환하는 함수
    double pixel_per_meter = 16.1668; // 예시 값, 실제 값으로 대체 필요 *****매우 중요한 파라미터
    auto utmToPixel = [&](double x, double y) -> cv::Point2i {
        int pixel_x = static_cast<int>((x - utm_origin_x) * pixel_per_meter) - pixel_cal_x;//***교정치
        int pixel_y = static_cast<int>((utm_origin_y - y) * pixel_per_meter) - pixel_cal_y;//***교정치
        return cv::Point2i(pixel_x, pixel_y);
    };

    // 시작점과 도착점의 위경도 좌표를 UTM 좌표로 변환
    auto [start_utm_x, start_utm_y, start_zone] = gpsToUTM(start.pose.position.x, start.pose.position.y);
    auto [goal_utm_x, goal_utm_y, goal_zone] = gpsToUTM(goal.pose.position.x, goal.pose.position.y);

    ROS_INFO_STREAM("Goal UTM x: " << goal_utm_x);
    ROS_INFO_STREAM("Goal UTM y: " << goal_utm_y);

    // UTM 좌표를 픽셀 좌표로 변환하여 변수에 저장
    cv::Point2i start_pixel = utmToPixel(start_utm_x, start_utm_y);
    cv::Point2i goal_pixel = utmToPixel(goal_utm_x, goal_utm_y);


    
    // UTM 영역 번호가 다르면 에러 처리
    if (start_zone != zone || goal_zone != zone) {
        ROS_ERROR("Start and goal positions are in different UTM zones. Cannot plan path.");
        return;
    }
    /////////////////////

    ROS_INFO_STREAM("Start pixel: " << start_pixel);
    ROS_INFO_STREAM("Goal pixel: " << goal_pixel);
    ROS_INFO_STREAM("zone: " << zone);
    
    /////////////////////
    // PRM 알고리즘 실행
    Graph graph;
    const int numNodes = 12000;
    const double prmConnectionDistance = 150;

    // 시작점과 도착점을 노드로 추가
    graph.addNode(start_pixel);
    int start_node_idx = 0;
    graph.addNode(goal_pixel);
    int goal_node_idx = graph.nodes.size() - 1;

    // 장애물이 없는 영역 찾기
    std::vector<cv::Point2i> free_areas;
    for (int i = 0; i < binaryMap.rows; ++i) {
        for (int j = 0; j < binaryMap.cols; ++j) {
            if (binaryMap.at<uchar>(i, j) == 0) { // 0이면 장애물이 없는 영역
                free_areas.emplace_back(j, i); // cv::Mat은 (row, col) 순서로 접근
            }
        }
    }

    // 무작위 노드 생성
    std::random_shuffle(free_areas.begin(), free_areas.end()); // free_areas를 무작위로 섞음
    for (int i = 0; i < numNodes && i < free_areas.size(); i++) {
        graph.addNode(free_areas[i]);
    }
    /*
    ROS_INFO("Graph Nodes:");
    for (const auto& node : graph.nodes) {
        ROS_INFO_STREAM("Node Coordinate: (" << node.coord.x << ", " << node.coord.y << ")");
    }
    */
    ROS_INFO_STREAM("start: (" << graph.nodes[start_node_idx].coord.x << ", " << graph.nodes[start_node_idx].coord.y << ")");
    ROS_INFO_STREAM("goal: (" << graph.nodes[goal_node_idx].coord.x << ", " << graph.nodes[goal_node_idx].coord.y << ")");
    ROS_INFO_STREAM("start idx in PROGRAM: " << start_node_idx);
    ROS_INFO_STREAM("goal idx in PROGRAM: " << goal_node_idx);
    // 간선 추가 로직 수정
    for (int i = 0; i < graph.nodes.size(); i++) {
        for (int j = i + 1; j < graph.nodes.size(); j++) {
            cv::Point2i node1 = graph.nodes[i].coord;
            cv::Point2i node2 = graph.nodes[j].coord;
            double distance = cv::norm(node1 - node2);

            if (distance < prmConnectionDistance) {
                if (i == start_node_idx || i == goal_node_idx || j == start_node_idx || j == goal_node_idx) {
                    // 출발점과 도착점에 연결되는 간선에 대해 수정된 충돌 검사를 수행합니다.
                    if (isPathCollisionFree(node1, node2, binaryMap)) {
                        graph.addEdge(i, j, distance);
                    }
                } else {
                    // 나머지 노드들 사이의 간선은 기존 로직대로 충돌 검사를 수행합니다.
                    cv::LineIterator it(binaryMap, node1, node2, 8);
                    bool collision_free = true;
                    for (int k = 0; k < it.count; k++, ++it) {
                        if (binaryMap.at<uchar>(it.pos()) == 255) {
                            collision_free = false;
                            break;
                        }
                    }
                    if (collision_free) {
                        graph.addEdge(i, j, distance);
                    }
                }
            }
        }
    }


    // 시작점과 도착점 사이의 최적 경로 찾기
    std::vector<int> path_indices = graph.shortestPath(start_node_idx, goal_node_idx);
    


    // 경로가 생성되었는지 확인
    if (path_indices.empty()) {
        ROS_WARN("Path could not be found. Make sure the start and goal locations are reachable.");
        // PRM 시각화
        visualizePRM(graph, binaryMap, path_indices); //이거 키면, planned_path 안나옴!!! 요주의 
        return; // 경로를 찾지 못했으므로 여기서 함수를 종료합니다.
    } else {
        // 경로를 통해 좌표를 추출하고, 후속 처리를 계속합니다.
        std::vector<cv::Point2i> path;
        for (int idx : path_indices) {
            path.push_back(graph.nodes[idx].coord);
        }
        // (여기에 경로 처리 및 출력 로직을 계속 추가합니다.)

         // 픽셀 좌표를 UTM 좌표로 변환하는 함수
        auto pixelToUTM = [&](const cv::Point2i& pixel) -> std::pair<double, double> {
            double x = utm_origin_x + (static_cast<double>(pixel.x) + pixel_cal_x) / pixel_per_meter;
            double y = utm_origin_y - (static_cast<double>(pixel.y) + pixel_cal_y)/ pixel_per_meter;
            
            return std::make_pair(x, y);
        };


        // UTM 좌표를 위경도 좌표로 변환하는 함수
        auto utmToGPS = [](double easting, double northing, int zone) -> std::pair<double, double> {
            double lat, lon;
            bool northp = true;
            GeographicLib::UTMUPS::Reverse(zone, northp, easting, northing, lat, lon);
            return std::make_pair(lat, lon); // 라디안 값을 도 단위로 변환
        };

        // 경로 결과를 위경도 좌표로 변환
        std::vector<geometry_msgs::PoseStamped> path_poses;
        for (const auto& point : path) {
            auto [utm_x, utm_y] = pixelToUTM(point);
            //ROS_INFO_STREAM("UTM x: " << std::fixed << std::setprecision(6) << utm_x);
            //ROS_INFO_STREAM("UTM y: " << std::fixed << std::setprecision(6) << utm_y);
            auto [lat, lon] = utmToGPS(utm_x, utm_y, zone);
            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = "/map";
            pose.pose.position.x = lat;
            pose.pose.position.y = lon;
            path_poses.push_back(pose);
            ROS_INFO_STREAM("path: " << point);
        }

            // 경로 결과를 CSV 파일에 기록
            if (file_stream_.is_open() && !path_planned_) {
                for (const auto& pose : path_poses) {
                    file_stream_ << std::fixed << std::setprecision(9) << pose.pose.position.x << "," << pose.pose.position.y << std::endl;
                }
                ROS_INFO_STREAM("Path saved to " << file_name_);
                path_planned_ = true; // 파일 쓰기 완료 후 플래그 설정
            } else if (path_planned_) {
                ROS_INFO("Path planning already completed, not writing to CSV.");
            } else {
                ROS_ERROR_STREAM("Failed to write to " << file_name_);
            }


            // 경로 결과를 토픽으로 발행
            publishPath(path_poses);


            ROS_INFO_STREAM("zone: " << zone);
            //ROS_INFO_STREAM("UTM x: " << utm_x);
            //ROS_INFO_STREAM("UTM y: " << utm_y);


            // 경로 계획 완료
            path_planned_ = true;


            // PRM 시각화
            visualizePRM(graph, binaryMap, path_indices); //이거 키면, planned_path 안나옴!!!
    }

}

void PathPlanner::publishPath(const std::vector<geometry_msgs::PoseStamped>& path) {
    nav_msgs::Path path_msg;
    path_msg.header.frame_id = "/map";
    path_msg.poses = path;

    ROS_INFO_STREAM("Planned Path:");
    for (const auto& pose : path) {
        ROS_INFO_STREAM("- Position: (" << pose.pose.position.x << ", " << pose.pose.position.y << ")");
    }

    path_pub_.publish(path_msg);
}