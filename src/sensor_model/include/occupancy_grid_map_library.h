#pragma once

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Geometry>
#include <algorithm>
#include <vector>
#include <cmath>
#include <yaml-cpp/yaml.h>
#include <nav_msgs/OccupancyGrid.h>

struct BinInfo{

    double range;
    double wx;
    double wy;

    BinInfo() = default;
    BinInfo(double range, double wx, double wy) : range(range), wx(wx), wy(wy) {}
};

class OccupancyGridMapLibrary{
    private:
        // We define the parameters
        std::string parameters_file_name = "/home/ericga/MASTER_THESIS/TGM_WS/src/sensor_model/params/occupancy_grid_map.yaml";
        float distance_margin_;
        float gridmap_size_;
        float origin_x_;
        float origin_y_;
        float resolution_;
        double width_;
        double height_;
        double angular_resolution;
        double UNKNOWN;  
        double OBSTACLE;
        double FREE_SPACE;
        geometry_msgs::Pose scan_origin_;
        geometry_msgs::Pose robot_pose_;
    public:
    OccupancyGridMapLibrary();
    void loadParametersOccupancy(const std::string& filename);//
    void initializeGrid(nav_msgs::OccupancyGrid& occupancyGrid_);
    std::pair<int, int> mapToGrid(double mx, double my);//
    void raytrace(double x0, double y0, double x1, double y1, int value, nav_msgs::OccupancyGrid& occupancyGrid_);//
    void setCellValue(double wx, double wy, unsigned char value, nav_msgs::OccupancyGrid& occupancyGrid_);//
     void processPointCloudsIfUpdated( bool rawPointCloudUpdated, 
                                       bool obstaclePointCloudUpdated,
                                       sensor_msgs::PointCloud2::ConstPtr latestRawPointCloud,
                                       sensor_msgs::PointCloud2::ConstPtr latestObstaclePointCloud,
                                       std::vector<std::vector<BinInfo>> &obstaclePointCloudAngleBins,
                                       std::vector<std::vector<BinInfo>> &rawPointCloudAngleBins);//
    void updateWithPointCloud( const sensor_msgs::PointCloud2::ConstPtr& rawPointCloud,
                               const sensor_msgs::PointCloud2::ConstPtr& obstaclePointCloud,
                               const geometry_msgs::Pose &robotPose,
                               const geometry_msgs::Pose &scanOrigin,
                               std::vector<std::vector<BinInfo>> &obstaclePointCloudAngleBins,
                               std::vector<std::vector<BinInfo>> &rawPointCloudAngleBins);//
    void initializeFreeSpace(   std::vector<std::vector<BinInfo>> &obstaclePointCloudAngleBins,
                                std::vector<std::vector<BinInfo>> &rawPointCloudAngleBins,
                                nav_msgs::OccupancyGrid& occupancyGrid_);
    void fillUnknownCells(std::vector<std::vector<BinInfo>> &obstaclePointCloudAngleBins,
                            std::vector<std::vector<BinInfo>> &rawPointCloudAngleBins,
                            nav_msgs::OccupancyGrid& occupancyGrid_);
    void fillOccupiedCells(std::vector<std::vector<BinInfo>> &obstaclePointCloudAngleBins,
                           std::vector<std::vector<BinInfo>> &rawPointCloudAngleBins,
                           nav_msgs::OccupancyGrid& occupancyGrid_);
    void clearOccupancyGrid(nav_msgs::OccupancyGrid& occupancyGrid_);
    
};